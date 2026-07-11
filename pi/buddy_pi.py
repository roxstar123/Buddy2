#!/usr/bin/env python3
"""
buddy_pi.py — Buddy v2.0 (Raspberry Pi 3 edition)
─────────────────────────────────────────────────────────────────────────────
Replaces the fried ESP32-CAM. Speaks the exact same protocol to the Buddy
hub server, so server.js / client.js / index.html need no changes.

Hardware:
  USB camera            → any /dev/video* UVC camera (default /dev/video0)
  PCA9685 servo driver  → Pi I2C: SDA=GPIO2 (pin 3), SCL=GPIO3 (pin 5),
                          VCC=3.3V, GND=GND. Address 0x40 (pins tied low).
    3 continuous-rotation wheel servos (pots decoupled), tangent to chassis
    center, positioned at 0°/120°/240° (clockwise from front, top-down):
      Servo 0 → PCA channel 0  (front, 0°)
      Servo 1 → PCA channel 1  (120°)
      Servo 2 → PCA channel 2  (240°)
  Speaker               → Pi 3.5mm jack / HDMI / USB audio (played via ffplay)

Protocol (same as ESP32 firmware):
  Connects wss://host/ws?id=BDY-XXXXX&role=device
  Sends   binary  0x01 + JPEG          — video frames (only while a browser
                                          client is paired)
  Receives binary 0x02 + audio chunk   — hold-to-talk audio from the browser
                                          (MediaRecorder webm/opus chunks)
  Receives text JSON:
    { "type": "client_connected" / "client_disconnected" }
    { "type": "cmd",   "dir": "fwd"|"back"|"left"|"right"|"stop" }
    { "type": "servo", "ch": 0-2, "angle": 0-180 }

Dependencies (see pi/README.md):
  sudo apt install python3-opencv ffmpeg i2c-tools
  pip3 install websockets smbus2
─────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
import subprocess
import sys

import cv2

try:
    import websockets
except ImportError:
    sys.exit("missing dependency: pip3 install websockets")

log = logging.getLogger("buddy")

# ─── Defaults (match the ESP32 config block) ─────────────────────────────────

DEFAULT_HOST = "buddy-production-948c.up.railway.app"
DEFAULT_PORT = 443
DEFAULT_ID   = "BDY-00001"

# ─── Frame type bytes ─────────────────────────────────────────────────────────

TYPE_VIDEO = 0x01
TYPE_AUDIO = 0x02

# ─── PCA9685 servo driver (minimal register-level driver over smbus2) ────────

SERVO_FREQ = 50
NUM_SERVOS = 3
PCA_ADDR   = 0x40
PCA_OSC_HZ = 27_000_000

# ─── Continuous-rotation servo control (pulse width in µs) ────────────────────
# These servos are pot-decoupled conversions: the electronics still think
# they're positional, but the pot is frozen, so pulse width maps to *speed*:
# 1500µs = stop, shorter = spin one way, longer = the other. The speed curve
# saturates fast — ±FULL_SPEED_US from neutral is already flat-out, and
# pushing far past that just slams the motor against its own controller
# (that's the grinding noise). So all commands stay inside a narrow band.

STOP_US       = 1500   # nominal stop pulse
FULL_SPEED_US = 200    # pulse offset at 100% commanded speed (lower = slower robot)
DEADBAND      = 0.05   # |speed| below this snaps to the exact stop pulse (no buzzing)

# Per-channel stop trim, in µs. Each servo's true stop is wherever its frozen
# pot happens to sit, not exactly 1500µs. Calibrate per wheel: send
# { "type": "servo", "ch": i, "angle": 90 } and nudge the angle until that
# wheel fully stops — each angle step = 2µs, so trim = (stop_angle - 90) * 2.
STOP_TRIM_US = [0, 0, 0]

# ─── Drive geometry ───────────────────────────────────────────────────────────
# Three wheels tangent to the chassis circle — TWO IN FRONT, ONE IN BACK
# (top-down view, positions measured clockwise from straight ahead):
#
#     FL ╱     ╲ FR       front-left  at 300°  → PCA channel 0
#                          front-right at  60°  → PCA channel 1
#         ───              back        at 180°  → PCA channel 2
#          B
#
# Wheel drive-direction unit vector = (-sin(pos), cos(pos)) in the
# (forward, right) frame; wheel speed = dot(drive_dir, commanded velocity).
#   fwd/back   → both front wheels drive (mirrored mounting, so opposite
#                signs = same direction of travel), back wheel stays still
#   left/right → back wheel does most of the work, fronts assist
WHEEL_FWD   = [ 0.8660254, -0.8660254,  0.0]   # FL, FR, B
WHEEL_RIGHT = [ 0.5,        0.5,       -1.0]


class Servos:
    """PCA9685 over the Pi's I2C bus. Degrades gracefully if absent."""

    _MODE1     = 0x00
    _PRESCALE  = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self):
        self.bus = None
        try:
            from smbus2 import SMBus
            self.bus = SMBus(1)  # Pi 3: I2C bus 1 (GPIO2/GPIO3)
            self._init_chip()
            self.center()
            log.info("[srv] PCA9685 ready — %d servos centred", NUM_SERVOS)
        except Exception as e:
            self.bus = None
            log.warning("[srv] PCA9685 unavailable (%s) — drive disabled", e)

    def _init_chip(self):
        prescale = round(PCA_OSC_HZ / (4096 * SERVO_FREQ)) - 1
        self.bus.write_byte_data(PCA_ADDR, self._MODE1, 0x00)         # wake
        self.bus.write_byte_data(PCA_ADDR, self._MODE1, 0x10)         # sleep
        self.bus.write_byte_data(PCA_ADDR, self._PRESCALE, prescale)  # set freq
        self.bus.write_byte_data(PCA_ADDR, self._MODE1, 0x00)         # wake
        import time
        time.sleep(0.005)
        self.bus.write_byte_data(PCA_ADDR, self._MODE1, 0xA0)         # restart + auto-inc

    def _write_us(self, ch: int, us: float):
        """Pulse width in µs → PCA9685 ticks (4096 ticks per 20ms frame @ 50Hz)."""
        ticks = round(us * SERVO_FREQ * 4096 / 1_000_000)
        try:
            self.bus.write_i2c_block_data(
                PCA_ADDR, self._LED0_ON_L + 4 * ch,
                [0, 0, ticks & 0xFF, ticks >> 8])
            log.debug("[srv] ch%d → %.0fµs", ch, us)
        except OSError as e:
            log.warning("[srv] i2c write failed: %s", e)

    def set_speed(self, ch: int, speed: float):
        """speed -1..1 → narrow pulse band around this channel's stop point."""
        if self.bus is None or not (0 <= ch < NUM_SERVOS):
            return
        speed = max(-1.0, min(1.0, speed))
        if abs(speed) < DEADBAND:
            speed = 0.0
        self._write_us(ch, STOP_US + STOP_TRIM_US[ch] + speed * FULL_SPEED_US)

    def set(self, ch: int, angle: int):
        """Calibration path for { "type": "servo" } messages.
        angle 90 = nominal stop; each degree = 2µs of pulse width,
        so 45/135 ≈ half speed either way. Used to find STOP_TRIM_US."""
        if self.bus is None or not (0 <= ch < NUM_SERVOS):
            return
        angle = max(0, min(180, angle))
        self._write_us(ch, STOP_US + (angle - 90) * 2)

    def center(self):
        """All stop."""
        for i in range(NUM_SERVOS):
            self.set_speed(i, 0.0)

    def drive(self, vx: float, vy: float):
        for i in range(NUM_SERVOS):
            self.set_speed(i, WHEEL_FWD[i] * vx + WHEEL_RIGHT[i] * vy)

    def cmd(self, direction: str):
        vx, vy = {
            "fwd":   ( 1,  0),
            "back":  (-1,  0),
            "left":  ( 0, -1),
            "right": ( 0,  1),
        }.get(direction, (0, 0))  # stop / unknown → all stop
        self.drive(vx, vy)


# ─── Speaker (hold-to-talk audio from the browser) ────────────────────────────
# The browser sends MediaRecorder chunks (webm/opus). Each press of the speak
# button starts a fresh MediaRecorder, whose first chunk begins with the EBML
# magic — when we see it, restart ffplay so the demuxer gets a clean stream.

EBML_MAGIC = b"\x1a\x45\xdf\xa3"


class Speaker:
    def __init__(self):
        self.proc = None

    def _spawn(self):
        self.stop()
        try:
            self.proc = subprocess.Popen(
                ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", "-i", "pipe:0"],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            log.warning("[spk] ffplay not found — audio playback disabled "
                        "(sudo apt install ffmpeg)")
            self.proc = None

    def feed(self, chunk: bytes):
        if chunk.startswith(EBML_MAGIC) or self.proc is None or self.proc.poll() is not None:
            self._spawn()
        if self.proc and self.proc.stdin:
            try:
                self.proc.stdin.write(chunk)
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError):
                self.proc = None

    def stop(self):
        if self.proc:
            try:
                self.proc.stdin.close()
            except Exception:
                pass
            self.proc.terminate()
            self.proc = None


# ─── Camera (USB UVC via OpenCV) ──────────────────────────────────────────────

class Camera:
    def __init__(self, device: int, width: int, height: int, fps: int, quality: int):
        self.quality = quality
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # Ask the camera for MJPG so the USB link / Pi CPU isn't the bottleneck
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always grab the freshest frame
        self.ready = self.cap.isOpened()
        if self.ready:
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            log.info("[cam] ready — /dev/video%d @ %dx%d", device, w, h)
        else:
            log.error("[cam] open FAILED — is the USB camera plugged in? (ls /dev/video*)")

    def grab_jpeg(self) -> bytes | None:
        """Blocking read + JPEG encode. Run in an executor thread."""
        ok, frame = self.cap.read()
        if not ok:
            return None
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return jpg.tobytes() if ok else None

    def release(self):
        if self.ready:
            self.cap.release()


# ─── Device client ────────────────────────────────────────────────────────────

class Buddy:
    def __init__(self, args):
        self.args = args
        self.servos  = Servos()
        self.speaker = Speaker()
        self.camera  = Camera(args.camera, args.width, args.height, args.fps, args.quality)
        self.peer_online = False

    @property
    def url(self) -> str:
        scheme = "wss" if self.args.port == 443 else "ws"
        return f"{scheme}://{self.args.host}:{self.args.port}/ws?id={self.args.id}&role=device"

    # ── Camera task ──────────────────────────────────────────────────────────
    async def camera_task(self, ws):
        loop = asyncio.get_running_loop()
        interval = 1.0 / self.args.fps
        while True:
            start = loop.time()
            if self.peer_online and self.camera.ready:
                jpg = await loop.run_in_executor(None, self.camera.grab_jpeg)
                if jpg:
                    await ws.send(bytes([TYPE_VIDEO]) + jpg)
            await asyncio.sleep(max(0.0, interval - (loop.time() - start)))

    # ── Incoming messages ────────────────────────────────────────────────────
    async def recv_task(self, ws):
        async for msg in ws:
            if isinstance(msg, bytes):
                if len(msg) > 1 and msg[0] == TYPE_AUDIO:
                    self.speaker.feed(msg[1:])
                continue

            try:
                doc = json.loads(msg)
            except json.JSONDecodeError:
                continue
            t = doc.get("type", "")

            if t == "client_connected":
                self.peer_online = True
                log.info("[ws]  browser online")
            elif t == "client_disconnected":
                self.peer_online = False
                self.servos.center()
                self.speaker.stop()
                log.info("[ws]  browser offline")
            elif t == "servo":
                self.servos.set(int(doc.get("ch", 0)), int(doc.get("angle", 90)))
            elif t == "cmd":
                self.servos.cmd(doc.get("dir", "stop"))

    # ── Main connect/reconnect loop ──────────────────────────────────────────
    async def run(self):
        log.info("[buddy] booting — id: %s", self.args.id)
        while True:
            try:
                log.info("[ws]  → %s", self.url)
                async with websockets.connect(
                    self.url,
                    ping_interval=15,   # matches ESP32 heartbeat
                    ping_timeout=10,
                    max_size=None,
                ) as ws:
                    log.info("[ws]  connected — id: %s", self.args.id)
                    try:
                        cam = asyncio.create_task(self.camera_task(ws))
                        await self.recv_task(ws)
                    finally:
                        cam.cancel()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.warning("[ws]  disconnected (%s) — retrying in 3s", e)
            self.peer_online = False
            self.servos.center()
            self.speaker.stop()
            await asyncio.sleep(3)

    def shutdown(self):
        self.servos.center()
        self.speaker.stop()
        self.camera.release()


# ─── Entry point ──────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Buddy device client for Raspberry Pi 3")
    p.add_argument("--host",    default=DEFAULT_HOST, help="hub server hostname")
    p.add_argument("--port",    default=DEFAULT_PORT, type=int, help="hub server port (443 = wss)")
    p.add_argument("--id",      default=DEFAULT_ID, help="buddy ID, e.g. BDY-00001")
    p.add_argument("--camera",  default=0, type=int, help="V4L2 device index (/dev/videoN)")
    p.add_argument("--width",   default=640, type=int)
    p.add_argument("--height",  default=480, type=int)
    p.add_argument("--fps",     default=15, type=int)
    p.add_argument("--quality", default=70, type=int, help="JPEG quality 0-100")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    args.id = args.id.upper().strip()
    if not args.id.startswith("BDY-"):
        args.id = f"BDY-{args.id}"
    return args


def main():
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(message)s",
    )

    buddy = Buddy(args)
    loop = asyncio.new_event_loop()
    main_task = loop.create_task(buddy.run())

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, main_task.cancel)
        except NotImplementedError:
            pass  # non-POSIX platforms

    try:
        loop.run_until_complete(main_task)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        buddy.shutdown()
        loop.close()
        log.info("[buddy] stopped")


if __name__ == "__main__":
    main()
