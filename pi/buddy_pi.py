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
    center — two in front, one sideways in back:
      front-left  → PCA channel 0
      front-right → PCA channel 1
      back        → PCA channel 2
    Servos get NO signal at idle — pulses only while a button is held.
  Microphone            → USB camera mic (or any ALSA input). Default ALSA device
                          hw:1 — run `arecord -l` on the Pi to find yours.
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
# they're positional, but the pot is frozen, so ANY pulse makes the board
# drive the motor toward wherever the frozen pot says it is. A "stop pulse"
# only truly stops a wheel if it exactly matches that servo's frozen pot
# reading — anything else means permanent creep, buzz, or grind.
#
# So: when idle we send NO PULSE AT ALL (PCA9685 channel fully off). With no
# signal, the servo electronics do nothing — the wheel sits silent and limp.
# Pulses are only sent while a drive command is actively held; releasing the
# button (or losing the connection) cuts all outputs.

# Per-channel pulse that holds the wheel still, i.e. the pulse matching each
# servo's frozen pot reading. Only affects speed balance while DRIVING (we
# never send it at idle). Calibrate per wheel — see README.
# Measured per-wheel: [front-left (ch0), front-right (ch1), back (ch2)]
NEUTRAL_US = [1489, 1494, 1494]   # FL: 89 · FR: 89.5 · back: 89.5 (stop angles)

DRIVE_US = 100    # offset from neutral at 100% commanded speed (lower = slower)
DEADBAND = 0.05   # |speed| below this cuts the channel entirely

# ─── Stall-protection workaround: command dithering ───────────────────────────
# With the pot frozen, the servo firmware sees a position error that never
# shrinks, decides the motor is stalled, and cuts power after a few seconds.
# Observed behavior: sending a DIFFERENT command re-arms it briefly; repeating
# the same value (or gaps of silence) does not. So while any channel is
# active, we alternate its pulse between the wanted value and a slightly
# different one — the servo keeps seeing "new order", keeps re-arming, and
# the wheel speed barely changes.
DITHER_US = 60    # how far the alternate pulse deviates (away from neutral)
DITHER_MS = 0     # how often to alternate. 0 = disable dithering (default: off).

# Rest cycling: the protection clears after a LONG signal absence (~1-2s),
# so drive in bursts — REST_ON_MS of driving, then REST_OFF_MS of silence.
# Motion pulses, but never permanently stalls. Tune REST_ON_MS comfortably
# below your measured time-to-stall. 0 = disable resting.
REST_ON_MS  = 4000  # measured cutoff ≈ 5s — rest comfortably before it
REST_OFF_MS = 50    # brief break between drive windows (0 = never rest)

# Rest style: True = command each channel's NEUTRAL_US during the rest —
# the firmware sees "target reached" (pulse matches the centered pot) and
# resets its stall detector far faster than silence does. False (or
# --rest-silence) = cut the signal entirely during the rest instead.
# NOTE: rest cycling applies only to D-pad driving; browser-console
# calibration commands always get one raw steady pulse.
REST_NEUTRAL = True

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

# Per-wheel direction sign — flips a wheel whose real-world spin is mirrored
# vs the math above (depends on which way the servo is mounted). Field-set:
# both front wheels run reversed on this chassis.
WHEEL_DIR   = [-1.0, -1.0, 1.0]                # FL, FR, B


class Servos:
    """PCA9685 over the Pi's I2C bus. Degrades gracefully if absent."""

    _MODE1     = 0x00
    _PRESCALE  = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self):
        self.bus = None
        self.targets = [0.0] * NUM_SERVOS  # commanded drive speed per channel
        try:
            from smbus2 import SMBus
            self.bus = SMBus(1)  # Pi 3: I2C bus 1 (GPIO2/GPIO3)
            self._init_chip()
            self.stop_all()
            log.info("[srv] PCA9685 ready — all outputs off until driven")
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

    def _off(self, ch: int):
        """Cut the channel entirely — no pulse, servo goes idle/limp.
        Bit 4 of LED_OFF_H is the PCA9685 full-off flag."""
        try:
            self.bus.write_i2c_block_data(
                PCA_ADDR, self._LED0_ON_L + 4 * ch, [0, 0, 0, 0x10])
            log.debug("[srv] ch%d → off", ch)
        except OSError as e:
            log.warning("[srv] i2c write failed: %s", e)

    def set_speed(self, ch: int, speed: float):
        """speed -1..1 → pulse around this channel's neutral; 0 → output off."""
        if self.bus is None or not (0 <= ch < NUM_SERVOS):
            return
        speed = max(-1.0, min(1.0, speed))
        if abs(speed) < DEADBAND:
            speed = 0.0
        self.targets[ch] = speed
        if speed == 0.0:
            self._off(ch)
        else:
            self._write_us(ch, NEUTRAL_US[ch] + speed * DRIVE_US)

    # Dither/rest support — applies ONLY to D-pad drive targets. Calibration
    # pulses (browser-console "servo" messages) are always raw and steady so
    # pot centering gets clean, predictable feedback.
    def dither_active(self) -> bool:
        return any(self.targets)

    def dither_write(self, phase: bool):
        for i in range(NUM_SERVOS):
            if not self.targets[i]:
                continue
            base = NEUTRAL_US[i] + self.targets[i] * DRIVE_US
            if phase:
                base += DITHER_US if base >= NEUTRAL_US[i] else -DITHER_US
            self._write_us(i, base)

    def rest_pause(self):
        """Rest all driven channels: silence them, or (REST_NEUTRAL) hold
        them at their neutral pulse so the firmware sees 'target reached'."""
        for i in range(NUM_SERVOS):
            if self.targets[i]:
                if REST_NEUTRAL:
                    self._write_us(i, NEUTRAL_US[i])
                else:
                    self._off(i)

    def set(self, ch: int, angle: float):
        """Calibration path for { "type": "servo" } messages.
        Maps angle 0-180 linearly onto 500-2500µs (so 90 = 1500µs,
        1° ≈ 11µs; fractional angles allowed for fine steps). The µs value
        is logged — when you find the angle where the wheel stops, put that
        µs into NEUTRAL_US. angle < 0 cuts the channel (off)."""
        if self.bus is None or not (0 <= ch < NUM_SERVOS):
            return
        if angle < 0:
            self._off(ch)
            return
        us = 500 + min(180.0, angle) * 2000.0 / 180.0
        log.info("[srv] ch%d calibrate → %.0fµs (angle %.1f)", ch, us, angle)
        self._write_us(ch, us)

    def stop_all(self):
        """All outputs off — nothing is driven, nothing hums."""
        self.targets = [0.0] * NUM_SERVOS
        if self.bus is None:
            return
        for i in range(NUM_SERVOS):
            self._off(i)

    def drive(self, vx: float, vy: float):
        for i in range(NUM_SERVOS):
            self.set_speed(i, WHEEL_DIR[i] * (WHEEL_FWD[i] * vx + WHEEL_RIGHT[i] * vy))

    def cmd(self, direction: str):
        vx, vy = {
            "fwd":   ( 1,  0),
            "back":  (-1,  0),
            "left":  ( 0, -1),
            "right": ( 0,  1),
        }.get(direction, (0, 0))  # stop / unknown → all stop
        self.drive(vx, vy)


# ─── Speaker (hold-to-talk audio from the browser) ────────────────────────────
# The browser sends MediaRecorder chunks — webm/opus from Chrome, ogg/opus
# from Firefox. Each press of the speak button starts a fresh MediaRecorder,
# whose first chunk begins with a container magic; when we see one, restart
# ffplay with the right format hint so the demuxer starts instantly (no
# probing delay — a short press never delivers enough data to finish a probe).
#
# All pipe I/O happens on a worker thread behind a bounded queue: a wedged or
# codec-confused player drops audio, but can NEVER block the asyncio loop.
# (A blocked loop stops answering websocket pings → keepalive timeout → the
# whole connection drops. Audio must never be able to do that.)

EBML_MAGIC = b"\x1a\x45\xdf\xa3"   # webm/matroska (Chrome)
OGG_MAGIC  = b"OggS"               # ogg (Firefox)

import queue as _queue
import threading as _threading


class Speaker:
    def __init__(self, dump_path: str | None = None):
        self.q = _queue.Queue(maxsize=64)
        self.proc = None
        self.disabled = False
        self.dump_path = dump_path
        self.dump_file = None
        _threading.Thread(target=self._worker, daemon=True, name="spk").start()

    # ── async-loop side: never blocks ─────────────────────────────────────
    def feed(self, chunk: bytes):
        try:
            self.q.put_nowait(chunk)
        except _queue.Full:
            pass  # player is behind — drop audio rather than stall the loop

    def stop(self):
        self.feed(b"")  # empty chunk = kill-player sentinel

    # ── worker-thread side: all blocking lives here ───────────────────────
    def _worker(self):
        while True:
            chunk = self.q.get()
            if not chunk:
                self._kill()
                continue
            if self.disabled:
                continue

            # New-stream detection. NOTE: "OggS" alone is NOT enough — ogg puts
            # it at the start of EVERY page. A stream truly begins only when
            # the page's BOS (beginning-of-stream) flag is set: bit 0x02 of
            # byte 5. Mid-stream pages must be fed to the existing player.
            is_new_webm = chunk.startswith(EBML_MAGIC)
            is_new_ogg  = (chunk.startswith(OGG_MAGIC)
                           and len(chunk) > 5 and (chunk[5] & 0x02))
            is_new_mp4  = len(chunk) > 8 and chunk[4:8] == b"ftyp"  # iOS MediaRecorder
            if is_new_webm:
                log.info("[spk] webm audio stream from browser (%d bytes)", len(chunk))
                self._spawn("matroska")
                self._dump_open()
            elif is_new_ogg:
                log.info("[spk] ogg audio stream from browser (%d bytes)", len(chunk))
                self._spawn("ogg")
                self._dump_open()
            elif is_new_mp4:
                log.info("[spk] mp4 audio stream from browser (%d bytes)", len(chunk))
                self._spawn("mp4")
                self._dump_open()
            elif self.proc is None or self.proc.poll() is not None:
                # Chunk with no live player and no recognizable stream header —
                # log what it actually starts with so unknown formats show up.
                log.warning("[spk] unrecognized audio chunk (%d bytes, header %s)",
                            len(chunk), chunk[:16].hex())
                self._kill()
                continue

            if self.dump_file:
                try:
                    self.dump_file.write(chunk)
                    self.dump_file.flush()
                except OSError:
                    pass

            if self.proc and self.proc.stdin:
                try:
                    self.proc.stdin.write(chunk)
                    self.proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    log.warning("[spk] ffplay pipe broke — player died mid-stream")
                    self._kill()

    def _dump_open(self):
        """--dump-audio: start a fresh capture file for this stream."""
        if not self.dump_path:
            return
        try:
            if self.dump_file:
                self.dump_file.close()
            self.dump_file = open(self.dump_path, "wb")
            log.info("[spk] dumping stream to %s", self.dump_path)
        except OSError as e:
            log.warning("[spk] dump open failed: %s", e)
            self.dump_file = None

    def _spawn(self, fmt: str):
        self._kill()
        # Standard buffering on purpose: low-latency flags (nobuffer /
        # probesize 32) proved too aggressive for chunked network audio.
        cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "warning",
               "-f", fmt, "-i", "pipe:0"]
        try:
            # stderr inherited on purpose: if ffplay can't open an audio
            # output device or decode a stream, that must land in the journal.
            self.proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
            )
            log.info("[spk] ffplay started (pid %d, %s)", self.proc.pid, fmt)
        except FileNotFoundError:
            log.warning("[spk] ffplay not found — audio playback disabled "
                        "(sudo apt install ffmpeg)")
            self.proc = None
            self.disabled = True

    def _kill(self):
        if self.proc:
            try:
                self.proc.stdin.close()
            except Exception:
                pass
            try:
                self.proc.terminate()
            except Exception:
                pass
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


# ─── Microphone (USB camera mic or any ALSA capture device) ──────────────────
# Captures raw PCM16 mono at 16 kHz via arecord and sends 30 ms chunks to the
# browser as TYPE_AUDIO (0x02) binary frames.  The browser's playPCM16() decodes
# raw S16_LE directly — no container/codec needed on this path.

class Microphone:
    RATE        = 16_000
    CHANNELS    = 1
    CHUNK_MS    = 30
    CHUNK_BYTES = RATE * 2 * CHUNK_MS // 1000   # 960 bytes = 480 samples

    def __init__(self, device: str):
        self.device = device
        self.proc: "asyncio.subprocess.Process | None" = None

    async def start(self) -> bool:
        cmd = [
            "arecord",
            "-D", self.device,
            "-f", "S16_LE",
            "-r", str(self.RATE),
            "-c", str(self.CHANNELS),
            "-t", "raw",
        ]
        try:
            self.proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            log.info("[mic] arecord started (pid %d, device %s)", self.proc.pid, self.device)
            return True
        except FileNotFoundError:
            log.warning("[mic] arecord not found — mic disabled (sudo apt install alsa-utils)")
            return False
        except Exception as e:
            log.warning("[mic] failed to start: %s", e)
            return False

    async def read_chunk(self) -> "bytes | None":
        if self.proc is None or self.proc.stdout is None:
            return None
        try:
            return await self.proc.stdout.readexactly(self.CHUNK_BYTES)
        except (asyncio.IncompleteReadError, Exception):
            return None

    def stop(self):
        if self.proc:
            try:
                self.proc.terminate()
            except Exception:
                pass
            self.proc = None


# ─── Device client ────────────────────────────────────────────────────────────

class Buddy:
    def __init__(self, args):
        self.args = args
        self.servos     = Servos()
        self.speaker    = Speaker(dump_path=args.dump_audio)
        self.camera     = Camera(args.camera, args.width, args.height, args.fps, args.quality)
        self.microphone = Microphone(args.mic_device)
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

    # ── Microphone task ───────────────────────────────────────────────────────
    async def mic_task(self, ws):
        if not await self.microphone.start():
            return
        while True:
            chunk = await self.microphone.read_chunk()
            if chunk is None:
                log.warning("[mic] arecord ended — mic task stopped")
                break
            if self.peer_online:
                await ws.send(bytes([TYPE_AUDIO]) + chunk)

    # ── Stall-protection task: dither + rest cycling ─────────────────────────
    # While any channel is active: alternate its pulse (dither) so the servo
    # keeps seeing "new" commands, and every REST_ON_MS go fully silent for
    # REST_OFF_MS — the only thing observed to reliably clear the protection.
    async def burst_task(self):
        phase = False
        driven_s = 0.0
        step = (DITHER_MS if DITHER_MS > 0 else 100) / 1000
        while True:
            if not self.servos.dither_active():
                driven_s = 0.0
                await asyncio.sleep(0.05)
                continue
            await asyncio.sleep(step)
            if not self.servos.dither_active():
                continue
            driven_s += step
            if REST_OFF_MS > 0 and driven_s >= REST_ON_MS / 1000:
                self.servos.rest_pause()
                await asyncio.sleep(REST_OFF_MS / 1000)
                driven_s = 0.0
                phase = False
                if self.servos.dither_active():
                    self.servos.dither_write(phase)  # resume still-held commands
            elif DITHER_MS > 0:
                phase = not phase
                self.servos.dither_write(phase)

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
                self.servos.stop_all()
                self.speaker.stop()
                log.info("[ws]  browser offline")
            elif t == "servo":
                self.servos.set(int(doc.get("ch", 0)), float(doc.get("angle", 90)))
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
                        cam   = asyncio.create_task(self.camera_task(ws))
                        mic   = asyncio.create_task(self.mic_task(ws))
                        burst = asyncio.create_task(self.burst_task())
                        await self.recv_task(ws)
                    finally:
                        cam.cancel()
                        mic.cancel()
                        burst.cancel()
                        self.microphone.stop()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.warning("[ws]  disconnected (%s) — retrying in 3s", e)
            self.peer_online = False
            self.servos.stop_all()
            self.speaker.stop()
            await asyncio.sleep(3)

    def shutdown(self):
        self.servos.stop_all()
        self.speaker.stop()
        self.microphone.stop()
        self.camera.release()


# ─── Entry point ──────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Buddy device client for Raspberry Pi 3")
    p.add_argument("--host",    default=DEFAULT_HOST, help="hub server hostname")
    p.add_argument("--port",    default=DEFAULT_PORT, type=int, help="hub server port (443 = wss)")
    p.add_argument("--id",      default=DEFAULT_ID, help="buddy ID, e.g. BDY-00001")
    p.add_argument("--camera",     default=0,    type=int, help="V4L2 device index (/dev/videoN)")
    p.add_argument("--mic-device", default="hw:1", help="ALSA capture device (run 'arecord -l' to list). Default: hw:1")
    p.add_argument("--width",   default=640, type=int)
    p.add_argument("--height",  default=480, type=int)
    p.add_argument("--fps",     default=15, type=int)
    p.add_argument("--quality", default=70, type=int, help="JPEG quality 0-100")
    # Servo experiment knobs — override the file constants without editing code
    p.add_argument("--drive-us",  default=DRIVE_US, type=int,
                   help="pulse offset from neutral at full speed (default %(default)s)")
    p.add_argument("--dither-us", default=DITHER_US, type=int,
                   help="alternate-pulse deviation in µs (default %(default)s)")
    p.add_argument("--dither-ms", default=DITHER_MS, type=int,
                   help="dither period in ms, 0 = off (default %(default)s)")
    p.add_argument("--rest-on",   default=REST_ON_MS, type=int,
                   help="drive window in ms before resting (default %(default)s)")
    p.add_argument("--rest-off",  default=REST_OFF_MS, type=int,
                   help="rest window in ms, 0 = never rest (default %(default)s)")
    p.add_argument("--rest-neutral", action="store_true",
                   help="rest at the neutral pulse ('target reached') — this is the default")
    p.add_argument("--rest-silence", action="store_true",
                   help="rest with no signal at all instead of the neutral pulse")
    p.add_argument("--neutral",   default=None,
                   help="per-channel neutral pulses, e.g. --neutral 1500,1620,1480")
    p.add_argument("--dump-audio", default=None, metavar="PATH",
                   help="also save incoming browser audio to this file (debug)")
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

    # Apply servo knob overrides to the module-level constants
    globals().update(
        DRIVE_US=args.drive_us,
        DITHER_US=args.dither_us,
        DITHER_MS=args.dither_ms,
        REST_ON_MS=args.rest_on,
        REST_OFF_MS=args.rest_off,
        REST_NEUTRAL=not args.rest_silence,
    )
    if args.neutral:
        vals = [int(v) for v in args.neutral.split(",")]
        if len(vals) != NUM_SERVOS:
            sys.exit(f"--neutral needs {NUM_SERVOS} comma-separated values")
        globals()["NEUTRAL_US"] = vals
    log.info("[srv] drive=%dµs dither=%dµs/%dms rest=%d/%dms neutral=%s",
             DRIVE_US, DITHER_US, DITHER_MS, REST_ON_MS, REST_OFF_MS, NEUTRAL_US)

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
