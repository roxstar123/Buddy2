# Buddy on Raspberry Pi 3

The device side of Buddy, replacing the fried ESP32-CAM. Speaks the same
WebSocket protocol to the hub server, so `server.js` / the web UI are
unchanged — enter the same `BDY-XXXXX` ID in the browser and drive.

## Hardware

| Part | Connection |
|---|---|
| USB camera | any USB port (`/dev/video0`) |
| PCA9685 servo driver | SDA → GPIO2 (pin 3), SCL → GPIO3 (pin 5), VCC → 3.3V (pin 1), GND → GND (pin 6) |
| Wheel servos ×3 | PCA channels 0 / 1 / 2 (front / 120° / 240°, same as before) |
| Speaker (optional) | 3.5mm jack, HDMI, or USB audio |

> The PCA9685's servo power (V+) still needs its own 5V supply, same as with
> the ESP32 — don't power servos from the Pi's 5V rail.

## Setup (Raspberry Pi OS)

```bash
# 1. Enable I2C
sudo raspi-config          # Interface Options → I2C → Enable

# 2. System packages
sudo apt update
sudo apt install -y python3-opencv ffmpeg i2c-tools python3-pip

# 3. Python packages
pip3 install websockets smbus2

# 4. Sanity checks
i2cdetect -y 1             # should show 40 (the PCA9685)
ls /dev/video*             # should show video0 (the USB camera)
```

## Run

```bash
python3 buddy_pi.py --id BDY-00001
```

Defaults match the old firmware config (Railway host, wss on 443). Options:

```
--host    hub server hostname          (default: buddy-production-948c.up.railway.app)
--port    hub port, 443 = wss          (default: 443)
--id      buddy ID                     (default: BDY-00001)
--camera  /dev/videoN index            (default: 0)
--width / --height / --fps / --quality video settings (default: 640x480 @ 15fps, q70)
-v        verbose logging (per-servo writes, etc.)
```

For a local hub: `python3 buddy_pi.py --host 192.168.1.50 --port 3000`

## Servo calibration

Same as before — the trim table moved to `SERVO_TRIM` in `buddy_pi.py`.
From the browser console (or any WS client), send
`{ "type": "servo", "ch": i, "angle": 90 }` and nudge the angle until that
wheel stops creeping, then set `SERVO_TRIM[i] = measured_stop_angle - 90`.

## Start on boot (systemd)

```bash
sudo tee /etc/systemd/system/buddy.service > /dev/null <<'EOF'
[Unit]
Description=Buddy device client
After=network-online.target
Wants=network-online.target

[Service]
User=pi
ExecStart=/usr/bin/python3 /home/pi/Buddy2/pi/buddy_pi.py --id BDY-00001
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable --now buddy
journalctl -u buddy -f      # watch logs
```

(Adjust the path/User if you cloned the repo elsewhere.)

## What changed vs the ESP32

- **Camera**: onboard OV2640 → USB UVC camera via OpenCV. Default bumped to
  640×480 (the Pi can afford it); use `--width 320 --height 240` to match the
  old QVGA feel if latency matters.
- **Speaker**: the hold-to-talk audio path is now actually enabled — incoming
  browser audio plays through the Pi's audio output via `ffplay`. (It was
  disabled on the ESP32-CAM.)
- **Servos**: same PCA9685, same channels, same drive math — just wired to
  the Pi's I2C pins instead of GPIO14/13.
- The `firmware/` directory and `flash.html` flashing tool are now legacy.
