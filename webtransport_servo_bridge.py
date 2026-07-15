"""
WebTransport (QUIC) → Servo Bridge
Receives compact binary datagrams from the Meta Quest WebXR page
and drives GPIO servos on a Raspberry Pi 3.

Packet format (matches webxr_controller.html):
    1 byte  → command byte (0x01 = spin stepper 1 one revolution)
    3 bytes → servo angles [S1, S2, S3], each 0-180
        byte 0 → S1 angle  (0–180)  pan  / yaw
        byte 1 → S2 angle  (0–180)  tilt / pitch
        byte 2 → S3 angle  (0–180)  roll

Dependencies (install on the Pi):
    pip install aioquic RPi.GPIO

Generate your TLS cert first:
    python setup_cert.py

Run:
    python webtransport_servo_bridge.py

Then open webxr_controller.html on the Meta Quest Browser,
paste the cert hash printed by setup_cert.py, and hit Connect.
"""

import asyncio
import logging
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

# ── Optional GPIO import ──────────────────────────────────────────────────────
SIMULATE = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
except (ImportError, RuntimeError):
    SIMULATE = True
    print("[bridge] RPi.GPIO unavailable — running in SIMULATE mode (no GPIO output).")

# ── aioquic imports ───────────────────────────────────────────────────────────
try:
    from aioquic.asyncio import serve
    from aioquic.asyncio.protocol import QuicConnectionProtocol
    from aioquic.h3.connection import H3_ALPN, H3Connection
    from aioquic.h3.events import (
        DatagramReceived,
        H3Event,
        HeadersReceived,
        WebTransportStreamDataReceived,
    )
    from aioquic.quic.configuration import QuicConfiguration
    from aioquic.quic.events import ProtocolNegotiated, QuicEvent
except ImportError:
    print("ERROR: 'aioquic' not installed.  Run:  pip install aioquic")
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────
HOST      = "0.0.0.0"
PORT      = 4433
CERT_FILE = Path("cert.pem")
KEY_FILE  = Path("key.pem")
WT_PATH   = "/servo"        # Must match the URL in webxr_controller.html

# BCM GPIO pin numbers
SERVO_PINS = {"s1": 17, "s2": 27, "s3": 22}
PWM_FREQ   = 50     # Hz
DUTY_MIN   = 2.5    # % → 0°
DUTY_MAX   = 12.5   # % → 180°

# A4988 stepper driver — stepper 1 (BCM numbering)
STEP1_PIN       = 23   # physical pin 16 — A4988 STEP
DIR1_PIN        = 24   # physical pin 18 — A4988 DIR
STEPPER1_STEPS  = 200  # steps per full revolution (1.8°/step motor, full-step mode)
STEPPER1_DELAY  = 0.002  # seconds per half-pulse — lower = faster spin

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("wt-bridge")
logging.getLogger("aioquic").setLevel(logging.WARNING)   # silence QUIC noise

# ── Servo state ───────────────────────────────────────────────────────────────
pwm_channels: dict[str, object] = {}
current_angles = {"s1": 90, "s2": 90, "s3": 90}

# Stats
pkt_count  = 0
last_stats = time.monotonic()


def angle_to_duty(angle: int) -> float:
    return DUTY_MIN + (max(0, min(180, angle)) / 180.0) * (DUTY_MAX - DUTY_MIN)


def init_servos():
    if SIMULATE:
        log.info("SIMULATE mode — no GPIO initialised.")
        return
    for key, pin in SERVO_PINS.items():
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, PWM_FREQ)
        pwm.start(angle_to_duty(90))
        pwm_channels[key] = pwm
        log.info(f"  GPIO {pin} → {key} initialised (90°)")


def move_servo(key: str, angle: int):
    current_angles[key] = angle
    if not SIMULATE and key in pwm_channels:
        pwm_channels[key].ChangeDutyCycle(angle_to_duty(angle))


def cleanup_servos():
    if SIMULATE:
        return
    for pwm in pwm_channels.values():
        pwm.stop()
    GPIO.cleanup()


def bar(a: int, w: int = 16) -> str:
    n = int((a / 180) * w)
    return f"[{'#'*n}{'-'*(w-n)}]{a:>4}°"


# ── Stepper 1 (A4988) ─────────────────────────────────────────────────────────

def init_stepper1():
    if SIMULATE:
        return
    GPIO.setup(STEP1_PIN, GPIO.OUT)
    GPIO.setup(DIR1_PIN, GPIO.OUT)
    GPIO.output(STEP1_PIN, GPIO.LOW)
    GPIO.output(DIR1_PIN, GPIO.HIGH)
    log.info(f"  GPIO {STEP1_PIN} → STEP1, GPIO {DIR1_PIN} → DIR1 initialised")


def spin_stepper1(steps: int = STEPPER1_STEPS, delay: float = STEPPER1_DELAY):
    """Blocking step loop — run this off the asyncio event loop (see run_in_executor call site)."""
    if SIMULATE:
        log.info(f"SIMULATE: would spin stepper 1 for {steps} steps.")
        return
    for _ in range(steps):
        GPIO.output(STEP1_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP1_PIN, GPIO.LOW)
        time.sleep(delay)
    log.info(f"Stepper 1 spun {steps} steps.")


def handle_packet(data: bytes):
    """
    Decode an incoming datagram.
      1 byte  -> command byte (0x01 = spin stepper 1 one revolution)
      3 bytes -> servo angle packet [s1, s2, s3]
    """
    global pkt_count

    if len(data) == 1:
        if data[0] == 0x01:
            log.info("Command received: spin stepper 1")
            asyncio.get_event_loop().run_in_executor(None, spin_stepper1)
        return

    if len(data) < 3:
        return

    s1, s2, s3 = data[0], data[1], data[2]

    # Validate range — ignore garbage
    if not (0 <= s1 <= 180 and 0 <= s2 <= 180 and 0 <= s3 <= 180):
        return

    move_servo("s1", s1)
    move_servo("s2", s2)
    move_servo("s3", s3)
    pkt_count += 1

    # Print live status every 30 packets instead of every packet
    # to avoid flooding the terminal while keeping feedback visible
    if pkt_count % 30 == 0:
        ts = datetime.now().strftime("%H:%M:%S")
        print(
            f"\r{ts}  S1{bar(s1)}  S2{bar(s2)}  S3{bar(s3)}  "
            f"[{pkt_count} pkts]   ",
            end="", flush=True,
        )


# ── WebTransport protocol ─────────────────────────────────────────────────────

class ServoHandler(QuicConnectionProtocol):
    """
    Minimal WebTransport server handler.
    Accepts any WebTransport session on WT_PATH and processes incoming datagrams.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._h3: H3Connection | None = None
        self._wt_sessions: set[int] = set()   # active WebTransport session IDs

    def quic_event_received(self, event: QuicEvent):
        if isinstance(event, ProtocolNegotiated):
            self._h3 = H3Connection(self._quic, enable_webtransport=True)

        if self._h3:
            for h3_event in self._h3.handle_event(event):
                self._h3_event(h3_event)

    def _h3_event(self, event: H3Event):
        if isinstance(event, HeadersReceived):
            # Inspect CONNECT request headers for WebTransport handshake
            headers = {k: v for k, v in event.headers}
            method  = headers.get(b":method", b"").decode()
            path    = headers.get(b":path",   b"").decode()

            if method == "CONNECT" and path == WT_PATH:
                # Accept the WebTransport session
                self._h3.send_headers(
                    stream_id=event.stream_id,
                    headers=[(b":status", b"200")],
                )
                self._wt_sessions.add(event.stream_id)
                addr = self._quic._network_paths[0].addr if self._quic._network_paths else "?"
                log.info(f"WebTransport session opened from {addr}  (stream {event.stream_id})")
            else:
                # Reject unknown paths
                self._h3.send_headers(
                    stream_id=event.stream_id,
                    headers=[(b":status", b"404")],
                )

        elif isinstance(event, DatagramReceived):
            # QUIC datagrams arrive here — these are our servo packets
            handle_packet(event.data)

        elif isinstance(event, WebTransportStreamDataReceived):
            # Also accept data on WebTransport streams (fallback from browser)
            handle_packet(event.data)


# ── Main ──────────────────────────────────────────────────────────────────────

async def main():
    if not CERT_FILE.exists() or not KEY_FILE.exists():
        print(f"\nERROR: {CERT_FILE} or {KEY_FILE} not found.")
        print("  Run:  python setup_cert.py\n")
        sys.exit(1)

    config = QuicConfiguration(
        alpn_protocols=H3_ALPN,
        is_client=False,
        max_datagram_frame_size=65536,   # enable QUIC datagrams
    )
    config.load_cert_chain(str(CERT_FILE), str(KEY_FILE))

    init_servos()
    init_stepper1()

    print(f"\n{'=' * 62}")
    print(f"  WebTransport Servo Bridge  (QUIC / UDP-like datagrams)")
    print(f"  Listening on  https://{HOST}:{PORT}{WT_PATH}")
    print(f"  Simulate mode: {SIMULATE}")
    print(f"\n  Servo GPIO map:")
    for k, pin in SERVO_PINS.items():
        print(f"    {k.upper()} → GPIO {pin} (BCM)")
    print(f"\n  Stepper 1 (A4988) GPIO map:")
    print(f"    STEP1 → GPIO {STEP1_PIN} (BCM)")
    print(f"    DIR1  → GPIO {DIR1_PIN} (BCM)")
    print(f"\n  Steps:")
    print(f"    1. Find Pi IP:   hostname -I")
    print(f"    2. Open webxr_controller.html on the Quest Browser")
    print(f"    3. Set Pi IP + cert hash (from setup_cert.py) → Connect")
    print(f"\n  Ctrl+C to stop.")
    print(f"{'=' * 62}\n")

    stop = asyncio.get_event_loop().create_future()

    def _sig(*_):
        if not stop.done():
            stop.set_result(None)

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _sig)
        except NotImplementedError:
            pass

    server = await serve(
        host=HOST,
        port=PORT,
        configuration=config,
        create_protocol=ServoHandler,
    )
    try:
        await stop
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        server.close()

    print()
    log.info(f"Shutting down.  Total packets received: {pkt_count}")
    cleanup_servos()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
