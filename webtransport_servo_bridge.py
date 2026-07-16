"""
WebTransport (QUIC) → Servo Bridge
Receives compact binary datagrams from the Meta Quest WebXR page
and drives servos directly via Pi GPIO PWM (no PCA9685 driver board).

Packet format (matches webxr_controller.html):
    1 byte  → command byte (0x01 = spin stepper 1 one revolution)
    3 bytes → servo angles [S1, S2, S3], each 0-180
        byte 0 → S1 angle  (0–180)  base rotation (pan / yaw)        → GPIO27 (physical pin 13)
        byte 1 → S2 angle  (0–180)  joint on base (tilt / pitch)     → GPIO17 (physical pin 11)
        byte 2 → S3 angle  (0–180)  roll                             → GPIO22 (physical pin 15)
        byte 3 → S4 angle  (0–180)  driven by analog trigger, 90=up/0=down → GPIO4 (physical pin 7)
        byte 4 → trigger strength (0-255), informational only

Wiring (servo → Pi 4):
    Signal → GPIO27 / GPIO17 / GPIO22 / GPIO4 (physical pins 13 / 11 / 15 / 7)
    S2B (second shoulder servo, mirrored follower of S2) → GPIO18 (physical pin 12)
    V+     → 5–6 V external supply for servos (NOT the Pi's 5V pin)
    GND    → common ground with the Pi

Dependencies (install on the Pi):
    pip install aioquic gpiozero lgpio RPi.GPIO

gpiozero + lgpio drive the servo PWM directly (no daemon needed) via the
modern GPIO chardev interface — works on current Raspberry Pi OS releases
where the unmaintained `pigpio` package is no longer available.

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
import threading
import time
from datetime import datetime
from pathlib import Path

# ── gpiozero / lgpio (direct GPIO PWM for servos) ─────────────────────────────
SIMULATE = False
_AngularServo = None
try:
    from gpiozero import AngularServo, Device
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()
    _AngularServo = AngularServo
except Exception as e:
    SIMULATE = True
    print(f"[bridge] gpiozero/lgpio unavailable ({e}) — SIMULATE mode (no servo output).")

# ── Stepper GPIO import ───────────────────────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    _GPIO_OK = True
except (ImportError, RuntimeError):
    _GPIO_OK = False

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

# GPIO pin (BCM numbering) for each servo's PWM signal wire
SERVO_GPIO = {"s1": 27, "s2": 17, "s3": 22, "s4": 4, "s2b": 18}

# s2b is a second shoulder servo mounted on the OPPOSITE side of the joint
# from s2, driving the first linkage in unison with it. Because it faces the
# other way it always follows s2 at the mirrored angle (180 - s2) — driving
# both to the same angle would make them fight each other and stall.
SERVO_FOLLOWERS = {"s2": ("s2b", lambda a: 180 - a)}

# The follower tracks the master's position from this long ago, tracing the
# same motion path uniformly time-shifted. Only useful for a TRUE timing
# mismatch — a lead/lag that flips with direction (in sync one way, delayed
# the other) is an angle offset and needs TRIM below, not delay.
SERVO_FOLLOWER_DELAY_S = 0.0

# Constant trim added to s2b's mirrored angle, correcting a horn seated a few
# degrees off true mirror. Tune this ONLY once behavior is consistent
# (stable power, no random twitching) — calibrating against noise is
# meaningless. Symptom of needing trim: lead/lag that flips with sweep
# direction, the same way every sweep.
SERVO_FOLLOWER_TRIM_DEG = 0.0

# Startup pose per servo (defaults to 0). s2b mirrors s2's 0° as 180°.
SERVO_INIT = {"s2b": 180}

# Pulse width range for 0-180 degrees (microseconds) — standard hobby servo range
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# Hard safety limits — servos are never driven outside these ranges, regardless
# of what a client requests. Physical range is still 0-180 (see AngularServo
# below); this just clamps the usable travel.
#   s1 (base)          — full range, no clamp
#   s2/s3/s4 (joints)  — clamped to ±85° around centre (90°), a few degrees
#                        short of the mechanical stops
SERVO_LIMITS = {
    "s1":  (0, 180),
    "s2":  (5, 175),
    "s2b": (5, 175),
    "s3":  (5, 175),
    "s4":  (5, 175),
}

# NOTE: a global "reverse s2's direction" mirror (angle -> 180 - angle) was
# tried here and reverted — logical angle 0 is confirmed to be the arm's
# straight-up rest pose (via zero_servos.py), and mirroring the whole 0-180
# range necessarily moves that reference to the opposite physical extreme
# (logical 0 -> physical 180), which is NOT "up". A true full-range direction
# reversal is mathematically incompatible with keeping 0 fixed at "up" — any
# monotonically-decreasing map from [0,180] onto [0,180] forces f(0)=180.
# If S2 still moves the "wrong" way, the real fix has to be mechanical
# (re-seat the horn a spline or two) or a narrower, non-mirroring correction —
# not a global software flip.

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

# Max servo speed in deg/sec. All motion is slew-rate limited to this in a
# single shared loop, which is what keeps the s2/s2b pair in true lockstep:
# a physically faster servo can only ever get one tick ahead instead of
# racing to the target on its own.
# Kept LOW on purpose: a servo's current draw scales with how far it lags
# its commanded position, so a gentle slew rate caps the peak current —
# critical on a weak supply (AA batteries). Raise this once the arm is on a
# proper 5-6V high-current supply.
SERVO_SLEW_DEG_PER_S = 20

# ── Servo state ───────────────────────────────────────────────────────────────
current_angles = {"s1": 0, "s2": 0, "s3": 0, "s4": 0, "s2b": 180}
target_angles  = dict(current_angles)
_slew_stop = None   # threading.Event, created when the slew loop starts

# Stats
pkt_count  = 0
last_stats = time.monotonic()


servos = {}   # key -> AngularServo instance, populated by init_servos()


def init_servos():
    if SIMULATE:
        log.info("SIMULATE mode — gpiozero not initialised.")
        return
    for key, gpio in SERVO_GPIO.items():
        init_angle = SERVO_INIT.get(key, 0)
        servos[key] = _AngularServo(
            gpio,
            initial_angle=init_angle,
            min_angle=0,
            max_angle=180,
            min_pulse_width=SERVO_MIN_US / 1_000_000,
            max_pulse_width=SERVO_MAX_US / 1_000_000,
        )
        log.info(f"  GPIO{gpio} → {key.upper()} initialised ({init_angle}°)")


def move_servo(key: str, angle: int):
    # Master/slave: commands only set TARGETS. The slew loop below owns all
    # hardware writes, stepping every servo toward its target in shared ticks
    # so the s2/s2b pair moves in lockstep instead of each servo racing to
    # the target at its own physical speed. Follower (s2b) targets are set
    # inside the loop from the master's position history, never here.
    lo, hi = SERVO_LIMITS[key]
    angle = max(lo, min(hi, angle))
    target_angles[key] = angle


def _slew_loop():
    from collections import deque
    # 50 Hz — matches the servo pulse rate. Updating pulse widths faster
    # than the servos consume them (e.g. 100 Hz) risks mid-pulse glitches
    # with software-timed PWM, which shows up as random twitching.
    TICK = 0.02
    step = SERVO_SLEW_DEG_PER_S * TICK       # max degrees per tick
    # -1: the loop ordering (target read → step → history append) already
    # contributes one tick of inherent lag
    delay_ticks = max(0, round(SERVO_FOLLOWER_DELAY_S / TICK) - 1)
    # Per-master ring buffer of past positions; the oldest entry is the
    # master's angle SERVO_FOLLOWER_DELAY_S ago once the buffer fills.
    history = {m: deque([current_angles[m]] * (delay_ticks + 1),
                        maxlen=delay_ticks + 1)
               for m in SERVO_FOLLOWERS}

    while not _slew_stop.is_set():
        # Followers chase the master's delayed position, not its live target
        for master, (fkey, transform) in SERVO_FOLLOWERS.items():
            flo, fhi = SERVO_LIMITS[fkey]
            target_angles[fkey] = max(flo, min(fhi,
                transform(history[master][0]) + SERVO_FOLLOWER_TRIM_DEG))

        for key in SERVO_GPIO:
            cur, tgt = current_angles[key], target_angles[key]
            if cur != tgt:
                if abs(tgt - cur) <= step:
                    new = tgt
                else:
                    new = cur + step if tgt > cur else cur - step
                current_angles[key] = new
                if not SIMULATE:
                    servos[key].angle = new

        for master in history:
            history[master].append(current_angles[master])
        time.sleep(TICK)


def start_slew_loop():
    global _slew_stop
    _slew_stop = threading.Event()
    threading.Thread(target=_slew_loop, daemon=True, name="servo-slew").start()
    log.info(f"Servo slew loop started ({SERVO_SLEW_DEG_PER_S}°/s, 50 Hz).")


def cleanup_servos():
    if not SIMULATE:
        # Stop the slew loop so it can't race these direct writes, then
        # centre all servos (within their limits) so they don't hold
        # tension, and release the pins
        if _slew_stop is not None:
            _slew_stop.set()
            time.sleep(0.05)
        for key, servo in servos.items():
            try:
                lo, hi = SERVO_LIMITS[key]
                servo.angle = (lo + hi) / 2
                time.sleep(0.3)
                servo.close()
            except Exception:
                pass
    if _GPIO_OK:
        GPIO.cleanup()


def bar(a: int, w: int = 16) -> str:
    n = int((a / 180) * w)
    return f"[{'#'*n}{'-'*(w-n)}]{a:>4}°"


# ── Per-channel servo "spin" test ────────────────────────────────────────────
SERVO_SPIN_CMDS = {0x02: "s1", 0x03: "s2", 0x04: "s3"}

# Guards against overlapping motions: pressing a spin button twice (or angle
# packets streaming in mid-sweep) would otherwise interleave two writers on
# the same servo, which shows up as erratic continuous sweeping.
_busy_lock = threading.Lock()
_busy: set[str] = set()


def _claim(key: str) -> bool:
    with _busy_lock:
        if key in _busy:
            return False
        _busy.add(key)
        return True


def _release(key: str):
    with _busy_lock:
        _busy.discard(key)


def sweep_servo(key: str, delay: float = 0.01):
    """Blocking sweep lo -> hi -> lo across this servo's SERVO_LIMITS — run off the event loop (see run_in_executor call site)."""
    if SIMULATE:
        log.info(f"SIMULATE: would sweep servo {key}.")
        return
    if not _claim(key):
        log.info(f"Servo {key.upper()} sweep already running — ignored.")
        return
    lo, hi = SERVO_LIMITS[key]
    try:
        for angle in list(range(lo, hi + 1, 2)) + list(range(hi, lo - 1, -2)):
            move_servo(key, angle)
            time.sleep(delay)
        log.info(f"Servo {key.upper()} sweep complete.")
    finally:
        _release(key)


# ── Stepper 1 (A4988) ─────────────────────────────────────────────────────────

def init_stepper1():
    if not _GPIO_OK:
        return
    GPIO.setup(STEP1_PIN, GPIO.OUT)
    GPIO.setup(DIR1_PIN, GPIO.OUT)
    GPIO.output(STEP1_PIN, GPIO.LOW)
    GPIO.output(DIR1_PIN, GPIO.HIGH)
    log.info(f"  GPIO {STEP1_PIN} → STEP1, GPIO {DIR1_PIN} → DIR1 initialised")


def spin_stepper1(steps: int = STEPPER1_STEPS, delay: float = STEPPER1_DELAY):
    """Blocking step loop — run this off the asyncio event loop (see run_in_executor call site)."""
    if not _GPIO_OK:
        log.info(f"SIMULATE: would spin stepper 1 for {steps} steps.")
        return
    if not _claim("stepper1"):
        log.info("Stepper 1 already spinning — ignored.")
        return
    try:
        for _ in range(steps):
            GPIO.output(STEP1_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(STEP1_PIN, GPIO.LOW)
            time.sleep(delay)
        log.info(f"Stepper 1 spun {steps} steps.")
    finally:
        _release("stepper1")


def handle_packet(data: bytes):
    """
    Decode an incoming datagram.
      1 byte  -> command byte (0x01 = spin stepper 1 one revolution)
      3 bytes -> servo angle packet [s1, s2, s3]
    """
    global pkt_count

    if len(data) == 1:
        cmd = data[0]
        if cmd == 0x01:
            log.info("Command received: spin stepper 1")
            asyncio.get_event_loop().run_in_executor(None, spin_stepper1)
        elif cmd in SERVO_SPIN_CMDS:
            key = SERVO_SPIN_CMDS[cmd]
            log.info(f"Command received: spin servo {key.upper()}")
            asyncio.get_event_loop().run_in_executor(None, sweep_servo, key)
        return

    if len(data) < 4:
        return

    s1, s2, s3, s4 = data[0], data[1], data[2], data[3]

    # Validate range — ignore garbage
    if not (0 <= s1 <= 180 and 0 <= s2 <= 180 and 0 <= s3 <= 180 and 0 <= s4 <= 180):
        return

    # Skip servos mid-sweep so streamed angles don't fight the sweep loop
    for key, val in (("s1", s1), ("s2", s2), ("s3", s3), ("s4", s4)):
        if key not in _busy:
            move_servo(key, val)
    pkt_count += 1

    # Print live status every 30 packets instead of every packet
    # to avoid flooding the terminal while keeping feedback visible
    if pkt_count % 30 == 0:
        ts = datetime.now().strftime("%H:%M:%S")
        print(
            f"\r{ts}  S1{bar(s1)}  S2{bar(s2)}  S3{bar(s3)}  S4{bar(s4)}  "
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
    start_slew_loop()

    print(f"\n{'=' * 62}")
    print(f"  WebTransport Servo Bridge  (QUIC / UDP-like datagrams)")
    print(f"  Listening on  https://{HOST}:{PORT}{WT_PATH}")
    print(f"  Simulate mode: {SIMULATE}")
    print(f"\n  Servo GPIO map (BCM numbering):")
    for k, gpio in SERVO_GPIO.items():
        print(f"    {k.upper()} → GPIO{gpio}")
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
