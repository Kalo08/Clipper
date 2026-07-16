"""
Servo pin finder — figures out which physical servo is wired to which GPIO.

Pulses one GPIO at a time and asks you to watch which servo wiggles.
Run it with the bridge STOPPED (Ctrl+C webtransport_servo_bridge.py first),
otherwise the pins are already claimed and nothing will move.

Run:
    source venv/bin/activate
    python servo_finder.py
"""

import time

try:
    from gpiozero import AngularServo, Device
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()
except Exception as e:
    print(f"ERROR: gpiozero/lgpio not available ({e})")
    print("  Run:  pip install gpiozero lgpio")
    raise SystemExit(1)

# (BCM GPIO, physical pin) — bridge pins first, then neighbours in case a
# wire landed one pin off from where it was supposed to go.
CANDIDATES = [
    (27, 13),   # bridge S1 — base rotation
    (17, 11),   # bridge S2 — joint on base
    (22, 15),   # bridge S3
    (18, 12),   # neighbour of pin 11 (same row)
    (4,  7),    # above pin 11
    (10, 19),   # below pin 15's area
    (9,  21),
    (11, 23),
]

print("=" * 60)
print("  Servo pin finder")
print("  Watch the servos — each GPIO gets wiggled one at a time.")
print("  Make sure webtransport_servo_bridge.py is NOT running.")
print("=" * 60)

for gpio, pin in CANDIDATES:
    input(f"\nPress Enter to wiggle GPIO{gpio} (physical pin {pin}) ... ")
    try:
        s = AngularServo(
            gpio,
            initial_angle=90,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025,
        )
    except Exception as e:
        print(f"  GPIO{gpio} unavailable: {e}")
        continue
    for _ in range(3):
        s.angle = 60
        time.sleep(0.4)
        s.angle = 120
        time.sleep(0.4)
    s.angle = 90
    time.sleep(0.3)
    s.close()
    print(f"  Done. If a servo just wiggled, its signal wire is on GPIO{gpio} (pin {pin}).")

print("\nAll candidates tested.")
print("Match what you saw against the bridge map: S1=GPIO27  S2=GPIO17  S3=GPIO22")
