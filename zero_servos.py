"""
Zero servos — drives S1/S2/S3 to 0 degrees and holds, then releases the pins.

Run this with the bridge STOPPED (Ctrl+C webtransport_servo_bridge.py first),
otherwise the GPIO pins are already claimed and this script will fail to
open them.

Run:
    source venv/bin/activate
    python zero_servos.py
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

# Must match SERVO_GPIO in webtransport_servo_bridge.py
# s1 -> physical pin 13, s2 -> physical pin 11, s3 -> physical pin 15
SERVO_GPIO = {"s1": 27, "s2": 17, "s3": 22}

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

HOLD_SECONDS = 1.0   # how long to hold at 0 degrees before releasing the pins

print("=" * 60)
print("  Zeroing all servos")
print("  Make sure webtransport_servo_bridge.py is NOT running.")
print("=" * 60)

servos = {}
for key, gpio in SERVO_GPIO.items():
    try:
        servo = AngularServo(
            gpio,
            initial_angle=0,
            min_angle=0,
            max_angle=180,
            min_pulse_width=SERVO_MIN_US / 1_000_000,
            max_pulse_width=SERVO_MAX_US / 1_000_000,
        )
        servos[key] = servo
        print(f"  GPIO{gpio} → {key.upper()} set to 0°")
    except Exception as e:
        print(f"  GPIO{gpio} ({key.upper()}) unavailable: {e}")

time.sleep(HOLD_SECONDS)

for key, servo in servos.items():
    servo.close()

print("\nDone. All reachable servos zeroed and pins released.")
