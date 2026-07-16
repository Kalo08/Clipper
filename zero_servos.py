"""
Zero servos — drives every servo to the arm's zeroed pose and HOLDS it there
(PWM stays active) until you press Ctrl+C, then releases the pins.

Note: "zeroed pose" is not 0° for every servo — S2B is mounted on the opposite
side of the shoulder joint from S2, so its mirrored zero is 180°.

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
# s1 -> pin 13, s2 -> pin 11, s3 -> pin 15, s4 -> pin 7, s2b -> pin 12
SERVO_GPIO = {"s1": 27, "s2": 17, "s3": 22, "s4": 4, "s2b": 18}

# Angle each servo holds in the zeroed pose (s2b mirrors s2: 180 - 0)
ZERO_ANGLES = {"s1": 0, "s2": 0, "s3": 0, "s4": 0, "s2b": 180}

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

print("=" * 60)
print("  Zeroing all servos")
print("  Make sure webtransport_servo_bridge.py is NOT running.")
print("=" * 60)

servos = {}
for key, gpio in SERVO_GPIO.items():
    angle = ZERO_ANGLES[key]
    try:
        servo = AngularServo(
            gpio,
            initial_angle=angle,
            min_angle=0,
            max_angle=180,
            min_pulse_width=SERVO_MIN_US / 1_000_000,
            max_pulse_width=SERVO_MAX_US / 1_000_000,
        )
        servos[key] = servo
        print(f"  GPIO{gpio} → {key.upper()} holding at {angle}°")
    except Exception as e:
        print(f"  GPIO{gpio} ({key.upper()}) unavailable: {e}")

print("\nHolding zero pose — press Ctrl+C to release the servos and exit.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

for key, servo in servos.items():
    servo.close()

print("\nDone. Servos released.")
