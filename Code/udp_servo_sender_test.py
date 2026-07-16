"""
UDP Servo Sender - Local test only
Simulates what the Android APK would send so you can verify the listener
is working before flashing anything to the Raspberry Pi 3.

Run the listener first:
    python udp_servo_listener.py

Then in a second terminal run this sender:
    python udp_servo_sender_test.py --mode json
    python udp_servo_sender_test.py --mode csv
    python udp_servo_sender_test.py --mode binary
"""

import socket
import json
import struct
import time
import argparse
import math

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005


def send_json(sock, addr, step: int):
    """Send servo data as a JSON object."""
    # Smoothly sweep S1 and S2 back and forth
    angle1 = int(90 + 89 * math.sin(step * 0.1))
    angle2 = int(90 + 89 * math.cos(step * 0.1))
    angle3 = (step * 5) % 181
    payload = json.dumps({"s1": angle1, "s2": angle2, "s3": angle3}).encode()
    sock.sendto(payload, addr)
    print(f"[JSON]   sent: {payload.decode()}")


def send_csv(sock, addr, step: int):
    """Send servo data as a CSV string."""
    angle1 = int(90 + 89 * math.sin(step * 0.1))
    angle2 = int(90 + 89 * math.cos(step * 0.1))
    payload = f"S1:{angle1},S2:{angle2}".encode()
    sock.sendto(payload, addr)
    print(f"[CSV]    sent: {payload.decode()}")


def send_binary(sock, addr, step: int):
    """Send servo data as raw binary (servo_id, angle) pairs."""
    angle1 = int(90 + 89 * math.sin(step * 0.1))
    angle2 = int(90 + 89 * math.cos(step * 0.1))
    # struct: B = unsigned char (1 byte)
    payload = struct.pack("BBBB", 1, angle1, 2, angle2)
    sock.sendto(payload, addr)
    print(f"[BINARY] sent: {payload.hex(' ').upper()}")


MODES = {"json": send_json, "csv": send_csv, "binary": send_binary}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Simulate Android APK UDP servo packets for local testing."
    )
    parser.add_argument(
        "--host", default=DEFAULT_HOST,
        help=f"Target host (default: {DEFAULT_HOST})"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"UDP port (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--mode", choices=list(MODES), default="json",
        help="Packet format to send (default: json)"
    )
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="Seconds between packets (default: 0.5)"
    )
    parser.add_argument(
        "--count", type=int, default=20,
        help="Number of packets to send (default: 20)"
    )
    args = parser.parse_args()

    sender = MODES[args.mode]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"\nSending {args.count} [{args.mode.upper()}] packets to "
          f"{args.host}:{args.port} every {args.interval}s\n")
    try:
        for i in range(args.count):
            sender(sock, (args.host, args.port), i)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print("\nDone.")
