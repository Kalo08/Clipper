"""
UDP Servo Data Listener - Test Script
Receives raw UDP packets from Android APK and decodes servo control data
intended for a Raspberry Pi 3.

Expected packet format (sent from Android APK):
  - Simple CSV:   "S1:90,S2:45,S3:120"
  - JSON:         {"s1": 90, "s2": 45, "s3": 120}
  - Binary:       [servo_id (1 byte), angle (1 byte), ...]
  - Raw bytes:    b'\x01\x5a\x02\x2d'  (servo_id, angle pairs)

Run this on your PC or Raspberry Pi 3 to verify the APK is sending
correctly before wiring up actual servo GPIO control.
"""

import socket
import struct
import json
import time
import argparse
from datetime import datetime

# ──────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────
DEFAULT_HOST = "0.0.0.0"   # Listen on all interfaces
DEFAULT_PORT = 5005         # Match this in your Android APK
BUFFER_SIZE  = 1024         # Max bytes per UDP packet


# ──────────────────────────────────────────────
# Packet parsers
# ──────────────────────────────────────────────

def try_parse_json(data: bytes) -> dict | None:
    """Try to decode the payload as JSON."""
    try:
        obj = json.loads(data.decode("utf-8"))
        if isinstance(obj, dict):
            return obj
    except (json.JSONDecodeError, UnicodeDecodeError):
        pass
    return None


def try_parse_csv(data: bytes) -> dict | None:
    """
    Try to decode the payload as CSV servo pairs.
    Accepted formats:
        "S1:90,S2:45"
        "1:90,2:45"
        "90,45,120"   (positional, no servo-id prefix)
    """
    try:
        text = data.decode("utf-8").strip()
        servos = {}

        if ":" in text:
            for part in text.split(","):
                part = part.strip()
                if ":" in part:
                    label, val = part.split(":", 1)
                    label = label.strip().upper().lstrip("S")
                    servos[f"S{label}"] = int(val.strip())
        else:
            for i, val in enumerate(text.split(","), start=1):
                servos[f"S{i}"] = int(val.strip())

        return servos if servos else None
    except (ValueError, UnicodeDecodeError):
        pass
    return None


def try_parse_binary(data: bytes) -> dict | None:
    """
    Try to decode the payload as raw binary servo pairs.
    Format: alternating bytes  [servo_id, angle, servo_id, angle, ...]
    Angle clamped to 0-180.
    """
    if len(data) < 2 or len(data) % 2 != 0:
        return None
    servos = {}
    for i in range(0, len(data), 2):
        servo_id = data[i]
        angle    = data[i + 1]
        if not (0 <= angle <= 180):
            return None   # Doesn't look like servo angles
        servos[f"S{servo_id}"] = angle
    return servos if servos else None


def parse_packet(data: bytes) -> tuple[str, dict | None]:
    """
    Attempt all parsers in order of likelihood.
    Returns (format_name, servo_dict) or (format_name, None) if unknown.
    """
    result = try_parse_json(data)
    if result:
        return "JSON", result

    result = try_parse_csv(data)
    if result:
        return "CSV", result

    result = try_parse_binary(data)
    if result:
        return "BINARY", result

    return "UNKNOWN", None


# ──────────────────────────────────────────────
# Display helpers
# ──────────────────────────────────────────────

def servo_bar(angle: int, width: int = 20) -> str:
    """Render a simple ASCII bar for a servo angle (0-180)."""
    filled = int((angle / 180) * width)
    bar = "#" * filled + "-" * (width - filled)
    return f"[{bar}] {angle:>3}°"


def print_servos(fmt: str, servos: dict, addr: tuple, raw: bytes):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"\n{'─' * 52}")
    print(f"  {ts}  from {addr[0]}:{addr[1]}   [{fmt}]")
    print(f"  raw ({len(raw)} bytes): {raw.hex(' ').upper()}")
    print()
    for name, angle in sorted(servos.items()):
        print(f"  {name:>4}  {servo_bar(angle)}")
    print(f"{'─' * 52}")


def print_unknown(addr: tuple, raw: bytes):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"\n[{ts}] UNRECOGNISED packet from {addr[0]}:{addr[1]}")
    print(f"  raw ({len(raw)} bytes): {raw.hex(' ').upper()}")
    try:
        print(f"  utf-8: {raw.decode('utf-8')!r}")
    except UnicodeDecodeError:
        pass


# ──────────────────────────────────────────────
# Main listener
# ──────────────────────────────────────────────

def listen(host: str, port: int, packet_limit: int | None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(1.0)   # Allow Ctrl-C to work cleanly

    print(f"\n{'=' * 52}")
    print(f"  UDP Servo Listener")
    print(f"  Listening on {host}:{port}")
    print(f"  Waiting for packets from Android APK ...")
    if packet_limit:
        print(f"  Will stop after {packet_limit} packets.")
    print(f"  Press Ctrl+C to quit.")
    print(f"{'=' * 52}\n")

    packet_count = 0
    stats = {"json": 0, "csv": 0, "binary": 0, "unknown": 0}

    try:
        while True:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                continue

            packet_count += 1
            fmt, servos = parse_packet(data)

            if servos:
                stats[fmt.lower()] = stats.get(fmt.lower(), 0) + 1
                print_servos(fmt, servos, addr, data)
            else:
                stats["unknown"] += 1
                print_unknown(addr, data)

            if packet_limit and packet_count >= packet_limit:
                print(f"\nReached packet limit ({packet_limit}). Stopping.")
                break

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f"\n{'=' * 52}")
        print(f"  Session ended. {packet_count} packet(s) received.")
        print(f"  JSON={stats['json']}  CSV={stats['csv']}  "
              f"BINARY={stats['binary']}  UNKNOWN={stats['unknown']}")
        print(f"{'=' * 52}\n")


# ──────────────────────────────────────────────
# CLI entry point
# ──────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test listener for UDP servo data from an Android APK."
    )
    parser.add_argument(
        "--host", default=DEFAULT_HOST,
        help=f"Interface to bind (default: {DEFAULT_HOST})"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"UDP port to listen on (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--limit", type=int, default=None,
        help="Stop after N packets (default: run forever)"
    )
    args = parser.parse_args()
    listen(args.host, args.port, args.limit)
