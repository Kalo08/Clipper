"""
TLS Certificate Generator for WebTransport
Run this ONCE on your Raspberry Pi to create the self-signed cert.

Install dependency first:
    pip install cryptography

Run:
    python setup_cert.py

Outputs:
    cert.pem  — certificate (give to webtransport_servo_bridge.py)
    key.pem   — private key  (give to webtransport_servo_bridge.py)

Also prints the SHA-256 fingerprint you must paste into the Quest browser page.
"""

import hashlib
import ipaddress
import socket
import datetime
from pathlib import Path

try:
    from cryptography import x509
    from cryptography.x509.oid import NameOID
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.hazmat.backends import default_backend
except ImportError:
    print("ERROR: 'cryptography' package not installed.")
    print("  Run:  pip install cryptography")
    raise SystemExit(1)

CERT_FILE = Path("cert.pem")
KEY_FILE  = Path("key.pem")

# WebTransport serverCertificateHashes certs must expire within 14 days
VALID_DAYS = 13


def local_ip() -> str:
    """Best-guess for the machine's LAN IP."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def generate_cert():
    ip = local_ip()
    now = datetime.datetime.now(datetime.timezone.utc)
    exp = now + datetime.timedelta(days=VALID_DAYS)

    # P-256 key (required for WebTransport serverCertificateHashes)
    key = ec.generate_private_key(ec.SECP256R1(), default_backend())

    name = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, "servo-bridge"),
    ])

    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(exp)
        .add_extension(
            x509.SubjectAlternativeName([
                x509.IPAddress(ipaddress.IPv4Address(ip)),
                x509.IPAddress(ipaddress.IPv4Address("127.0.0.1")),
                x509.DNSName("localhost"),
            ]),
            critical=False,
        )
        .sign(key, hashes.SHA256(), default_backend())
    )

    # Write files
    CERT_FILE.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    KEY_FILE.write_bytes(
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.TraditionalOpenSSL,
            serialization.NoEncryption(),
        )
    )

    # Compute SHA-256 fingerprint of the DER-encoded cert
    der   = cert.public_bytes(serialization.Encoding.DER)
    sha   = hashlib.sha256(der).digest()
    hex_fp = ":".join(f"{b:02X}" for b in sha)
    b64_fp = __import__("base64").b64encode(sha).decode()

    print(f"\n{'=' * 60}")
    print(f"  Certificate generated successfully!")
    print(f"  Valid for {VALID_DAYS} days (WebTransport limit).")
    print(f"  Re-run this script before it expires.\n")
    print(f"  Files written:")
    print(f"    {CERT_FILE.resolve()}")
    print(f"    {KEY_FILE.resolve()}\n")
    print(f"  Server IP detected: {ip}")
    print(f"\n  ── Paste this into webxr_controller.html ───────────────")
    print(f"  CERT_HASH_B64 = \"{b64_fp}\"")
    print(f"  SERVER_IP     = \"{ip}\"")
    print(f"\n  SHA-256 fingerprint (hex):")
    print(f"  {hex_fp}")
    print(f"{'=' * 60}\n")

    return b64_fp, ip


if __name__ == "__main__":
    generate_cert()
