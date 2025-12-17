"""Automate Let's Encrypt certificate renewal and deployment for MeasurePi.

This script requests/renews a certificate using Certbot, copies the updated
artifacts into MeasurePi's default SSL locations, and restarts the Flask
service so the new certificate is loaded. It is intended to be invoked by a
cron job or systemd timer on the Raspberry Pi.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

DEFAULT_CERT_DEST_DIR = Path.home() / "measure_pi"
DEFAULT_CERT_PATH = DEFAULT_CERT_DEST_DIR / "cert.pem"
DEFAULT_KEY_PATH = DEFAULT_CERT_DEST_DIR / "private.pem"
DEFAULT_SERVICE_NAME = "measurepi.service"


class RenewalError(RuntimeError):
    """Raised when certificate renewal or deployment fails."""


class CommandResult:
    """Container for command execution results."""

    def __init__(self, command: list[str], stdout: str, stderr: str, returncode: int):
        self.command = command
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode

    def raise_for_status(self) -> None:
        if self.returncode != 0:
            joined_cmd = " ".join(self.command)
            message_parts = [f"Command failed: {joined_cmd}"]
            if self.stdout:
                message_parts.append(f"STDOUT: {self.stdout.strip()}")
            if self.stderr:
                message_parts.append(f"STDERR: {self.stderr.strip()}")
            raise RenewalError(" | ".join(message_parts))


def run_command(command: list[str]) -> CommandResult:
    result = subprocess.run(command, capture_output=True, text=True)
    return CommandResult(command=command, stdout=result.stdout, stderr=result.stderr, returncode=result.returncode)


def obtain_certificate(domain: str, email: str, certbot_bin: str, staging: bool) -> None:
    """Request or renew a certificate using Certbot standalone mode."""

    command = [
        certbot_bin,
        "certonly",
        "--non-interactive",
        "--agree-tos",
        "--keep-until-expiring",
        "--standalone",
        "--preferred-challenges",
        "http",
        "-d",
        domain,
        "-m",
        email,
    ]

    if staging:
        command.append("--staging")

    print(f"[RENEWAL] Running Certbot for domain {domain}...")
    result = run_command(command)
    result.raise_for_status()
    print("[RENEWAL] Certbot completed successfully.")


def _copy_certificate(source: Path, destination: Path) -> None:
    if destination.exists():
        print(f"[DEPLOY] Removing existing certificate file: {destination}")
        destination.unlink()

    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    destination.chmod(0o600)
    print(f"[DEPLOY] Copied {source} -> {destination}")


def deploy_certificate(domain: str, cert_dest: Path, key_dest: Path) -> None:
    live_dir = Path("/etc/letsencrypt/live") / domain
    source_cert = live_dir / "fullchain.pem"
    source_key = live_dir / "privkey.pem"

    if not source_cert.is_file() or not source_key.is_file():
        raise RenewalError(
            f"Expected certificate files not found for domain {domain}. "
            "Has Certbot successfully issued a certificate?"
        )

    _copy_certificate(source_cert, cert_dest)
    _copy_certificate(source_key, key_dest)


def restart_service(service_name: str) -> None:
    print(f"[SERVICE] Restarting service {service_name} to reload certificates...")
    result = run_command(["sudo", "systemctl", "restart", service_name])
    result.raise_for_status()
    print(f"[SERVICE] Service {service_name} restarted.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Renew and deploy Let's Encrypt certificates for MeasurePi")
    parser.add_argument("--domain", default=os.getenv("LE_DOMAIN"), required=False, help="Domain name to issue the certificate for")
    parser.add_argument("--email", default=os.getenv("LE_EMAIL"), required=False, help="Email address for Let's Encrypt registration")
    parser.add_argument(
        "--service-name",
        default=os.getenv("MEASUREPI_SERVICE", DEFAULT_SERVICE_NAME),
        help="Systemd service name to restart after deployment",
    )
    parser.add_argument(
        "--certbot-bin",
        default=os.getenv("CERTBOT_BIN", "certbot"),
        help="Path to the certbot binary (defaults to 'certbot' on PATH)",
    )
    parser.add_argument(
        "--cert-path",
        default=os.getenv("SSL_CERT_PATH", DEFAULT_CERT_PATH),
        type=Path,
        help="Destination path for the deployed certificate",
    )
    parser.add_argument(
        "--key-path",
        default=os.getenv("SSL_KEY_PATH", DEFAULT_KEY_PATH),
        type=Path,
        help="Destination path for the deployed private key",
    )
    parser.add_argument("--staging", action="store_true", help="Use Let's Encrypt staging environment for testing")
    parser.add_argument(
        "--skip-restart",
        action="store_true",
        help="Skip restarting the service after deploying the certificate",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.domain or not args.email:
        raise RenewalError("Both domain and email must be provided via arguments or environment variables.")

    obtain_certificate(args.domain, args.email, certbot_bin=args.certbot_bin, staging=args.staging)
    deploy_certificate(args.domain, cert_dest=Path(args.cert_path), key_dest=Path(args.key_path))

    if not args.skip_restart:
        restart_service(args.service_name)
    else:
        print("[SERVICE] Restart skipped as requested; restart your Flask service to load the new certificate.")


if __name__ == "__main__":
    try:
        main()
    except RenewalError as exc:
        print(f"[ERROR] {exc}")
        sys.exit(1)
