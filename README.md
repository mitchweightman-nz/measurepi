# MeasurePi

MeasurePi is a Flask-based dashboard and MQTT client for running on a Raspberry Pi.
This repository also now includes tooling to automate HTTPS certificate renewal
for deployments that expose the dashboard over the internet.

## Automatic SSL renewal with Let's Encrypt

Use the provided `ssl_renewal.py` helper to request/renew Let's Encrypt
certificates, deploy them to the paths expected by `MeasurePi.py`, and restart the
Flask service so the new certificate is loaded.

### Prerequisites

1. Ports 80/443 reachable from the internet for HTTP-01 validation.
2. Certbot installed (`sudo apt-get install certbot`).
3. MeasurePi running under a systemd service (default service name: `measurepi.service`).

### One-time issuance / renewal

```bash
sudo python3 ssl_renewal.py \
  --domain example.yourdomain.com \
  --email admin@yourdomain.com \
  --service-name measurepi.service
```

The script will:

- Run Certbot in standalone mode to issue or renew the certificate for the domain.
- Remove any previously deployed certificate/key at `~/measure_pi/cert.pem` and
  `~/measure_pi/private.pem` (paths can be overridden with `--cert-path` and
  `--key-path`).
- Copy the latest certificate from `/etc/letsencrypt/live/<domain>/` into the
  MeasurePi SSL location.
- Restart the systemd service (override with `--service-name` or set
  `MEASUREPI_SERVICE`).

Use `--staging` to test renewal without hitting Let's Encrypt production limits.

### Automating renewals

Add a cron entry or systemd timer to run the script twice per day. Example cron
entry (edit via `sudo crontab -e`):

```
0 3,15 * * * /usr/bin/python3 /home/pi/measurepi/ssl_renewal.py --domain example.yourdomain.com --email admin@yourdomain.com >> /var/log/measurepi_ssl.log 2>&1
```

This safely reuses `--keep-until-expiring` so Certbot only performs validation
when the certificate is close to expiring.
