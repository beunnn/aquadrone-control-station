#!/usr/bin/env bash
# Regenerate the MAVLink 2 C headers (ardupilotmega dialect) used by the firmware.
# Uses the pymavlink in ./.venv (create it with: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt).
set -euo pipefail
cd "$(dirname "$0")/.."
DIALECTS=$(.venv/bin/python -c "import pymavlink,os;print(os.path.join(os.path.dirname(pymavlink.__file__),'dialects','v20'))")
.venv/bin/python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 \
    --output=firmware/components/mavlink_headers/generated "$DIALECTS/ardupilotmega.xml"
