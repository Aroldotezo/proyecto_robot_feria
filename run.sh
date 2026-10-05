#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

if [ ! -x ".venv/bin/python" ]; then
    echo "[setup] Creating virtual environment..."
    python3 -m venv .venv
fi

echo "[setup] Checking dependencies (the first run takes a few minutes)..."
.venv/bin/python -m pip install --quiet --disable-pip-version-check -r requirements.txt

exec .venv/bin/python -m robotic_arm "$@"
