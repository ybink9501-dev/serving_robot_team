#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv --system-site-packages "$ROOT/_local_only/venv"
"$ROOT/_local_only/venv/bin/python" -m pip install --no-compile -r "$ROOT/requirements-perception.txt"
"$ROOT/_local_only/venv/bin/python" -m pip check
