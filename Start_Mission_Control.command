#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

if [ -x ".venv311/bin/python" ]; then
  exec ".venv311/bin/python" "scripts/start_app.py"
fi

if command -v python3 >/dev/null 2>&1; then
  exec python3 "scripts/start_app.py"
fi

echo "Python3 not found. Install Python 3.11 and run make install."
exit 1
