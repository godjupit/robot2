#!/bin/sh
set -eu
cd "$(dirname "$0")"
PYTHON=${ROBOT_IMAGE_PYTHON:-/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python}
exec "$PYTHON" run_image.py examples/triangle.png "$@"
