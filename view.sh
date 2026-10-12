#!/bin/sh
set -eu
cd "$(dirname "$0")"
export CADGEN_CACHE_DIR="$PWD/.cadgen"
export CADGEN_JOBS=4
CADGEN_VIEWER_DIST="$(.venv/bin/python tools/prepare_viewer.py)"
export CADGEN_VIEWER_DIST
exec .venv/bin/python -m cadgen.viewer --host 0.0.0.0 --json "$@"
