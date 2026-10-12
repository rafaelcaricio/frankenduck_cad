#!/bin/sh
set -eu
cd "$(dirname "$0")"
export CADGEN_CACHE_DIR="$PWD/.cadgen"
export CADGEN_JOBS=4
exec .venv/bin/python -m cadgen.viewer --host 0.0.0.0 --json "$@"
