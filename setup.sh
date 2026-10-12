#!/bin/sh
set -eu
cd "$(dirname "$0")"
export UV_CACHE_DIR="$PWD/.uv-cache"
if [ ! -x .venv/bin/python ]; then
    uv venv --python 3.12 .venv
fi
uv pip install --python .venv/bin/python -r requirements.txt
# cadgen installs the no-VTK OCP wheel, while CadQuery requires IVtkOCC.
# Both expose the same OCP namespace. Install the matching VTK-enabled wheel last.
uv pip install --python .venv/bin/python --reinstall --no-deps cadquery-ocp==7.9.3.1.1
.venv/bin/python -c 'import cadgen; print("Text to CAD runtime ready")'
