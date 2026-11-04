"""Built-in CAD viewer handoff for the canonical saved STEP.

Run with ~/.local/bin/cad run review/canonical_assembly_view.py after ./cad build.
This static review document reads fresh STEP occurrence colors; the canonical
STEP and its sidecar remain the assembly and animation deliverables.
"""
from pathlib import Path
from cad import model, read_step
ROOT=Path(__file__).resolve().parents[1]

@model
def canonical_assembly():
    return read_step(ROOT/'STEP/frankenduck.step')

if __name__=='__main__':canonical_assembly()
