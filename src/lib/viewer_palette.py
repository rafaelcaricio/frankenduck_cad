"""MuJoCo visual colors; only entirely added printed parts are black; appearance only."""
import json
from pathlib import Path
from cadgen import declare_input
PALETTE=Path(__file__).resolve().parents[2]/'source/viewer_palette.json'

def apply_palette(root):
    declare_input(PALETTE)
    palette=json.loads(PALETTE.read_text())
    def paint(node,color):
        node.color=color
        for child in node.children:paint(child,color)
    for part in root.children:
        name=part.label
        color=palette['colors'].get(name)
        if name.startswith(('bearing_carrier','bearing_roll_adapter','bearing_shared','bearing_ankle_adapter','bearing_head_')):color=palette['new_printed']
        if name.startswith(('bearing_6704','bearing_screws')):color=palette['bearing']
        if color:paint(part,color)
    return root

def paint_original(shape, occurrence):
    """Modified existing parts retain their simulator color even when viewed alone."""
    declare_input(PALETTE)
    color=json.loads(PALETTE.read_text())['colors'][occurrence]
    def apply(node):
        node.color=color
        for child in node.children:apply(child)
    apply(shape)
    return shape
