#!/usr/bin/env python3
"""Convert rescue_mission.xml BehaviorTree to a Graphviz PNG diagram."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
XML_PATH = SCRIPT_DIR / "src" / "rescue_bt" / "behavior_trees" / "rescue_mission.xml"
DOT_PATH = SCRIPT_DIR / "images" / "bt_groot.dot"
PNG_PATH = SCRIPT_DIR / "images" / "bt_groot.png"
DOT_EXE = r"C:\Program Files\Graphviz\bin\dot.exe"

CONTROL_NODES = {"Sequence", "ReactiveSequence", "ReactiveFallback"}
DECORATOR_NODES = {"Timeout"}
SUBTREE_NODES = {"SubTree"}

STYLES = {
    "control": {"shape": "box", "style": "rounded,filled", "fillcolor": "#FFD966", "fontname": "Arial"},
    "decorator": {"shape": "box", "style": "rounded,filled", "fillcolor": "#A4C2F4", "fontname": "Arial"},
    "action": {"shape": "box", "style": "rounded,filled", "fillcolor": "#B6D7A8", "fontname": "Arial"},
    "condition": {"shape": "diamond", "style": "filled", "fillcolor": "#F4CCCC", "fontname": "Arial"},
    "subtree": {"shape": "box", "style": "rounded,filled,dashed", "fillcolor": "#D5A6BD", "fontname": "Arial"},
}


def get_style(tag: str) -> dict:
    if tag in CONTROL_NODES:
        return STYLES["control"]
    if tag in DECORATOR_NODES:
        return STYLES["decorator"]
    if tag in SUBTREE_NODES:
        return STYLES["subtree"]
    if tag.startswith("Check"):
        return STYLES["condition"]
    return STYLES["action"]


def make_label(elem: ET.Element) -> str:
    tag = elem.tag
    name = elem.get("name", "")
    desc = elem.get("description", "")

    if tag == "SubTree":
        return f"SubTree\\n{elem.get('ID', '')}"
    if tag == "Timeout":
        secs = int(elem.get("msec", 0)) // 1000
        return f"Timeout\\n{secs}s"
    if tag == "NavigateToWaypoint":
        return f"Navigate\\n→ {desc}"
    if tag == "WaitSeconds":
        return f"Wait {elem.get('seconds', '')}s"
    if tag == "SetCameraAngle":
        angle = float(elem.get("angle", 0))
        deg = round(angle * 180 / 3.14159, 1)
        return f"SetCamera\\n{deg}°"
    if tag == "PublishObjectTF":
        return f"PublishTF\\n{elem.get('frame_name', '')}"
    if tag == "PublishDockStatus":
        return f"DockStatus\\n{elem.get('docked', '')}"
    if name:
        return f"{tag}\\n'{name}'"
    return tag


def walk(elem: ET.Element, lines: list, counters: dict) -> str:
    node_id = f"n{counters['id']}"
    counters["id"] += 1

    label = make_label(elem)
    style = get_style(elem.tag)
    attrs = ", ".join(f'{k}="{v}"' for k, v in style.items())
    lines.append(f'  {node_id} [label="{label}", {attrs}];')

    for child in elem:
        if child.tag is ET.Comment or not isinstance(child.tag, str):
            continue
        child_id = walk(child, lines, counters)
        lines.append(f"  {node_id} -> {child_id};")

    return node_id


def main():
    tree = ET.parse(XML_PATH)
    root = tree.getroot()

    lines = [
        "digraph BehaviorTree {",
        '  rankdir=TB;',
        '  node [fontsize=10, margin="0.15,0.07"];',
        '  edge [arrowsize=0.7];',
        "",
        "  // Legend",
        '  subgraph cluster_legend {',
        '    label="Legend"; fontname="Arial"; fontsize=10; style=rounded; color=gray;',
        '    lc [label="Control", shape=box, style="rounded,filled", fillcolor="#FFD966", fontname="Arial", fontsize=9];',
        '    ld [label="Decorator", shape=box, style="rounded,filled", fillcolor="#A4C2F4", fontname="Arial", fontsize=9];',
        '    la [label="Action", shape=box, style="rounded,filled", fillcolor="#B6D7A8", fontname="Arial", fontsize=9];',
        '    lk [label="Condition", shape=diamond, style=filled, fillcolor="#F4CCCC", fontname="Arial", fontsize=9];',
        '    ls [label="SubTree", shape=box, style="rounded,filled,dashed", fillcolor="#D5A6BD", fontname="Arial", fontsize=9];',
        '    lc -> ld -> la -> lk -> ls [style=invis];',
        "  }",
        "",
    ]

    counters = {"id": 0}

    for bt in root.findall("BehaviorTree"):
        bt_id = bt.get("ID", "Tree")
        lines.append(f"  // Tree: {bt_id}")
        for child in bt:
            if child.tag is ET.Comment or not isinstance(child.tag, str):
                continue
            walk(child, lines, counters)
        lines.append("")

    lines.append("}")

    DOT_PATH.parent.mkdir(parents=True, exist_ok=True)
    DOT_PATH.write_text("\n".join(lines), encoding="utf-8")

    subprocess.run(
        [DOT_EXE, "-Tpng", "-Gdpi=150", str(DOT_PATH), "-o", str(PNG_PATH)],
        check=True,
    )
    print(f"Generated: {PNG_PATH}")


if __name__ == "__main__":
    main()
