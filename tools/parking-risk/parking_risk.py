#!/usr/bin/env python3
"""
Build an offline parking-risk report for Hospital-domain .lvl files.

The report is meant for debugging temporary box parking choices. It marks cells
that are risky because parking a box there would close a door-like cell in the
map, either in the static wall layout or in the current initial box layout.

Typical use:

    python tools/parking-risk/parking_risk.py complevels/ISO.lvl
    python tools/parking-risk/parking_risk.py --all
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Pos = Tuple[int, int]
GROUPS = ("levels", "complevels")
OUT_REL = Path("target/diagnostics/parking-risk")


def find_repo_root(start: Path) -> Path:
    current = start.resolve()
    for candidate in [current, *current.parents]:
        has_levels = (candidate / "levels").is_dir() or (candidate / "complevels").is_dir()
        has_marker = (candidate / "pom.xml").is_file() or (candidate / ".git").exists()
        if has_levels and has_marker:
            return candidate
    return current


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return path.read_text(encoding="latin-1")


def parse_level(path: Path, root: Path) -> Dict:
    text = read_text(path)
    section: Optional[str] = None
    level_name = path.stem
    color_lines: List[str] = []
    initial: List[str] = []
    goal: List[str] = []

    for line in text.splitlines():
        if line.startswith("#"):
            section = line[1:].strip().lower()
            if section == "end":
                break
            continue
        if section == "levelname" and line.strip():
            level_name = line.strip()
        elif section == "colors":
            color_lines.append(line)
        elif section == "initial":
            initial.append(line)
        elif section == "goal":
            goal.append(line)

    while initial and not initial[-1].strip():
        initial.pop()
    while goal and not goal[-1].strip():
        goal.pop()
    if not initial:
        raise ValueError(f"{path} has no #initial section")
    if not goal:
        raise ValueError(f"{path} has no #goal section")

    rows = max(len(initial), len(goal))
    cols = max(max((len(line) for line in initial), default=0),
               max((len(line) for line in goal), default=0))

    agent_colors: Dict[str, str] = {}
    box_colors: Dict[str, str] = {}
    for raw in color_lines:
        if ":" not in raw:
            continue
        color, items = raw.split(":", 1)
        color = color.strip().upper()
        for item in items.split(","):
            token = item.strip()
            if not token:
                continue
            ch = token[0]
            if ch.isdigit():
                agent_colors[ch] = color
            elif "A" <= ch <= "Z":
                box_colors[ch] = color

    walls: Set[Pos] = set()
    agents: List[Dict] = []
    boxes: List[Dict] = []
    agent_goals: List[Dict] = []
    box_goals: List[Dict] = []
    goal_cells: Set[Pos] = set()

    for r in range(rows):
        iline = initial[r] if r < len(initial) else ""
        gline = goal[r] if r < len(goal) else ""
        for c in range(cols):
            ic = iline[c] if c < len(iline) else " "
            gc = gline[c] if c < len(gline) else " "
            if ic == "+" or gc == "+":
                walls.add((r, c))
            if ic.isdigit():
                agents.append({"r": r, "c": c, "id": ic, "color": agent_colors.get(ic, "DEFAULT")})
            elif "A" <= ic <= "Z":
                boxes.append({"r": r, "c": c, "type": ic, "color": box_colors.get(ic, "DEFAULT")})
            if gc.isdigit():
                agent_goals.append({"r": r, "c": c, "id": gc})
                goal_cells.add((r, c))
            elif "A" <= gc <= "Z":
                box_goals.append({"r": r, "c": c, "type": gc})
                goal_cells.add((r, c))

    matching_agent_colors = {a["color"] for a in agents}
    immovable: Set[Pos] = set()
    for box in boxes:
        if box.get("color") not in matching_agent_colors:
            immovable.add((box["r"], box["c"]))

    return {
        "name": level_name,
        "path": str(path.resolve().relative_to(root)),
        "rows": rows,
        "cols": cols,
        "walls": sorted_dict_positions(walls),
        "initialGrid": [line.ljust(cols) for line in initial],
        "goalGrid": [line.ljust(cols) for line in goal],
        "agents": agents,
        "boxes": boxes,
        "agentGoals": agent_goals,
        "boxGoals": box_goals,
        "goalCells": sorted_dict_positions(goal_cells),
        "immovableBoxes": sorted_dict_positions(immovable),
        "agentColors": agent_colors,
        "boxColors": box_colors,
        "_sets": {
            "walls": walls,
            "goals": goal_cells,
            "immovable": immovable,
            "boxes": {(b["r"], b["c"]) for b in boxes},
            "agents": {(a["r"], a["c"]) for a in agents},
        },
    }


def sorted_dict_positions(points: Iterable[Pos]) -> List[Dict[str, int]]:
    return [{"r": r, "c": c} for r, c in sorted(points)]


def neighbors(pos: Pos) -> Iterable[Pos]:
    r, c = pos
    yield r - 1, c
    yield r + 1, c
    yield r, c - 1
    yield r, c + 1


def in_bounds(pos: Pos, rows: int, cols: int) -> bool:
    r, c = pos
    return 0 <= r < rows and 0 <= c < cols


def passable(pos: Pos, rows: int, cols: int, walls: Set[Pos], extra_walls: Set[Pos]) -> bool:
    return in_bounds(pos, rows, cols) and pos not in walls and pos not in extra_walls


def articulation_points(rows: int, cols: int, walls: Set[Pos], extra_walls: Set[Pos]) -> Set[Pos]:
    cells: List[Pos] = []
    index: Dict[Pos, int] = {}
    for r in range(rows):
        for c in range(cols):
            p = (r, c)
            if passable(p, rows, cols, walls, extra_walls):
                index[p] = len(cells)
                cells.append(p)
    n = len(cells)
    if n <= 2:
        return set()

    adj: List[List[int]] = [[] for _ in range(n)]
    for i, p in enumerate(cells):
        for q in neighbors(p):
            j = index.get(q)
            if j is not None:
                adj[i].append(j)

    disc = [-1] * n
    low = [-1] * n
    parent = [-1] * n
    aps: Set[Pos] = set()
    timer = 0

    def dfs(u: int, root: int) -> int:
        nonlocal timer
        disc[u] = low[u] = timer
        timer += 1
        children = 0
        for v in adj[u]:
            if disc[v] == -1:
                parent[v] = u
                children += 1
                dfs(v, root)
                low[u] = min(low[u], low[v])
                if u != root and low[v] >= disc[u]:
                    aps.add(cells[u])
            elif v != parent[u]:
                low[u] = min(low[u], disc[v])
        if u == root and children >= 2:
            aps.add(cells[u])
        return children

    for i in range(n):
        if disc[i] == -1:
            dfs(i, i)
    return aps


def component_sizes_after_removal(
    cell: Pos,
    rows: int,
    cols: int,
    walls: Set[Pos],
    extra_walls: Set[Pos],
) -> List[int]:
    blocked = set(extra_walls)
    blocked.add(cell)
    starts = [p for p in neighbors(cell) if passable(p, rows, cols, walls, blocked)]
    seen: Set[Pos] = set()
    sizes: List[int] = []
    for start in starts:
        if start in seen:
            continue
        q = deque([start])
        seen.add(start)
        size = 0
        while q:
            p = q.popleft()
            size += 1
            for nxt in neighbors(p):
                if nxt in seen:
                    continue
                if not passable(nxt, rows, cols, walls, blocked):
                    continue
                seen.add(nxt)
                q.append(nxt)
        sizes.append(size)
    sizes.sort(reverse=True)
    return sizes


def is_corner(pos: Pos, rows: int, cols: int, walls: Set[Pos], extra_walls: Set[Pos]) -> bool:
    r, c = pos
    north = not passable((r - 1, c), rows, cols, walls, extra_walls)
    south = not passable((r + 1, c), rows, cols, walls, extra_walls)
    west = not passable((r, c - 1), rows, cols, walls, extra_walls)
    east = not passable((r, c + 1), rows, cols, walls, extra_walls)
    return (north or south) and (west or east)


def count_free_neighbors(pos: Pos, rows: int, cols: int, walls: Set[Pos], extra_walls: Set[Pos]) -> int:
    return sum(1 for p in neighbors(pos) if passable(p, rows, cols, walls, extra_walls))


def component_sizes_if_opened(
    cell: Pos,
    rows: int,
    cols: int,
    walls: Set[Pos],
    extra_walls: Set[Pos],
) -> List[int]:
    """Component sizes adjacent to an occupied cell if that cell were opened."""
    blocked = set(extra_walls)
    blocked.discard(cell)
    starts = [p for p in neighbors(cell) if passable(p, rows, cols, walls, blocked)]
    seen: Set[Pos] = set()
    sizes: List[int] = []
    for start in starts:
        if start in seen:
            continue
        q = deque([start])
        seen.add(start)
        size = 0
        while q:
            p = q.popleft()
            size += 1
            for nxt in neighbors(p):
                if nxt in seen:
                    continue
                if not passable(nxt, rows, cols, walls, blocked):
                    continue
                seen.add(nxt)
                q.append(nxt)
        sizes.append(size)
    sizes.sort(reverse=True)
    return sizes


def analyze(level: Dict) -> Dict:
    rows = level["rows"]
    cols = level["cols"]
    sets = level["_sets"]
    walls: Set[Pos] = sets["walls"]
    goals: Set[Pos] = sets["goals"]
    immovable: Set[Pos] = sets["immovable"]
    boxes: Set[Pos] = sets["boxes"]
    agents: Set[Pos] = sets["agents"]

    static_ap = articulation_points(rows, cols, walls, immovable)
    dynamic_ap = articulation_points(rows, cols, walls, boxes)

    candidates: List[Dict] = []
    occupied_risks: List[Dict] = []
    counts = {
        "safe": 0,
        "caution": 0,
        "danger": 0,
        "blocked": 0,
        "occupiedDanger": 0,
        "occupiedCaution": 0,
        "staticArticulation": len(static_ap),
        "dynamicArticulation": len(dynamic_ap),
    }

    box_by_pos = {(b["r"], b["c"]): b for b in level["boxes"]}
    agent_by_pos = {(a["r"], a["c"]): a for a in level["agents"]}

    for r in range(rows):
        for c in range(cols):
            p = (r, c)
            if p in walls:
                continue
            if p in boxes or p in agents or p in goals:
                counts["blocked"] += 1
                occupant = None
                if p in boxes:
                    b = box_by_pos[p]
                    occupant = {"kind": "box", "label": b["type"], "color": b.get("color", "DEFAULT")}
                elif p in agents:
                    a = agent_by_pos[p]
                    occupant = {"kind": "agent", "label": a["id"], "color": a.get("color", "DEFAULT")}
                elif p in goals:
                    occupant = {"kind": "goal", "label": "goal", "color": "DEFAULT"}

                reasons: List[str] = []
                severity = "occupied"
                score = 0
                static_parts: List[int] = []
                current_parts: List[int] = []

                if p in static_ap:
                    severity = "danger"
                    score = max(score, 95)
                    reasons.append("occupied-static-door: this occupied cell is a wall-layout door")
                    static_parts = component_sizes_after_removal(p, rows, cols, walls, immovable)

                if p in boxes:
                    current_parts = component_sizes_if_opened(p, rows, cols, walls, boxes)
                    if len(current_parts) >= 2:
                        if severity != "danger":
                            severity = "caution"
                        score = max(score, 80)
                        reasons.append("occupied-current-blocker: removing this box would connect separated walking regions")

                if reasons:
                    if severity == "danger":
                        counts["occupiedDanger"] += 1
                    else:
                        counts["occupiedCaution"] += 1
                    occupied_risks.append({
                        "r": r,
                        "c": c,
                        "severity": severity,
                        "score": score,
                        "occupant": occupant,
                        "reasons": reasons,
                        "staticSplit": static_parts,
                        "currentOpenSplit": current_parts,
                    })
                continue

            reasons: List[str] = []
            severity = "safe"
            score = 0
            static_parts: List[int] = []
            dynamic_parts: List[int] = []

            if p in static_ap:
                severity = "danger"
                score = max(score, 95)
                reasons.append("static-door: removing this cell splits the wall-only map")
                static_parts = component_sizes_after_removal(p, rows, cols, walls, immovable)
            if p in dynamic_ap:
                if severity != "danger":
                    severity = "caution"
                score = max(score, 75)
                reasons.append("current-door: with initial boxes as blockers, this cell splits walking space")
                dynamic_parts = component_sizes_after_removal(p, rows, cols, walls, boxes)
            if is_corner(p, rows, cols, walls, immovable):
                if severity == "safe":
                    severity = "caution"
                score = max(score, 60)
                reasons.append("corner: easy to create a stuck box if it is not a real goal")

            if not reasons:
                reasons.append("no static risk found")

            counts[severity] += 1
            candidates.append({
                "r": r,
                "c": c,
                "severity": severity,
                "score": score,
                "freeNeighbors": count_free_neighbors(p, rows, cols, walls, boxes),
                "reasons": reasons,
                "staticSplit": static_parts,
                "dynamicSplit": dynamic_parts,
            })

    level_public = {k: v for k, v in level.items() if k != "_sets"}
    level_public["analysis"] = {
        "coordinateSystem": "Java zero-based grid coordinates: top-left level grid cell is (0,0)",
        "staticArticulation": sorted_dict_positions(static_ap),
        "dynamicArticulation": sorted_dict_positions(dynamic_ap),
        "candidates": candidates,
        "occupiedRisks": occupied_risks,
        "summary": counts,
    }
    return level_public


def collect_level_paths(root: Path, args: argparse.Namespace) -> List[Path]:
    if args.all:
        paths: List[Path] = []
        for group in GROUPS:
            group_dir = root / group
            if group_dir.is_dir():
                paths.extend(sorted(group_dir.glob("*.lvl"), key=lambda p: str(p).lower()))
        return paths
    if args.levels:
        return [(root / p).resolve() if not p.is_absolute() else p.resolve() for p in args.levels]
    default_iso = root / "complevels" / "ISO.lvl"
    if default_iso.is_file():
        return [default_iso]
    raise SystemExit("No level specified. Pass a .lvl path or use --all.")


def copy_viewer_assets(tool_dir: Path, out_dir: Path, refresh: bool) -> None:
    for name in ("index.html", "viewer.css", "viewer.js"):
        src = tool_dir / name
        dst = out_dir / name
        if refresh or not dst.exists():
            dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("levels", nargs="*", type=Path, help="Level .lvl file(s), relative to repo root or absolute")
    parser.add_argument("--all", action="store_true", help="Analyze every .lvl under levels/ and complevels/")
    parser.add_argument("--root", type=Path, default=None, help="Repository root, auto-detected if omitted")
    parser.add_argument("--out", type=Path, default=None, help="Output directory, default target/diagnostics/parking-risk")
    parser.add_argument("--refresh-viewer", action="store_true", help="Overwrite viewer HTML/CSS/JS assets")
    parser.add_argument("--no-viewer", action="store_true", help="Only write parking-risk-data.js")
    args = parser.parse_args()

    tool_dir = Path(__file__).resolve().parent
    root = args.root.resolve() if args.root else find_repo_root(tool_dir)
    out_dir = args.out.resolve() if args.out else root / OUT_REL
    out_dir.mkdir(parents=True, exist_ok=True)

    paths = collect_level_paths(root, args)
    reports = []
    for path in paths:
        if not path.is_file():
            raise SystemExit(f"Level not found: {path}")
        reports.append(analyze(parse_level(path, root)))

    data_path = out_dir / "parking-risk-data.js"
    payload = json.dumps(reports, ensure_ascii=False, indent=2)
    data_path.write_text(
        "// Auto-generated by tools/parking-risk/parking_risk.py\n"
        f"// Reports: {len(reports)}\n"
        f"window.PARKING_RISK_REPORTS = {payload};\n",
        encoding="utf-8",
    )
    if not args.no_viewer:
        copy_viewer_assets(tool_dir, out_dir, args.refresh_viewer)

    print(f"Wrote {data_path.relative_to(root)} ({len(reports)} report{'s' if len(reports) != 1 else ''})")
    if not args.no_viewer:
        print(f"Open {out_dir / 'index.html'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
