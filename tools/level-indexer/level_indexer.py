#!/usr/bin/env python3
"""
Index Hospital-domain .lvl files for test selection.

This intentionally lives under tools/level-indexer instead of the repository
root. It produces both machine-readable JSON and a compact Markdown report.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict, deque
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple


ROOT_DEFAULTS = ("levels", "complevels")
OUT_DIR_DEFAULT = Path("target/diagnostics/level-index")
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


Pos = Tuple[int, int]


def find_repo_root(start: Path) -> Path:
    """Find the repository root from any subdirectory.

    The indexer is often launched from tools/level-indexer directly, so using
    the current working directory as the repo root is too brittle.
    """
    current = start.resolve()
    for candidate in [current, *current.parents]:
        has_levels = (candidate / "levels").is_dir() or (candidate / "complevels").is_dir()
        has_project_marker = (candidate / "pom.xml").is_file() or (candidate / ".git").exists()
        if has_levels and has_project_marker:
            return candidate
    return current


@dataclass
class LevelIndex:
    path: str
    name: str
    source_dir: str
    rows: int
    cols: int
    free_cells: int
    wall_cells: int
    agents: int
    boxes: int
    box_goals: int
    agent_goals: int
    colors: int
    box_types: List[str]
    repeated_box_types: Dict[str, int]
    max_same_letter_count: int
    corridor_cells: int
    junction_cells: int
    deadend_cells: int
    corridor_ratio: float
    density: float
    possible_namo_blockers: int
    possible_namo_pairs: List[str]
    tags: List[str]
    score_for_tests: int


def read_sections(path: Path) -> Dict[str, List[str]]:
    sections: Dict[str, List[str]] = defaultdict(list)
    current: Optional[str] = None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if raw.startswith("#"):
            current = raw[1:].strip().lower()
            if current == "end":
                break
            continue
        if current:
            sections[current].append(raw.rstrip("\n"))
    return sections


def parse_colors(lines: Iterable[str]) -> Tuple[Dict[int, str], Dict[str, str]]:
    agent_colors: Dict[int, str] = {}
    box_colors: Dict[str, str] = {}
    for line in lines:
        if ":" not in line:
            continue
        color, rest = line.split(":", 1)
        color = color.strip().lower()
        for token in rest.split(","):
            token = token.strip()
            if not token:
                continue
            if token.isdigit():
                agent_colors[int(token)] = color
            elif len(token) == 1 and token.isalpha() and token.isupper():
                box_colors[token] = color
    return agent_colors, box_colors


def char_at(grid: List[str], r: int, c: int) -> str:
    if r < 0 or r >= len(grid):
        return " "
    line = grid[r]
    return line[c] if c < len(line) else " "


def parse_grid(initial: List[str], goal: List[str]) -> dict:
    rows = len(initial)
    cols = max((len(line) for line in initial), default=0)
    walls: Set[Pos] = set()
    agents: Dict[int, Pos] = {}
    boxes: Dict[Pos, str] = {}
    box_goals: Dict[Pos, str] = {}
    agent_goals: Dict[int, Pos] = {}

    for r in range(rows):
        for c in range(cols):
            ch = char_at(initial, r, c)
            if ch == "+":
                walls.add((r, c))
            elif ch.isdigit():
                agents[int(ch)] = (r, c)
            elif ch.isalpha() and ch.isupper():
                boxes[(r, c)] = ch

            g = char_at(goal, r, c)
            if g.isdigit():
                agent_goals[int(g)] = (r, c)
            elif g.isalpha() and g.isupper():
                box_goals[(r, c)] = g

    return {
        "rows": rows,
        "cols": cols,
        "walls": walls,
        "agents": agents,
        "boxes": boxes,
        "box_goals": box_goals,
        "agent_goals": agent_goals,
    }


def neighbors(pos: Pos, rows: int, cols: int, walls: Set[Pos]) -> List[Pos]:
    out: List[Pos] = []
    r, c = pos
    for dr, dc in DIRS:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in walls:
            out.append((nr, nc))
    return out


def reachable(start: Pos, rows: int, cols: int, walls: Set[Pos], blocked: Set[Pos]) -> Set[Pos]:
    if start in walls:
        return set()
    seen = {start}
    q = deque([start])
    while q:
        p = q.popleft()
        for n in neighbors(p, rows, cols, walls):
            if n in blocked or n in seen:
                continue
            seen.add(n)
            q.append(n)
    return seen


def adj_any(pos: Pos, cells: Set[Pos], rows: int, cols: int, walls: Set[Pos]) -> bool:
    return any(n in cells for n in neighbors(pos, rows, cols, walls))


def possible_namo(
    rows: int,
    cols: int,
    walls: Set[Pos],
    agents: Dict[int, Pos],
    boxes: Dict[Pos, str],
    box_goals: Dict[Pos, str],
    agent_colors: Dict[int, str],
    box_colors: Dict[str, str],
) -> Tuple[int, List[str]]:
    """Approximate cross-color blocker signal.

    For each unsatisfied same-color goal, compare reachability with boxes as
    walls vs boxes ignored. If the agent can theoretically reach the box/goal
    only when boxes are ignored, count other-color boxes adjacent to that
    theoretical region as possible NAMO blockers.
    """
    blockers: Set[Pos] = set()
    pairs: Set[str] = set()
    all_box_positions = set(boxes.keys())
    goals_by_type: Dict[str, List[Pos]] = defaultdict(list)
    for pos, typ in box_goals.items():
        goals_by_type[typ].append(pos)

    boxes_by_type: Dict[str, List[Pos]] = defaultdict(list)
    for pos, typ in boxes.items():
        boxes_by_type[typ].append(pos)

    for agent_id, agent_pos in agents.items():
        acolor = agent_colors.get(agent_id)
        if not acolor:
            continue
        actual = reachable(agent_pos, rows, cols, walls, all_box_positions)
        theoretical = reachable(agent_pos, rows, cols, walls, set())
        for goal_pos, goal_type in box_goals.items():
            if box_colors.get(goal_type) != acolor:
                continue
            if boxes.get(goal_pos) == goal_type:
                continue
            candidate_boxes = boxes_by_type.get(goal_type, [])
            if not candidate_boxes:
                continue
            box_pos = min(candidate_boxes, key=lambda p: abs(p[0] - goal_pos[0]) + abs(p[1] - goal_pos[1]))
            actual_ok = adj_any(box_pos, actual, rows, cols, walls) or adj_any(goal_pos, actual, rows, cols, walls)
            theoretical_ok = adj_any(box_pos, theoretical, rows, cols, walls) or adj_any(goal_pos, theoretical, rows, cols, walls)
            if actual_ok or not theoretical_ok:
                continue
            for bpos, btype in boxes.items():
                if bpos == box_pos:
                    continue
                if box_colors.get(btype) == acolor:
                    continue
                if bpos in theoretical and bpos not in actual:
                    blockers.add(bpos)
                    pairs.add(f"a{agent_id}:{goal_type}@{goal_pos} blocked-by {btype}@{bpos}")

    return len(blockers), sorted(pairs)[:12]


def tag_level(index: LevelIndex) -> List[str]:
    tags: List[str] = []
    area = index.rows * index.cols
    if area <= 120 and index.boxes <= 4:
        tags.append("tiny")
    elif area <= 300 and index.boxes <= 8:
        tags.append("small")
    if index.agents == 1:
        tags.append("single-agent")
    elif index.agents <= 3:
        tags.append("few-agent")
    else:
        tags.append("multi-agent")
    if index.colors == 1:
        tags.append("single-color")
    elif index.colors <= 3:
        tags.append("few-color")
    else:
        tags.append("multi-color")
    if index.max_same_letter_count >= 2:
        tags.append("same-letter-multiple")
    if index.possible_namo_blockers > 0:
        tags.append("possible-namo")
    if index.corridor_ratio >= 0.40:
        tags.append("corridor-heavy")
    if index.deadend_cells > 0:
        tags.append("deadends")
    if index.box_goals == index.boxes and index.agent_goals == 0:
        tags.append("box-only")
    return tags


def score_for_tests(index: LevelIndex) -> int:
    score = 0
    if "tiny" in index.tags:
        score += 35
    if "small" in index.tags:
        score += 25
    if "few-agent" in index.tags:
        score += 15
    if "single-agent" in index.tags:
        score += 12
    if "possible-namo" in index.tags:
        score += 20
    if "same-letter-multiple" in index.tags:
        score += 10
    if index.boxes <= 8:
        score += 10
    if index.deadend_cells > 0:
        score += 5
    if index.rows * index.cols > 800:
        score -= 25
    if index.boxes > 20:
        score -= 20
    return score


def index_file(path: Path, repo_root: Path) -> Optional[LevelIndex]:
    sections = read_sections(path)
    initial = sections.get("initial", [])
    goal = sections.get("goal", [])
    if not initial or not goal:
        return None

    level_name = sections.get("levelname", [path.stem])[0].strip() or path.stem
    agent_colors, box_colors = parse_colors(sections.get("colors", []))
    parsed = parse_grid(initial, goal)
    rows = parsed["rows"]
    cols = parsed["cols"]
    walls = parsed["walls"]
    agents = parsed["agents"]
    boxes = parsed["boxes"]
    box_goals = parsed["box_goals"]
    agent_goals = parsed["agent_goals"]

    free_cells = rows * cols - len(walls)
    degree_counts = Counter(len(neighbors((r, c), rows, cols, walls))
                            for r in range(rows) for c in range(cols)
                            if (r, c) not in walls)
    corridor_cells = degree_counts[2]
    junction_cells = sum(v for k, v in degree_counts.items() if k >= 3)
    deadend_cells = degree_counts[1]
    box_counts = Counter(boxes.values())
    possible_blockers, possible_pairs = possible_namo(
        rows, cols, walls, agents, boxes, box_goals, agent_colors, box_colors)

    source_dir = path.relative_to(repo_root).parts[0]
    idx = LevelIndex(
        path=str(path.relative_to(repo_root)).replace("\\", "/"),
        name=level_name,
        source_dir=source_dir,
        rows=rows,
        cols=cols,
        free_cells=free_cells,
        wall_cells=len(walls),
        agents=len(agents),
        boxes=len(boxes),
        box_goals=len(box_goals),
        agent_goals=len(agent_goals),
        colors=len(set(agent_colors.values()) | set(box_colors.values())),
        box_types=sorted(box_counts.keys()),
        repeated_box_types={k: v for k, v in sorted(box_counts.items()) if v >= 2},
        max_same_letter_count=max(box_counts.values(), default=0),
        corridor_cells=corridor_cells,
        junction_cells=junction_cells,
        deadend_cells=deadend_cells,
        corridor_ratio=round(corridor_cells / free_cells, 4) if free_cells else 0.0,
        density=round((len(boxes) + len(agents)) / free_cells, 4) if free_cells else 0.0,
        possible_namo_blockers=possible_blockers,
        possible_namo_pairs=possible_pairs,
        tags=[],
        score_for_tests=0,
    )
    idx.tags = tag_level(idx)
    idx.score_for_tests = score_for_tests(idx)
    return idx


def markdown_report(indexes: List[LevelIndex]) -> str:
    total = len(indexes)
    by_source = Counter(i.source_dir for i in indexes)
    by_tag = Counter(tag for i in indexes for tag in i.tags)
    top = sorted(indexes, key=lambda i: (-i.score_for_tests, i.rows * i.cols, i.boxes, i.path))[:40]
    namo = sorted(
        (i for i in indexes if "possible-namo" in i.tags),
        key=lambda i: (-i.possible_namo_blockers, -i.score_for_tests, i.rows * i.cols, i.path),
    )[:30]
    same_letter = sorted(
        (i for i in indexes if "same-letter-multiple" in i.tags),
        key=lambda i: (-i.score_for_tests, i.rows * i.cols, i.boxes, i.path),
    )[:30]

    def row(i: LevelIndex) -> str:
        return (
            f"| `{i.path}` | {i.name} | {i.rows}x{i.cols} | {i.agents} | {i.boxes} | "
            f"{i.box_goals} | {i.colors} | {i.corridor_ratio:.2f} | {i.possible_namo_blockers} | "
            f"{i.score_for_tests} | {', '.join(i.tags)} |"
        )

    lines: List[str] = []
    lines.append("# Level Index")
    lines.append("")
    lines.append(f"- Total levels: {total}")
    lines.append(f"- Sources: " + ", ".join(f"{k}={v}" for k, v in sorted(by_source.items())))
    lines.append(f"- Top tags: " + ", ".join(f"{k}={v}" for k, v in by_tag.most_common(12)))
    lines.append("")
    lines.append("## Best Test Candidates")
    lines.append("")
    lines.append("| path | name | size | agents | boxes | box goals | colors | corridor | NAMO | score | tags |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    lines.extend(row(i) for i in top)
    lines.append("")
    lines.append("## Possible NAMO Candidates")
    lines.append("")
    lines.append("| path | name | size | agents | boxes | box goals | colors | corridor | NAMO | score | tags |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    lines.extend(row(i) for i in namo)
    lines.append("")
    lines.append("## Same-Letter Multiple Box Candidates")
    lines.append("")
    lines.append("| path | name | size | agents | boxes | box goals | colors | corridor | NAMO | score | tags |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    lines.extend(row(i) for i in same_letter)
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Index Hospital .lvl files for testing.")
    parser.add_argument("--root", default=None,
                        help="Repository root. Default: auto-detect by walking upward from current directory.")
    parser.add_argument("--dirs", nargs="*", default=list(ROOT_DEFAULTS),
                        help="Directories to scan. Default: levels complevels.")
    parser.add_argument("--out", default=str(OUT_DIR_DEFAULT), help="Output directory.")
    args = parser.parse_args()

    repo_root = Path(args.root).resolve() if args.root else find_repo_root(Path.cwd())
    out_dir = (repo_root / args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    indexes: List[LevelIndex] = []
    scanned_roots: List[Path] = []
    for dirname in args.dirs:
        root = repo_root / dirname
        if not root.exists():
            continue
        scanned_roots.append(root)
        for path in sorted(root.rglob("*.lvl")):
            idx = index_file(path, repo_root)
            if idx is not None:
                indexes.append(idx)

    indexes.sort(key=lambda i: (i.source_dir, i.path.lower()))
    json_path = out_dir / "level-index.json"
    md_path = out_dir / "level-index.md"

    print(f"Repository root: {repo_root}")
    if scanned_roots:
        print("Scanned: " + ", ".join(str(p) for p in scanned_roots))
    else:
        print("Scanned: none (levels/complevels not found under repository root)")
    print(f"Indexed {len(indexes)} levels")
    try:
        json_path.write_text(json.dumps([asdict(i) for i in indexes], indent=2, ensure_ascii=False), encoding="utf-8")
        md_path.write_text(markdown_report(indexes), encoding="utf-8")
    except PermissionError as exc:
        print(f"ERROR: cannot write output under {out_dir}")
        print("Run from a shell with write permission to the repository, or pass --out <writable-dir>.")
        raise exc
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
