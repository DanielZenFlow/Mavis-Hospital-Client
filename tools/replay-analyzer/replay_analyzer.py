#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def agent_pos(frame: dict, agent_id: int) -> str:
    for agent in frame.get("agents", []):
        if agent.get("id") == agent_id:
            return f"{agent['r']},{agent['c']}"
    return "?"


def boxes_of(frame: dict, box_type: str) -> List[str]:
    return sorted(f"{b['r']},{b['c']}" for b in frame.get("boxes", []) if b.get("type") == box_type)


def full_state_key(frame: dict) -> str:
    agents = sorted(f"a{a['id']}@{a['r']},{a['c']}" for a in frame.get("agents", []))
    boxes = sorted(f"{b['type']}@{b['r']},{b['c']}" for b in frame.get("boxes", []))
    return "|".join(agents) + "#" + "|".join(boxes)


def diff_box_edges(frames: List[dict], box_type: str, start: int, end: int) -> Counter:
    edges: Counter = Counter()
    for i in range(max(1, start), min(end, len(frames) - 1) + 1):
        prev = set(boxes_of(frames[i - 1], box_type))
        cur = set(boxes_of(frames[i], box_type))
        gone = sorted(prev - cur)
        added = sorted(cur - prev)
        for a, b in zip(gone, added):
            edges[f"{a}->{b}"] += 1
    return edges


def repeated_states(frames: List[dict], start: int, end: int) -> List[Tuple[int, int, int]]:
    seen: Dict[str, int] = {}
    repeats: List[Tuple[int, int, int]] = []
    for i in range(start, min(end, len(frames) - 1) + 1):
        key = full_state_key(frames[i])
        if key in seen:
            first = seen[key]
            repeats.append((first, i, i - first))
        else:
            seen[key] = i
    return repeats


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze replay JSON for oscillations.")
    parser.add_argument("replay", type=Path)
    parser.add_argument("--agent", type=int, default=8)
    parser.add_argument("--box", default="L")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--until", type=int, default=None)
    parser.add_argument("--top", type=int, default=20)
    args = parser.parse_args()

    data = load(args.replay)
    frames = data.get("frames", [])
    if not frames:
        print("No frames found.")
        return 1
    end = args.until if args.until is not None else len(frames) - 1
    end = min(end, len(frames) - 1)
    start = max(0, args.start)

    print(f"Replay: {args.replay}")
    print(f"Frames analyzed: {start}..{end} / {len(frames) - 1}")
    print(f"Summary: {data.get('summary', {})}")

    actions = Counter()
    accepted = Counter()
    agent_positions = Counter()
    for i in range(start + 1, end + 1):
        frame = frames[i]
        acts = frame.get("actions", [])
        acc = frame.get("accepted", [])
        if args.agent < len(acts):
            actions[acts[args.agent]] += 1
        if args.agent < len(acc):
            accepted[str(acc[args.agent]).lower()] += 1
        agent_positions[agent_pos(frame, args.agent)] += 1

    print("")
    print(f"Agent {args.agent} actions:")
    for action, count in actions.most_common(args.top):
        print(f"  {count:5}  {action}")
    print(f"Agent {args.agent} accepted: {dict(accepted)}")
    print(f"Agent {args.agent} top positions:")
    for pos, count in agent_positions.most_common(min(args.top, 10)):
        print(f"  {count:5}  {pos}")

    print("")
    repeats = repeated_states(frames, start, end)
    long_repeats = [r for r in repeats if r[2] >= 10]
    print(f"Repeated full states: {len(repeats)} total, {len(long_repeats)} with period >= 10")
    for first, second, period in long_repeats[:args.top]:
        print(f"  {first:5} -> {second:5}  period={period}")

    print("")
    print(f"Box {args.box} repeated move edges:")
    for edge, count in diff_box_edges(frames, args.box, start, end).most_common(args.top):
        if count <= 1:
            break
        print(f"  {count:5}  {edge}")

    print("")
    for step in [start, min(end, start + 29), min(end, start + 84), min(end, start + 224), end]:
        frame = frames[step]
        action = frame.get("actions", [None] * (args.agent + 1))
        action_text = action[args.agent] if args.agent < len(action) else "-"
        print(f"step {step}: agent{args.agent}={agent_pos(frame, args.agent)} "
              f"{args.box}={boxes_of(frame, args.box)} action={action_text}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
