# CLAUDE.md - Project Intelligence for MAPF Hospital Client

## Project Overview

This is a **Multi-Agent Path Finding (MAPF)** solver for a hospital robotics domain.
Multiple agents (robots 0-9) coordinate to push/pull boxes (A-Z) to goal positions
on a grid-based map. The system communicates with a competition server via stdin/stdout.

**Language**: Java 17 · **Build**: Maven · **Entry Point**: `mapf.client.Client`

## Critical Domain Rules

These rules are **non-negotiable** and must never be violated:

1. **Color constraint**: Agents can ONLY push/pull boxes of the **same color**.
   Use `level.getAgentColor(agentId) == level.getBoxColor(boxType)` before any agent-box interaction.
2. **Actions**: Move(dir), Push(agentDir, boxDir), Pull(agentDir, boxDir), NoOp.
3. **Conflicts**: Two agents moving into the same cell → both get NoOp.
   Two agents pushing boxes into the same cell → both get NoOp.
   Two agents trying to move the same box → both get NoOp.
4. **Joint actions are synchronous**: All agents act simultaneously per timestep.
5. **Hard limits**: 3 minutes, 20,000 joint actions, max 50×50 grid, max 10 agents.
6. **Success priority**: Solve count > action count > computation time.

## Build and Run

```bash
# Compile
mvn compile

# Package JAR
mvn package -q -DskipTests

# Run with server (competition mode)
java -jar server.jar -l <level.lvl> -c "java -Xmx4g -jar target/mavis-hospital-client-1.0.jar" -g -s 500 -t 60

# Run with classpath (dev mode)
java -jar server.jar -l <level.lvl> -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 500 -t 60
```

**Regression test level**: `levels/SAbotbot.lvl` (single agent, push/pull, ~0.5s)
**Multi-agent test**: `levels/MAsimple1.lvl` (2 agents, CBS, ~0.2s)

## Architecture

```
mapf/
├── client/
│   └── Client.java              # Entry point: parse level → plan → send actions
├── domain/                      # Immutable data model
│   ├── Action.java              # Move/Push/Pull/NoOp with direction pairs
│   ├── Color.java               # 10 colors enum
│   ├── Direction.java           # N/S/E/W with opposite()
│   ├── Level.java               # Static map: walls, goals, colors. canManipulate()
│   ├── Position.java            # (row, col) immutable, manhattanDistance()
│   └── State.java               # Immutable world state: agent positions + box map
├── planning/
│   ├── SearchStrategy.java      # Interface: search(State, Level) → List<Action[]>
│   ├── SearchConfig.java        # Timeouts, weights, limits constants
│   ├── StrategySelector.java    # Factory: LevelAnalyzer → pick strategy
│   ├── PortfolioController.java # Multi-strategy fallback controller
│   ├── analysis/
│   │   ├── LevelAnalyzer.java   # Pre-analysis: dependencies, bottlenecks, coupling
│   │   ├── TaskFilter.java      # Identifies satisfied goals + immovable boxes
│   │   └── DependencyAnalyzer.java
│   ├── heuristic/
│   │   ├── Heuristic.java       # Interface: estimate(State, Level) → int
│   │   ├── ManhattanHeuristic.java    # Sum of Manhattan distances (fast, weak)
│   │   └── TrueDistanceHeuristic.java # BFS-precomputed shortest paths (slower, accurate)
│   ├── strategy/
│   │   ├── SingleAgentStrategy.java      # A* for 1-agent levels
│   │   ├── JointAStarStrategy.java       # Joint-space A* for ≤3 tightly coupled agents
│   │   ├── PriorityPlanningStrategy.java # Sequential subgoal decomposition (main solver)
│   │   ├── SimplePriorityStrategy.java   # Lightweight alternative priority planner
│   │   ├── SubgoalManager.java           # Task ordering using LevelAnalyzer topology
│   │   ├── BoxSearchPlanner.java         # Single-box push/pull path planning
│   │   ├── ConflictResolver.java         # Joint action conflict checks
│   │   ├── DeadlockBreaker.java          # Cycle detection + resolution
│   │   ├── AgentCoordinator.java         # Clear blocking agents
│   │   └── ...                           # Other helper classes
│   ├── cbs/
│   │   ├── CBSStrategy.java     # Conflict-Based Search (two-level)
│   │   └── SpaceTimeAStar.java  # Low-level (x,y,t) planner for CBS
│   └── coordination/
│       ├── ConflictDetector.java
│       ├── AgentYieldingManager.java
│       ├── DeadlockResolver.java
│       └── SafeZoneCalculator.java
```

## Strategy Selection Flow

```
Client.searchWithFallback()
    → PortfolioController.search()  (default, USE_PORTFOLIO=true)
        → LevelAnalyzer.analyze() → LevelFeatures
        → buildStrategySequence() → ordered list of (StrategyType, weight)
        → try each strategy with timeout budget, return first success

Fallback sequences by recommendation:
    SINGLE_AGENT  → [A*(w=1), A*(w=5)]
    CBS           → [CBS(w=1), PP(w=1), PP(w=2)]
    JOINT_SEARCH  → [JointA*(w=1), CBS(w=1), PP(w=1)]
    STRICT_ORDER  → [PP(w=1), PP(w=2), Greedy(w=5)]
    CYCLE_BREAKER → [PP(w=1), JointA*(w=2)¹, Greedy(w=5)]
    GREEDY        → [Greedy(w=1), Greedy(w=2), Greedy(w=5)]

    ¹ Only if ≤4 agents
```

Legacy path: `USE_PORTFOLIO=false` env var → StrategySelector (single strategy, no fallback).

## Key Algorithms

### LevelAnalyzer (Pre-planning Analysis)

**Reachability-based dependency analysis**:
- For each goal pair (Gi, Gj): hypothetically block Gi, check if box can still
  be pushed into Gj via BFS reachability to adjacent push positions.
- If blocking Gi makes Gj unreachable → Gi depends on Gj (fill Gj first).
- Produces a DAG → topological sort → goal execution order.

**Immovable boxes**: Detected by TaskFilter. Boxes that are already on their goal
AND cannot be moved without making the goal unsatisfiable are treated as walls.

**Coupling degree**: 0.0 (independent) to 1.0 (fully coupled).
Weighted: 40% dependency ratio + 30% dependency intensity + 30% bottleneck ratio.

### CBS (Conflict-Based Search)

**High level**: Priority queue of Constraint Tree (CT) nodes.
Each CT node = { per-agent constraints, per-agent solution paths, total cost }.

**Low level**: SpaceTimeAStar in (position, timestep) space with constraints.

**Task allocation**: Greedy nearest-match with **color constraint** enforcement.
Each agent assigned one box goal via `assignTasks()`.

**Conflict detection**: Checks agent body + moved boxes for vertex/edge conflicts.
Moved boxes identified by comparing current state vs initial state (t=0).

**Current limitations**:
- Single task per agent (no sequential multi-box delivery)
- Greedy task assignment (not optimal matching)
- No Sokoban-specific deadlock pruning

### PriorityPlanningStrategy (Main Multi-Agent Solver)

**Subgoal decomposition**: Uses LevelAnalyzer's topological order.
Plans one box-to-goal at a time using BoxSearchPlanner (A*).

**Conflict handling**: Multi-layered:
1. ConflictResolver for joint action validation
2. ConflictDetector for space-time collision avoidance
3. AgentCoordinator for clearing blocking agents
4. DeadlockBreaker for cycle detection and resolution

**Fallback**: On cyclic dependency → creates CBSStrategy for that subproblem.

## State Representation

States are **immutable**. Key design:
```java
class State {
    Map<Integer, Position> agentPositions;  // agent# → (row,col)
    Map<Position, Character> boxes;          // (row,col) → box letter
    int hashCode;  // pre-computed for efficient closed list lookup
}
```

Successors generated via `state.getSuccessors(agentId, level)` which returns
all applicable (Action, State) pairs respecting walls, box colors, and physics.

## Heuristics

| Heuristic | Method | Admissible | When to use |
|-----------|--------|------------|-------------|
| Manhattan | Σ(box→nearest goal) + Σ(agent→goal) | Yes | Fast baseline |
| BFS True Distance | Pre-computed BFS from each goal | Yes | Default. Respects walls |

Both **ignore box-blocking** (underestimate in congested levels).
ARCHITECTURE.md suggests adding `h_conflicts` — not yet implemented.

## Common Patterns

### Adding a new strategy
1. Implement `SearchStrategy` interface: `search(State, Level) → List<Action[]>`
2. Add enum value to `LevelAnalyzer.StrategyType`
3. Add case in `PortfolioController.buildStrategySequence()` (fallback sequence)
4. Add case in `PortfolioController.createStrategy()` (instantiation)
5. Set timeout/maxStates via `setTimeout()` / `setMaxStates()`

### Testing a level
```bash
# Single-agent level
java -jar server.jar -l levels/SAbotbot.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 500 -t 60

# Multi-agent level
java -jar server.jar -l levels/MAsimple1.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 100 -t 30

# Competition level
java -jar server.jar -l complevels/DECrunchy.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 500 -t 180
```

## Known Issues and TODOs

### P0 — Must Fix
- [x] Multi-strategy fallback: PortfolioController is now default entry point.
      CBS → PP → Greedy cascade with timeout budget.
- [ ] Weighted A* fallback: w=1.0 first, timeout → w=1.5 retry

### P1 — Should Fix
- [ ] Agent-level dependency analysis: map goal deps → agent deps via color/reachability,
      detect cross-agent push-route conflicts, enable independent subproblem decomposition
- [ ] Simple deadlock pruning: reject Push into corners/edges with no exit
      (ARCHITECTURE.md: "don't push boxes into corners")
- [ ] Conflict-aware heuristic: h += estimated conflict resolution cost

### P2 — Nice to Have
- [ ] Parking/buffer zones for high-density levels (DECrunchy-type)
- [ ] Independence detection: split disconnected agent groups into subproblems

### Not Needed (per project scope)
- Reverse/Pull search (pure Sokoban technique, not in requirements)
- Pattern Database heuristics (overkill for max 50×50)
- K-robust conflicts (synchronous model only)
- Hungarian algorithm for task assignment (≤10 agents, greedy sufficient)
- Symmetry breaking (small scale)

## File Conventions

- **Level files**: `levels/` (warmup), `complevels/` (competition)
- **Level format**: `.lvl` files with `#domain`, `#colors`, `#initial`, `#goal`, `#end`
- **Reference docs**: `PRODUCT.md` (domain rules), `ARCHITECTURE.md` (design guidance)
- **All States immutable** — never mutate a State after creation
- **Logging**: `System.err.println()` — stderr only (stdout is server protocol)

## Performance Notes

- `TrueDistanceHeuristic` precomputation: O(Goals × Rows × Cols). Cache at Level load.
- State hashing is pre-computed in constructor. HashMap closed-list is O(1) amortized.
- JointA* branching factor: (5 actions)^(n agents). Unusable above 3 agents.
- CBS high-level nodes: exponential in conflict count. Best for loosely coupled agents.
- PriorityPlanning: linear in goal count, but each subgoal is an A* search.
- Memory: `-Xmx4g` recommended. Large levels can generate millions of states.
