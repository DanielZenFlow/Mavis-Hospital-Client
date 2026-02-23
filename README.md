# MAvis Hospital Client

A **Multi-Agent Path Finding (MAPF)** solver for the hospital robotics domain, built as a DTU course project. Multiple agents (robots `0`–`9`) coordinate to push/pull boxes (`A`–`Z`) to goal positions on a grid-based map, communicating with a competition server via stdin/stdout.

## Features

- **Portfolio-based strategy selection** — automatically picks the best solving approach based on level analysis
- **Independence detection** — decomposes multi-agent problems into independent subproblems via BFS + union-find
- **Conflict-Based Search (CBS)** — two-level optimal multi-agent planner for loosely coupled agents
- **Priority Planning** — subgoal decomposition with topological ordering, distance-greedy, and multi-seed random orderings
- **Hungarian algorithm** — globally optimal box-goal assignment with automatic fallback
- **True-distance heuristic** — BFS-precomputed shortest paths respecting walls
- **Deadlock detection & resolution** — cycle detection, corner pruning, and agent coordination

## Requirements

- **Java** 17+
- **Maven** 3.6+
- **Server JAR** (`server.jar`) — the MAvis competition server

## Build

```bash
# Compile
mvn compile

# Package executable JAR
mvn package -q -DskipTests
```

## Run

The client is designed to run with the MAvis competition server:

```bash
# Basic usage
java -jar server.jar -l <level_file> \
  -c "java -Xmx4g -jar target/mavis-hospital-client-1.0.jar" \
  -g -s 500 -t 180

# Development mode (using classpath)
java -jar server.jar -l <level_file> \
  -c "java -Xmx4g -cp target/classes mapf.client.Client" \
  -g -s 500 -t 60
```

### Server Flags

| Flag | Description |
|------|-------------|
| `-l` | Path to the level file (`.lvl`) |
| `-c` | Client command to execute |
| `-g` | Enable GUI |
| `-s` | Action speed in ms (lower = faster) |
| `-t` | Timeout in seconds |

### Examples

```bash
# Single-agent level
java -jar server.jar -l levels/SAbotbot.lvl \
  -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 500 -t 60

# Multi-agent level
java -jar server.jar -l levels/MAsimple1.lvl \
  -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 100 -t 30

# Competition level
java -jar server.jar -l complevels/ZOOM.lvl \
  -c "java -Xmx4g -jar target/mavis-hospital-client-1.0.jar" -g -s 500 -t 180
```

## Project Structure

```
src/main/java/mapf/
├── client/
│   ├── Client.java                 # Entry point: parse → plan → send actions
│   └── LevelParser.java            # .lvl file parser
├── domain/                          # Immutable data model
│   ├── Action.java                  # Move / Push / Pull / NoOp
│   ├── Color.java                   # 10-color enum
│   ├── Direction.java               # N / S / E / W with opposite()
│   ├── Level.java                   # Static map: walls, goals, colors
│   ├── Position.java                # (row, col) immutable value
│   └── State.java                   # Immutable world state
├── planning/
│   ├── SearchStrategy.java          # Strategy interface
│   ├── SearchConfig.java            # Timeouts, weights, limits
│   ├── StrategySelector.java        # Legacy single-strategy factory
│   ├── PortfolioController.java     # Multi-strategy fallback controller
│   ├── analysis/
│   │   ├── LevelAnalyzer.java       # Dependency & bottleneck analysis
│   │   ├── DependencyAnalyzer.java  # Goal dependency graph
│   │   ├── TaskFilter.java          # Satisfied goals & immovable boxes
│   │   └── CrossColorBarrierAnalyzer.java
│   ├── heuristic/
│   │   ├── Heuristic.java           # Heuristic interface
│   │   ├── ManhattanHeuristic.java  # Fast Manhattan distance sum
│   │   └── TrueDistanceHeuristic.java # BFS-precomputed wall-aware distances
│   ├── strategy/
│   │   ├── SingleAgentStrategy.java      # A* for single-agent levels
│   │   ├── JointAStarStrategy.java       # Joint-space A* (≤3 agents)
│   │   ├── PriorityPlanningStrategy.java # Main multi-agent solver
│   │   ├── SimplePriorityStrategy.java   # Lightweight priority planner
│   │   ├── GreedyPlanner.java            # Greedy fallback
│   │   ├── SubgoalManager.java           # Task ordering strategies
│   │   ├── BoxSearchPlanner.java         # Single-box push/pull planner
│   │   ├── HungarianAlgorithm.java       # Optimal assignment solver
│   │   ├── HungarianBoxAssigner.java     # Box-goal assignment via Hungarian
│   │   ├── ConflictResolver.java         # Joint action validation
│   │   ├── DeadlockBreaker.java          # Cycle detection & resolution
│   │   ├── AgentCoordinator.java         # Agent yielding & priority
│   │   ├── PathAnalyzer.java             # Parking, corridors, path planning
│   │   ├── PlanMerger.java               # Merge independent agent plans
│   │   ├── GoalChecker.java              # Goal satisfaction checks
│   │   ├── ImmovableBoxDetector.java     # Detect permanently placed boxes
│   │   ├── ArticulationPointFinder.java  # Graph cut-vertex analysis
│   │   └── PlanningUtils.java            # Shared utilities
│   ├── cbs/
│   │   ├── CBSStrategy.java        # Conflict-Based Search (high-level)
│   │   └── SpaceTimeAStar.java      # (x, y, t) low-level planner
│   ├── coordination/
│   │   ├── ConflictDetector.java    # Space-time collision detection
│   │   └── DeadlockResolver.java    # Multi-agent deadlock resolution
│   ├── goal/
│   │   ├── Goal.java                # Goal representation
│   │   └── GoalExtractor.java       # Extract goals from level
│   ├── pathfinding/
│   │   ├── AStar.java               # Core A* implementation
│   │   ├── AStarPathPlanner.java    # A*-based path planner
│   │   ├── JointAStar.java          # Multi-agent joint A*
│   │   └── PathPlanner.java         # Path planner interface
│   └── spacetime/
│       └── ReservationTable.java    # Space-time reservation tracking
```

## Architecture

### Solving Pipeline

```
Level File (.lvl)
    │
    ▼
Client.java → LevelParser → Level + initial State
    │
    ▼
PortfolioController.search()
    │
    ├── LevelAnalyzer.analyze() → LevelFeatures (dependencies, coupling, bottlenecks)
    │
    ├── Independence Detection (BFS + union-find)
    │   ├── Independent groups? → solve each separately → merge plans
    │   └── Single group? → continue to strategy portfolio
    │
    └── Strategy Portfolio (try in sequence with time budgets)
        ├── Single Agent     → weighted A*
        ├── Acyclic deps     → Topological → Distance Greedy → Random
        └── Cyclic deps      → Random#42 → Random#137 → Distance Greedy → Topological
    │
    ▼
List<Action[]> → stdout to server
```

### Strategy Overview

| Strategy | Best For | Complexity |
|----------|----------|------------|
| **SingleAgentStrategy** | 1 agent | A* search |
| **JointAStarStrategy** | ≤3 tightly coupled agents | $O(5^n)$ branching |
| **CBSStrategy** | Loosely coupled agents | Optimal, exponential worst-case |
| **PriorityPlanningStrategy** | General multi-agent | Linear in goals, A* per subgoal |
| **SimplePriorityStrategy** | Lightweight fallback | Fast, less optimal |

### Domain Rules

- **Color constraint**: Agents can only interact with boxes of the **same color**
- **Actions**: `Move(dir)`, `Push(agentDir, boxDir)`, `Pull(agentDir, boxDir)`, `NoOp`
- **Joint actions**: All agents act simultaneously per timestep
- **Conflicts**: Overlapping moves → both agents receive `NoOp`
- **Hard limits**: 3 min timeout, 20,000 joint actions, max 50×50 grid, max 10 agents

## Level Files

Level files use the `.lvl` format with sections:

```
#domain
hospital
#colors
blue: 0, A, B
red: 1, C
#initial
++++++
+0A  +
+ BC1+
++++++
#goal
++++++
+  A +
+  CB+
++++++
#end
```

- `levels/` — warmup and test levels
- `complevels/` — competition levels

## Configuration

| Environment Variable | Default | Description |
|---------------------|---------|-------------|
| `USE_PORTFOLIO` | `true` | Use portfolio controller (multi-strategy fallback) |

## Performance Notes

- **Memory**: `-Xmx4g` recommended for large levels
- **TrueDistanceHeuristic**: $O(\text{Goals} \times \text{Rows} \times \text{Cols})$ precomputation, cached at level load
- **State hashing**: Pre-computed in constructor for $O(1)$ amortized closed-list lookup
- **JointA\***: Branching factor of $5^n$ — practical only for ≤3 agents
- **CBS**: Exponential in conflict count — best for loosely coupled agents
- **Priority Planning**: Linear in goal count; each subgoal solved with A*

## License

Course project — DTU Artificial Intelligence and Multi-Agent Systems.
