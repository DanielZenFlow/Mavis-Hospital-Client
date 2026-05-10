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
   Inapplicable actions fail and are executed as NoOp by the server.
3. **Conflicts**: If two or more agents try to move any moved object (agent or box)
   into the same cell, or try to move the same box, all involved agents fail and
   execute NoOp. The planner may pre-resolve a conflict by replacing selected
   actions with NoOp before sending the joint action.
4. **Joint actions are synchronous**: All agents act simultaneously per timestep.
   Applicability and occupancy are evaluated against the state at the start of
   the timestep; an agent cannot move into a cell another object leaves in the
   same joint action.
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
**Multi-agent test**: `levels/MAsimple1.lvl` (2 agents, PP, ~0.3s)

## Architecture

```
mapf/
├── client/
�?  ├── Client.java              # Entry point: parse level �?plan �?send actions
�?  └── LevelParser.java         # .lvl file parsing
├── domain/                      # L1 �?Immutable data model
�?  ├── Action.java              # Move/Push/Pull/NoOp with direction pairs
�?  ├── Color.java               # 10 colors enum
�?  ├── Direction.java           # N/S/E/W with opposite()
�?  ├── Level.java               # Static map: walls, goals, colors. canAgentMoveBox()
�?  ├── Position.java            # (row, col) immutable + flyweight (Position.of cache)
�?  └── State.java               # Immutable world state: agent positions + box map.
�?                               #   apply / applyJointAction (official sync/no-op/conflict semantics)
├── planning/
�?  ├── SearchStrategy.java      # Interface: search(State, Level) �?List<Action[]>
�?  ├── SearchConfig.java        # Timeouts, weights, limits, log level (isMinimal/isNormal)
�?  ├── PortfolioController.java # Top-level orchestrator: independence detection +
�?  �?                           # warm-start sequence + CBSR + F7-PRE + F7B
�?  ├── analysis/                # L3 �?Pre-planning analysis (static)
�?  �?  ├── LevelAnalyzer.java          # Reachability deps, coupling, StrategyType, executionOrder
�?  �?  ├── DependencyAnalyzer.java     # Goal �?goal dependency edges (block-and-test)
�?  �?  ├── TaskFilter.java             # Already-satisfied goals + immovable boxes
�?  �?  ├── ImmovableFusion.java        # Fuse immovable boxes into walls (lvl projection)
�?  �?  ├── CrossColorBarrierAnalyzer.java # Wrong-color box barriers in agent's pathway
�?  �?  └── GoalTransitAnalyzer.java    # Per-goal transit corridor / push-bay analysis
�?  ├── heuristic/               # L2 �?Heuristics
�?  �?  ├── Heuristic.java
�?  �?  ├── ManhattanHeuristic.java    # Fast baseline (admissible)
�?  �?  └── TrueDistanceHeuristic.java # BFS-precomputed; +2 penalty per wrong-type box on goal
�?  ├── pathfinding/             # L2 �?Generic search primitives
�?  �?  ├── PathPlanner.java            # Interface
�?  �?  ├── AStar.java                  # Generic A*
�?  �?  ├── AStarPathPlanner.java       # 2D agent-only A* with frozen-cell support
�?  �?  └── JointAStar.java             # Joint-space A* (used internally; not in portfolio)
�?  ├── spacetime/
�?  �?  └── ReservationTable.java       # (pos, t) reservation for ST-A*
�?  ├── strategy/                # L3/L4 �?Subgoal decomposition + coordination
�?  �?  ├── PriorityPlanningStrategy.java  # Main multi-agent solver (PP + multi-round BSP)
�?  �?  ├── SingleAgentStrategy.java       # Full-state A* for 1-agent levels
�?  �?  ├── SubgoalManager.java            # Subgoal ordering + Hungarian assignment
�?  �?  ├── BoxSearchPlanner.java          # Single-box push/pull A*/IDA* search
�?  �?  ├── IW1Planner.java                # Width-1 search (P2 escape + P4b find-route fallback)
�?  �?  ├── HungarianAlgorithm.java        # O(n³) assignment
�?  �?  ├── HungarianBoxAssigner.java      # Box→goal optimal matching wrapper
�?  �?  ├── ImmovableBoxDetector.java      # Distance with immovable boxes treated as walls
�?  �?  ├── ConflictResolver.java          # Joint-action conflict resolution
�?  �?  ├── DeadlockBreaker.java           # 2-cycle / oscillation detection
�?  �?  ├── AgentCoordinator.java          # Yielding / clearing / parking
�?  �?  ├── PathAnalyzer.java              # Corridor / parking / path classification
�?  �?  ├── ArticulationPointFinder.java   # Cut-vertex detection for chokepoints
�?  �?  ├── GoalChecker.java               # Per-agent goal-completion bookkeeping
�?  �?  ├── GreedyPlanner.java             # Helper: nearest-box greedy fallback
�?  �?  ├── PlanMerger.java                # Merge per-group plans into joint plan
�?  �?  └── PlanningUtils.java
�?  ├── synthesis/               # L3 �?Synthetic subgoal injection
�?  �?  ├── EscapeSubgoalSynthesizer.java   # P_temp escape goals to break 2-cycles
�?  �?  └── BlockerReliefSynthesizer.java   # NAMO cross-color blocker relief
�?  ├── coordination/            # L4 �?Conflict/deadlock detection
�?  �?  ├── ConflictDetector.java
�?  �?  └── DeadlockResolver.java
�?  ├── goal/
�?  �?  ├── Goal.java
�?  �?  └── GoalExtractor.java
�?  ├── signal/
�?  �?  └── FailureReport.java   # Structured failure signal (Kind + lastSubgoal + blocked)
�?  �?                           #   Consumed by CBSR for nogood derivation
�?  └── diag/
�?      └── FailureSnapshot.java # Markdown failure snapshot �?target/diagnostics/
```

> **Removed since previous revision**: `cbs/` package, `JointAStarStrategy`,
> `SimplePriorityStrategy`, `StrategySelector`. CBSStrategy is no longer present �?
> CBS-style **conflict→constraint** logic now lives in PortfolioController's
> CBSR loop at the **subgoal-ordering** level (per `claudeopus47.txt §3.2.2`).

## Strategy Selection Flow

```
Client.searchWithFallback()
    �?PortfolioController.search()
        ├─ Step 1:  LevelAnalyzer.analyze() �?LevelFeatures
        �?          ImmovableFusion.applyAndUpdateFeatures() (project immovable boxes as walls)
        �?
        ├─ Step 1b: Independence detection (if numAgents > 1)
        �?          �?detectIndependentGroups()             �?BFS + union-find on physical reachability
        �?          �?refineGroupsByFootprint()             �?split when box/goal footprints disjoint
        �?          �?mergeGroupsByGoalDependencies()       �?merge when logical (P3) deps cross groups
        �?          �?project Level/State per group, solve in parallel time budget, merge plans
        �?          �?if ALL groups solved �?return; otherwise keep best partial, fall through
        �?
        ├─ Step 2:  Warm-start sequence (buildStrategySequence)
        �?          Always 2 attempts of STRICT_ORDER/PP differing only in OrderingMode + budget
        �?          SINGLE_AGENT (�? active goals) �?[A*(w=1.0, 40%), A*(w=5.0, 60%)]
        �?          SINGLE_AGENT (> 8 active goals) �?fall back to PP (DIST_GREEDY 40%, TOPO 35%)
        �?          Cyclic + corridorRatio>0.5     �?[DISTANCE_FARTHEST 40%, DISTANCE_GREEDY 30%]
        �?          Cyclic + corridorRatio�?.5     �?[TOPOLOGICAL 35%,    DISTANCE_GREEDY 35%]
        �?          No cycles                      �?[TOPOLOGICAL 40%,    DISTANCE_GREEDY 30%]
        �?
        ├─ Step 3:  CBSR �?Conflict-Based Subgoal Reordering (�? iterations)
        �?          Collects FailureReport from every PP attempt above.
        �?          learnNogoodsFromReport() �?OrderingNogood (e.g. "goal A must precede goal B").
        �?          Each iteration:
        �?             build a TOPOLOGICAL order respecting dedup'd nogoods,
        �?             run PP with that order, harvest new nogoods on failure.
        �?          When ordering stabilises �?switch to seeded RANDOM diversification.
        �?
        ├─ Step 4:  F7-PRE �?Unsatisfied-First fresh attempt
        �?          Re-orders so unsatisfied goals run first; tries TOPO seed=0 + RANDOM seed=9001.
        �?
        └─ Step 5:  F7B �?Independent warm-start from CBSR base with seeds
                    {42, 137, 9999, 12345, 7777, 55555}; per-iter budget capped at 30s.
```

**Top-level strategy menu (today)**: only `STRICT_ORDER` (= `PriorityPlanningStrategy`
with different `OrderingMode`s) and `SINGLE_AGENT`. `JointAStar` and a true
`CBSStrategy` are intentionally excluded from the portfolio. Rationale captured
in `docs/adr/0001-portfolio-pp-only.md` (per `deepseek-bug.txt` peer review):

- Vanilla CBS assumes point agents + static environment; both fail in push/pull
  Sokoban (`claudeopus47.txt §3.1.2`). A faithful Sokoban-CBS port is PhD-scale.
- `JointAStar` branching is `5^n` agents �?unusable beyond 3 agents.
- CBS *spirit* (conflict �?constraint �?re-search) is preserved as **CBSR at the
  subgoal layer** (cheaper search space, ≤O(numGoals) constraints vs. ≤O(actions)).

`StrategyConfig` schema (PortfolioController.java private class): `(type, weight, orderingMode, randomSeed, timeBudgetFraction)`.

`OrderingMode` (PriorityPlanningStrategy):
`TOPOLOGICAL` · `REVERSE_TOPOLOGICAL` · `DISTANCE_GREEDY` · `DISTANCE_FARTHEST` · `RANDOM`.

## Key Algorithms

### LevelAnalyzer (Pre-planning Analysis)

**Reachability-based dependency analysis** (`DependencyAnalyzer`):
- For each goal pair (Gi, Gj): hypothetically block Gi, check if box can still
  be pushed into Gj via BFS reachability to adjacent push positions.
- If blocking Gi makes Gj unreachable �?Gi depends on Gj (fill Gj first).
- Produces a DAG �?topological sort �?`features.executionOrder`.

**Immovable boxes** (`TaskFilter` + `ImmovableFusion`): Boxes that are already on
their goal AND cannot be moved without violating the goal are detected, then
**fused into walls** so downstream BFS/A* never even considers them as obstacles.

**Cross-color barriers** (`CrossColorBarrierAnalyzer`): wrong-color boxes lying on
an agent's required corridor �?flagged for `BlockerReliefSynthesizer`.

**Coupling degree**: 0.0 (independent) to 1.0 (fully coupled). Weighted: 40%
dependency ratio + 30% dependency intensity + 30% bottleneck ratio. Drives
`StrategyType` recommendation in `LevelFeatures`.

### CBSR (Conflict-Based Subgoal Reordering) �?replaces classical CBS

**Principle** (`claudeopus47.txt §3.2.2`): instead of CBS's *action-level*
constraint splitting (which is impractical in push/pull Sokoban), apply CBS's
**conflict→constraint→re-search** loop at the **subgoal-ordering layer**.

**Loop** (`PortfolioController.java`, MAX_CBSR_ITER=8):
1. Run PP with current ordering. On failure, harvest a `FailureReport`
   (`STUCK_NO_PROGRESS` / `PARTIAL_PLAN` / `NO_PLAN`) carrying `lastAttemptedSubgoal`
   + `unsatisfiedAtFailure` + `blockedGoals`.
2. `learnNogoodsFromReport()` derives `OrderingNogood`s
   (e.g. "goal A must NOT be scheduled before goal B").
3. Build a new TOPOLOGICAL order respecting deduplicated nogoods, re-run PP.
4. When ordering stabilises (same order for 2 iters) �?switch to seeded RANDOM
   diversification with seeds `{42, 137, 9999, 12345}`.

Followed by **F7-PRE** (unsatisfied-first ordering) and **F7B** (warm-start from
the best CBSR result with seeds `{42, 137, 9999, 12345, 7777, 55555}`).

### PriorityPlanningStrategy (Main Multi-Agent Solver)

**Subgoal decomposition**: Uses LevelAnalyzer's topological order, then dispatches
one box-to-goal at a time using `BoxSearchPlanner` (A*).

**Hungarian box-goal assignment** (`HungarianBoxAssigner`): pure box→goal distance
cost, no execution-order assumptions. Cache invalidated whenever a completed goal
is disturbed (`subgoalManager.invalidateHungarianCache()` on regress paths).
`planSubgoal()` falls back to greedy Layer-2 candidates when Hungarian's pick
fails all BSP rounds.

**Ordering modes** (each is one portfolio attempt):
- `TOPOLOGICAL` �?dependency-respecting
- `REVERSE_TOPOLOGICAL` �?used by CBSR when forward fails
- `DISTANCE_GREEDY` �?nearest-box-first
- `DISTANCE_FARTHEST` �?hardest-first (spiral/maze)
- `RANDOM` �?seeded shuffle (CBSR diversification)

**Multi-round BSP per subgoal** (`planSubgoal` ~L3600+):
- Round 1: All `completedBoxGoals` frozen �?ST-A* then 2D A*
- Round 2: Self-blockers unlocked (targeted) �?ST-A* then 2D A*
- Round 3: No frozen at all (desperate) �?2D A*
- Round 4: Weighted A* (w=3.0)
- **P4b**: IW(1) FIND-ROUTE FALLBACK for non-escape subgoals where
  `IW1_FINDROUTE_MIN_MANHATTAN �?d �?IW1_FINDROUTE_MAX_MANHATTAN` (10�?5).
  See `PriorityPlanningStrategy.java` ~L3700.

**REGRESS tolerance**: �? box-goal regression with current subgoal succeeded �?
remove from `completedBoxGoals` for re-planning. �? regressions or agent-goal
regression �?full rollback + REGRESS-UNPROTECT.

**Conflict handling stack**:
1. `ConflictResolver` �?joint action validation; yielder picked by holding-box /
   on-goal priority (NOT raw agent ID �?fixed per `deepseek-bug.txt #1`).
2. `ConflictDetector` (coordination/) �?vertex/edge conflict detection.
3. `AgentCoordinator` �?yielding + clearing + parking.
4. `DeadlockBreaker` �?2-cycle / oscillation detection.
5. Synthesizers �?`EscapeSubgoalSynthesizer`, `BlockerReliefSynthesizer` inject
   helper subgoals on stuck.

### Independence Detection (PortfolioController)

```
detectIndependentGroups   �?BFS + union-find on physical reachability components
refineGroupsByFootprint   �?split when box/goal/transit footprints disjoint (P1)
mergeGroupsByGoalDeps     �?merge when features.goalDependsOn crosses groups (P3)
                            but NEVER collapse to 1 (preserves warm-up + fallback)
```

Per group: project Level (mask out other-color boxes/goals) and State, run
PortfolioController recursively with proportional time budget, then `PlanMerger`
stitches per-group plans back into a joint plan.

## State Representation

States are **immutable**. Key design:
```java
class State {
    Position[] agentPositions;              // indexed by agent id
    Map<Position, Character> boxes;          // (row,col) �?box letter
    int hashCode;                            // cached after first computation
}
```

Successors generated via `state.getSuccessors(agentId, level)` which returns
all applicable (Action, State) pairs respecting walls, box colors, and physics.
`state.applyJointAction(actions, level)` mirrors the server: single-agent
inapplicability becomes NoOp, conflicts NoOp all involved agents, and all effects
are applied atomically from the start-of-timestep state.

## Heuristics

| Heuristic | Method | Admissible | When to use |
|-----------|--------|------------|-------------|
| Manhattan | Σ(box→nearest goal) + Σ(agent→goal) | Yes | Fast baseline |
| BFS True Distance | Pre-computed BFS from each goal | Yes | Default. Respects walls |

Both **ignore box-blocking** (underestimate in congested levels).
`h_conflicts` partially implemented: +2 penalty per wrong-type box occupying a goal.

## Common Patterns

### Adding a new strategy
1. Implement `SearchStrategy` interface: `search(State, Level) �?List<Action[]>`
2. Add enum value to `LevelAnalyzer.StrategyType` (current: `SINGLE_AGENT`, `STRICT_ORDER`, `GREEDY_WITH_RETRY`, `CYCLE_BREAKER`).
3. Add case in `PortfolioController.buildStrategySequence()` �?append a `StrategyConfig` (type, weight, orderingMode, randomSeed, timeBudgetFraction).
4. Add case in `PortfolioController.createStrategy()` for instantiation.
5. Set timeout/maxStates via `setTimeout()` / `setMaxStates()`.
6. If the strategy emits failure information consumable by CBSR, return a populated `FailureReport` so nogoods can be derived.

### Testing a level
```bash
# Single-agent level
java -jar server.jar -l levels/SAbotbot.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 500 -t 60

# Multi-agent level
java -jar server.jar -l levels/MAsimple1.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 100 -t 30

# Competition level
java -jar server.jar -l complevels/DECrunchy.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -s 500 -t 180
```

## Current Implementation State

### Portfolio and Ordering

- `PortfolioController` is the default top-level planner.
- The active portfolio uses `STRICT_ORDER` / `PriorityPlanningStrategy` with different `OrderingMode`s plus `SINGLE_AGENT` for small single-agent cases.
- Classical CBS and top-level `JointAStarStrategy` are not part of the active portfolio. CBS-style reasoning is implemented as CBSR at the subgoal-ordering layer: collect `FailureReport`, learn ordering nogoods, rebuild a goal order, and re-run PP.
- The current L3 repair phases are: initial ordering probe, L3 CBSR ordering repair, L3 residual-order repair, L3 partial-plan continuation, and L3 CBSR-base continuation.
- `MAVIS_TIMEOUT_MS` can be used to pass the intended server timeout to the client process. Example: set `MAVIS_TIMEOUT_MS=360000` when server `-t 360` is used.

### Priority Planning and Box Search

- `PriorityPlanningStrategy` decomposes the level into box/agent subgoals and executes them in a fixed ordering for each PP run.
- `SubgoalManager` handles ordering and box-goal selection, with Hungarian assignment enabled for box-goal matching. Hungarian selection and dependency ordering are treated as separate concerns.
- `BoxSearchPlanner` performs single-box push/pull search. Current displacement planning tracks all boxes movable by the acting agent for blocker-clearing searches, so same-color helper boxes are represented in the search key.
- BSP has multi-round fallback behavior: frozen completed goals, targeted unfreezing, unfrozen fallback, weighted A*, and selected IW(1) fallback for route-finding.
- `completedBoxGoals` are seeded from the input state when PP starts from a continuation state, so partial-plan continuation preserves already completed goals.

### NAMO and Synthetic Relief

- `BlockerReliefSynthesizer` produces synthetic relief subgoals with `ReliefCertificate`s. A synthetic relief is accepted only when its certificate is resolved.
- `PriorityPlanningStrategy` also has task-conditioned blocker relief inside `tryExecuteSubgoals()`. When a real box subgoal cannot be planned, it identifies current access blockers for that task and attempts local NAMO-style clearing.
- Task-conditioned relief treats relief as a causal subtask: partial relief is rolled back if blockers for the current task remain after the relief attempt.
- `BoxSearchPlanner.planBoxReleaseFromForbidden()` supports NAMO-style release: the blocker does not need to reach one exact parking cell; success means the target box leaves the forbidden resource cells while respecting frozen goals and protected cells.
- Synthetic parking validation checks whether a newly parked synthetic box becomes a blocker for a remaining real subgoal. Such a synthetic placement is rejected and rolled back.
- Task-relief diagnostics log blocker release attempts, fixed parking attempts, candidate counts, BSP failures, validation failures, and skipped nogoods.

### Diagnostics and Replay

- Replay JSON files are written under `target/diagnostics/replays/` with timestamped names such as `ISO__2026-05-10T07-45-25__partial__1782-steps.json`.
- The reusable replay viewer lives under `target/diagnostics/replay-viewer/`. It reads replay data and allows step-by-step inspection of agents, boxes, and accepted actions.
- `FailureSnapshot` writes a structured Markdown snapshot to `target/diagnostics/ISO-failure.md` for the latest ISO run, including phase timing, attempt timeline, final unsatisfied goals, and local blocker views.
- The level/parking visualization tools live under `tools/level-indexer/` and `target/diagnostics/parking-risk/`.

### Current ISO Run Context

- Recent ISO runs are being debugged with: `java -jar server.jar -l complevels/ISO.lvl -c "java -Xmx4g -cp target/classes mapf.client.Client" -g -t 360` plus `MAVIS_TIMEOUT_MS=360000`.
- The latest inspected replay before this note was `target/diagnostics/replays/ISO__2026-05-10T07-45-25__partial__1782-steps.json`.
- In that replay, an early synthetic movement placed an `L` box at `(2,2)` at timestep `465`. Later frames show `L@(2,2)` and the two `G` boxes in the upper-left region remaining in place while agent 8 works on later `L` goals.
- After the latest code changes, synthetic relief validation should reject synthetic placements that block a remaining real task. Look for `[PP][SYNTHETIC-RELIEF-REJECT] ... reason=blocks-future-task` in new logs.

### Active Architectural Decisions

- Hungarian algorithm for box-goal assignment is enabled with box-retry fallback. Hungarian decides which box fills which goal; dependency analysis decides the order in which goals are attempted.
- Classical CBS remains excluded from the top-level portfolio. The project uses CBSR at the subgoal layer as the cheaper conflict-to-constraint loop.
- `JointAStar` remains under `pathfinding/` for internal use only and is not a top-level portfolio strategy.

## File Conventions
- **Level files**: `levels/` (warmup), `complevels/` (competition)
- **Level format**: `.lvl` files with `#domain`, `#colors`, `#initial`, `#goal`, `#end`
- **Reference docs**: `PRODUCT.md` (domain rules), `ARCHITECTURE.md` (design guidance)
- **All States immutable** �?never mutate a State after creation
- **Logging**: `System.err.println()` �?stderr only (stdout is server protocol)

## Performance Notes

- `TrueDistanceHeuristic` precomputation: O(Goals × Rows × Cols). Cached at Level load.
- State hashing is pre-computed in constructor. HashMap closed-list is O(1) amortized.
- `JointAStar` (now in `pathfinding/`, not in portfolio) branching is `5^n`.
- CBSR high-level: �? iterations × one PP run each. Nogoods deduplicated.
- PriorityPlanning: linear in goal count, but each subgoal is a multi-round BSP A*.
- Independence-detection groups run with **proportional** time budget (group_weight
  × planning_budget) and are joined by `PlanMerger` (NoOp-pad to longest plan).
- Memory: `-Xmx4g` recommended. Large levels can generate millions of states.
- Wall-clock budget: planning gets `serverTimeout - 10s` (10s reserved for partial-plan
  flush + serialization). Override via `MAVIS_TIMEOUT_MS` env var when running outside
  the official server.
