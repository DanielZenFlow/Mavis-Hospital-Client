package mapf.planning.synthesis;

import mapf.domain.*;
import mapf.planning.strategy.PriorityPlanningStrategy.Subgoal;

import java.util.*;

/**
 * Generic NAMO (Navigation Among Movable Obstacles) reifier.
 *
 * <p>Companion to {@link EscapeSubgoalSynthesizer} (which handles 2-cycles in the
 * goal-dependency graph). This synthesizer handles the orthogonal case:
 *
 * <pre>
 *   Agent A (color C_A) must push box B (color C_A) to goal G,
 *   but A cannot geometrically reach adj(B) — the path is blocked by one or
 *   more boxes B_x of color C_x ≠ C_A. A cannot move B_x itself (color rule).
 *   A helper agent of color C_x must move B_x out of the way first.
 * </pre>
 *
 * <p>The contribution of this class is to <b>reify</b> the helper action as a
 * regular {@link Subgoal} of the form {@code (helper, blockerType, P_temp)}
 * which is then prepended to the SubgoalManager queue via the existing
 * {@code escapeSubgoals} pipeline. No new Subgoal type, no new planner — the
 * existing PP/BSP infrastructure consumes it like any other box-push task.
 *
 * <p>Literature mapping: this is the multi-agent push-pull formulation of
 * <i>Navigation Among Movable Obstacles</i> (Stilman & Kuffner 2005), also
 * called <i>Helper-Agent Planning</i> / <i>Cooperative Box Clearing</i> in the
 * MAPF + manipulation literature. The reification step is the classical-AI
 * <i>Subgoal Reification</i> from HTN planning.
 *
 * <p>Conservative bounds: emits at most {@link #MAX_RELIEF_SUBGOALS} per call,
 * BFS expansion is capped, and a blocker is skipped if no same-color helper
 * agent exists or if no safe P_temp can be found. Safe to call on any level —
 * returns empty list when no blocked-agent pattern is detected.
 */
public final class BlockerReliefSynthesizer {

    /** Maximum relief subgoals to emit per invocation. */
    private static final int MAX_RELIEF_SUBGOALS = 6;

    /** BFS expansion cap for reachability searches. */
    private static final int REACH_BFS_CAP = 4000;

    /** BFS expansion cap for P_temp search per blocker. */
    private static final int PTEMP_BFS_CAP = 600;

    private BlockerReliefSynthesizer() {}

    /**
     * Detect blocked-agent patterns and emit relief subgoals.
     *
     * @param state          current world state
     * @param level          static level data
     * @param immovableBoxes treated as walls (from TaskFilter); pass empty if unknown
     * @return list of relief Subgoals (possibly empty); to be prepended to PP order
     */
    public static List<Subgoal> synthesize(State state, Level level,
                                           Set<Position> immovableBoxes) {
        if (immovableBoxes == null) immovableBoxes = Collections.emptySet();

        Set<Position> emittedTemps = new HashSet<>();
        Set<Position> handledBlockerPositions = new HashSet<>();
        List<Subgoal> reliefs = new ArrayList<>();

        // Pre-compute set of all goal cells (to forbid as P_temp targets).
        Set<Position> allGoalCells = new HashSet<>();
        for (List<Position> gs : level.getBoxGoalsByType().values()) allGoalCells.addAll(gs);
        for (Position ag : level.getAgentGoalPositionMap().values()) allGoalCells.add(ag);

        int numAgents = state.getNumAgents();
        Map<Position, Character> boxes = state.getBoxes();

        // Index boxes by color for fast helper assignment.
        Map<Color, List<Integer>> agentsByColor = new EnumMap<>(Color.class);
        for (int i = 0; i < numAgents; i++) {
            agentsByColor.computeIfAbsent(level.getAgentColor(i), k -> new ArrayList<>()).add(i);
        }

        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (reliefs.size() >= MAX_RELIEF_SUBGOALS) break;

            Color agentColor = level.getAgentColor(agentId);
            Position agentPos = state.getAgentPosition(agentId);

            // Find this agent's unsatisfied same-color box-goals.
            // For each, check whether ANY adj(box) ∪ adj(goal) is reachable in R_actual;
            // if not, identify blockers via R_through path enumeration.
            Set<Position> rActual = bfsReachable(agentPos, state, level, immovableBoxes,
                    /*treatBoxesAsWalls*/ true, agentColor);
            Set<Position> rThrough = null; // computed lazily

            for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
                if (reliefs.size() >= MAX_RELIEF_SUBGOALS) break;
                char goalType = entry.getKey();
                Color boxColor = level.getBoxColor(goalType);
                if (boxColor != agentColor) continue;

                for (Position goalPos : entry.getValue()) {
                    if (reliefs.size() >= MAX_RELIEF_SUBGOALS) break;

                    // Skip already-satisfied goal.
                    Character boxOnGoal = boxes.get(goalPos);
                    if (boxOnGoal != null && boxOnGoal == goalType) continue;

                    // Find the box that should fill this goal.
                    Position boxPos = findNearestSameTypeBox(goalType, goalPos, state, level);
                    if (boxPos == null) continue;
                    if (boxPos.equals(goalPos)) continue;

                    // Reachability check: agent must reach adj(box) to push it.
                    if (anyAdjacentInSet(boxPos, rActual, level)) continue;
                    if (anyAdjacentInSet(goalPos, rActual, level)) continue;

                    // Confirm the obstruction is BOX-shaped (not a wall problem).
                    if (rThrough == null) {
                        rThrough = bfsReachable(agentPos, state, level, immovableBoxes,
                                /*treatBoxesAsWalls*/ false, agentColor);
                    }
                    if (!anyAdjacentInSet(boxPos, rThrough, level)
                            && !anyAdjacentInSet(goalPos, rThrough, level)) {
                        continue; // not reachable even through boxes — wall problem, give up
                    }

                    // Identify blockers: BFS path from agent to adj(boxPos) in rThrough,
                    // collecting other-color boxes encountered on the path.
                    List<Position> blockerPath = findBlockerPath(agentPos, boxPos, state, level,
                            immovableBoxes, agentColor);
                    if (blockerPath == null || blockerPath.isEmpty()) continue;

                    for (Position blockerPos : blockerPath) {
                        if (reliefs.size() >= MAX_RELIEF_SUBGOALS) break;
                        if (handledBlockerPositions.contains(blockerPos)) continue;

                        Character blockerType = boxes.get(blockerPos);
                        if (blockerType == null) continue;
                        Color blockerColor = level.getBoxColor(blockerType);
                        if (blockerColor == null || blockerColor == agentColor) continue;

                        List<Integer> helpers = agentsByColor.get(blockerColor);
                        if (helpers == null || helpers.isEmpty()) continue;

                        // Pick helper closest to blocker (Manhattan; cheap heuristic).
                        int bestHelper = -1;
                        int bestDist = Integer.MAX_VALUE;
                        for (int h : helpers) {
                            int d = state.getAgentPosition(h).manhattanDistance(blockerPos);
                            if (d < bestDist) { bestDist = d; bestHelper = h; }
                        }
                        if (bestHelper < 0) continue;

                        Position pTemp = findPTempBFS(blockerPos, state, level, immovableBoxes,
                                allGoalCells, emittedTemps);
                        if (pTemp == null) continue;

                        reliefs.add(new Subgoal(bestHelper, blockerType, pTemp, false));
                        emittedTemps.add(pTemp);
                        handledBlockerPositions.add(blockerPos);
                    }
                }
            }
        }
        return reliefs;
    }

    // ---------------------------------------------------------------------------
    // BFS helpers
    // ---------------------------------------------------------------------------

    /**
     * BFS from {@code start}. Walls always block. Other agents are NOT blockers
     * (we plan in single-agent space; cross-agent conflicts are resolved later).
     * If {@code treatBoxesAsWalls=true}, all boxes block. Otherwise boxes are
     * walkable (used to assess theoretical reachability for blocker detection).
     */
    private static Set<Position> bfsReachable(Position start, State state, Level level,
                                              Set<Position> immovableBoxes,
                                              boolean treatBoxesAsWalls,
                                              Color agentColor) {
        Map<Position, Character> boxes = state.getBoxes();
        int rows = level.getRows();
        int cols = level.getCols();
        Set<Position> visited = new HashSet<>();
        Deque<Position> queue = new ArrayDeque<>();
        queue.add(start);
        visited.add(start);
        int expansions = 0;
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};

        while (!queue.isEmpty() && expansions < REACH_BFS_CAP) {
            Position p = queue.poll();
            expansions++;
            for (int[] d : dirs) {
                int nr = p.row + d[0];
                int nc = p.col + d[1];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                if (level.isWall(nr, nc)) continue;
                Position np = new Position(nr, nc);
                if (visited.contains(np)) continue;
                if (immovableBoxes.contains(np)) continue;
                if (treatBoxesAsWalls) {
                    Character bx = boxes.get(np);
                    if (bx != null) {
                        // Same-color box: agent can push, so the cell is "reachable
                        // for path planning" purposes only if we let it through.
                        // For blocker detection (rActual) we keep it strict: same-color
                        // box is also a wall, since the box itself occupies the cell
                        // and the agent cannot stand on it.
                        continue;
                    }
                }
                visited.add(np);
                queue.add(np);
            }
        }
        return visited;
    }

    private static boolean anyAdjacentInSet(Position p, Set<Position> set, Level level) {
        int rows = level.getRows();
        int cols = level.getCols();
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};
        for (int[] d : dirs) {
            int nr = p.row + d[0];
            int nc = p.col + d[1];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (set.contains(new Position(nr, nc))) return true;
        }
        return false;
    }

    /**
     * BFS from agent to a cell adjacent to {@code targetBox} in box-permeable space,
     * then walk back the parent chain to enumerate boxes encountered.
     * Returns the list of blocker box positions (excluding target box itself).
     * Empty list means no path or no blockers (caller should treat as no-op).
     */
    private static List<Position> findBlockerPath(Position agentPos, Position targetBox,
                                                  State state, Level level,
                                                  Set<Position> immovableBoxes,
                                                  Color agentColor) {
        Map<Position, Character> boxes = state.getBoxes();
        int rows = level.getRows();
        int cols = level.getCols();

        Map<Position, Position> parent = new HashMap<>();
        Set<Position> visited = new HashSet<>();
        Deque<Position> queue = new ArrayDeque<>();
        queue.add(agentPos);
        visited.add(agentPos);
        int expansions = 0;
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};

        Position foundAdj = null;
        while (!queue.isEmpty() && expansions < REACH_BFS_CAP) {
            Position p = queue.poll();
            expansions++;
            // Goal: any cell adjacent to the target box.
            if (Math.abs(p.row - targetBox.row) + Math.abs(p.col - targetBox.col) == 1) {
                foundAdj = p;
                break;
            }
            for (int[] d : dirs) {
                int nr = p.row + d[0];
                int nc = p.col + d[1];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                if (level.isWall(nr, nc)) continue;
                Position np = new Position(nr, nc);
                if (visited.contains(np)) continue;
                if (immovableBoxes.contains(np)) continue;
                // Allow walking THROUGH boxes (for blocker enumeration).
                visited.add(np);
                parent.put(np, p);
                queue.add(np);
            }
        }

        if (foundAdj == null) return Collections.emptyList();

        // Walk back, collect cells that contain other-color boxes.
        List<Position> blockers = new ArrayList<>();
        Position cur = foundAdj;
        while (cur != null && !cur.equals(agentPos)) {
            Character bx = boxes.get(cur);
            if (bx != null && !cur.equals(targetBox)) {
                Color bc = level.getBoxColor(bx);
                if (bc != null && bc != agentColor) {
                    blockers.add(cur);
                }
            }
            cur = parent.get(cur);
        }
        return blockers;
    }

    /**
     * Find a same-type box position to fill {@code goalPos}. Picks the one
     * nearest to {@code goalPos} that is not already on a different goal of the
     * same type. Returns {@code null} if none found.
     */
    private static Position findNearestSameTypeBox(char boxType, Position goalPos,
                                                   State state, Level level) {
        Set<Position> sameTypeGoals = new HashSet<>(level.getBoxGoalsByType()
                .getOrDefault(boxType, Collections.emptyList()));
        Position best = null;
        int bestDist = Integer.MAX_VALUE;
        for (Map.Entry<Position, Character> e : state.getBoxes().entrySet()) {
            if (e.getValue() != boxType) continue;
            Position bp = e.getKey();
            // Skip boxes already sitting on a same-type goal (they're "done" for that goal).
            if (sameTypeGoals.contains(bp) && !bp.equals(goalPos)) continue;
            int d = bp.manhattanDistance(goalPos);
            if (d < bestDist) { bestDist = d; best = bp; }
        }
        return best;
    }

    /**
     * BFS from box position to find a free, non-goal, non-corner cell to park at.
     * Mirrors {@link EscapeSubgoalSynthesizer}'s findPTempBFS logic but takes
     * immovableBoxes as input.
     */
    private static Position findPTempBFS(Position boxPos, State state, Level level,
                                         Set<Position> immovableBoxes,
                                         Set<Position> goalCells,
                                         Set<Position> usedTemps) {
        Map<Position, Character> boxes = state.getBoxes();
        int rows = level.getRows();
        int cols = level.getCols();
        Set<Position> visited = new HashSet<>();
        Deque<Position> queue = new ArrayDeque<>();
        queue.add(boxPos);
        visited.add(boxPos);
        int expansions = 0;
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};

        while (!queue.isEmpty() && expansions < PTEMP_BFS_CAP) {
            Position p = queue.poll();
            expansions++;

            if (!p.equals(boxPos)
                    && !goalCells.contains(p)
                    && !usedTemps.contains(p)
                    && !boxes.containsKey(p)
                    && !immovableBoxes.contains(p)
                    && !isCorner(p, level, immovableBoxes)) {
                return p;
            }

            for (int[] d : dirs) {
                int nr = p.row + d[0];
                int nc = p.col + d[1];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                if (level.isWall(nr, nc)) continue;
                Position np = new Position(nr, nc);
                if (visited.contains(np)) continue;
                if (immovableBoxes.contains(np)) continue;
                visited.add(np);
                queue.add(np);
            }
        }
        return null;
    }

    private static boolean isCorner(Position p, Level level, Set<Position> immovableBoxes) {
        boolean wallN = isBlocked(p.row - 1, p.col, level, immovableBoxes);
        boolean wallS = isBlocked(p.row + 1, p.col, level, immovableBoxes);
        boolean wallW = isBlocked(p.row, p.col - 1, level, immovableBoxes);
        boolean wallE = isBlocked(p.row, p.col + 1, level, immovableBoxes);
        return (wallN && wallW) || (wallN && wallE) || (wallS && wallW) || (wallS && wallE);
    }

    private static boolean isBlocked(int r, int c, Level level, Set<Position> immovableBoxes) {
        if (r < 0 || r >= level.getRows() || c < 0 || c >= level.getCols()) return true;
        if (level.isWall(r, c)) return true;
        return immovableBoxes.contains(new Position(r, c));
    }
}
