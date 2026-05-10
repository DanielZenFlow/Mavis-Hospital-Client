package mapf.planning.synthesis;

import mapf.domain.*;
import mapf.planning.strategy.ArticulationPointFinder;
import mapf.planning.strategy.PriorityPlanningStrategy.Subgoal;
import mapf.planning.strategy.PriorityPlanningStrategy.ReliefCertificate;

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

    /** Maximum relief subgoals to emit per invocation (incl. recursive sub-reliefs). */
    private static final int MAX_RELIEF_SUBGOALS = 20;

    /** Maximum recursion depth for cascade NAMO (helper-of-helper-of...). */
    private static final int MAX_RECURSION_DEPTH = 3;

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
        return synthesizeWithMeta(state, level, immovableBoxes).reliefs;
    }

    /**
     * Result of {@link #synthesizeWithMeta} carrying both the relief subgoal list
     * and the F2 suspension set (original goals of relocated blockers).
     */
    public static final class ReliefResult {
        public final List<Subgoal> reliefs;
        public final Set<Position> suspendedBoxGoals;
        ReliefResult(List<Subgoal> reliefs, Set<Position> suspended) {
            this.reliefs = reliefs;
            this.suspendedBoxGoals = suspended;
        }
    }

    /**
     * Like {@link #synthesize} but additionally returns the set of original box-goals
     * to suspend (F2). For each emitted relief, the blocker box's closest same-type
     * unsatisfied goal is added to the suspension set so the PP planner won't try to
     * push the blocker from P_temp back to that goal during this attempt.
     */
    public static ReliefResult synthesizeWithMeta(State state, Level level,
                                                  Set<Position> immovableBoxes) {
        if (immovableBoxes == null) immovableBoxes = Collections.emptySet();

        Set<Position> emittedTemps = new HashSet<>();
        Set<Position> handledBlockerPositions = new HashSet<>();
        // Tree nodes: each carries the emitted Subgoal, the original blocker position
        // (so we can later re-check helper reachability), and a recursion depth so we
        // can output deepest-first (sub-reliefs must execute before their parents).
        List<ReliefNode> nodes = new ArrayList<>();
        Deque<ReliefNode> toExpand = new ArrayDeque<>();

        // Pre-compute set of all goal cells (to forbid as P_temp targets).
        Set<Position> allGoalCells = new HashSet<>();
        for (List<Position> gs : level.getBoxGoalsByType().values()) allGoalCells.addAll(gs);
        for (Position ag : level.getAgentGoalPositionMap().values()) allGoalCells.add(ag);
        Set<Position> staticNoParkingCells =
                ArticulationPointFinder.findArticulationPoints(level, immovableBoxes);
        Set<Position> dynamicRiskParkingCells =
                ArticulationPointFinder.findArticulationPoints(level, new HashSet<>(state.getBoxes().keySet()));

        int numAgents = state.getNumAgents();
        Map<Position, Character> boxes = state.getBoxes();

        // Index boxes by color for fast helper assignment.
        Map<Color, List<Integer>> agentsByColor = new EnumMap<>(Color.class);
        for (int i = 0; i < numAgents; i++) {
            agentsByColor.computeIfAbsent(level.getAgentColor(i), k -> new ArrayList<>()).add(i);
        }

        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (nodes.size() >= MAX_RELIEF_SUBGOALS) break;

            Color agentColor = level.getAgentColor(agentId);
            Position agentPos = state.getAgentPosition(agentId);

            // Find this agent's unsatisfied same-color box-goals.
            // For each, check whether ANY adj(box) ∪ adj(goal) is reachable in R_actual;
            // if not, identify blockers via R_through path enumeration.
            Set<Position> rActual = bfsReachable(agentPos, state, level, immovableBoxes,
                    /*treatBoxesAsWalls*/ true, agentColor);
            Set<Position> rThrough = null; // computed lazily

            for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
                if (nodes.size() >= MAX_RELIEF_SUBGOALS) break;
                char goalType = entry.getKey();
                Color boxColor = level.getBoxColor(goalType);
                if (boxColor != agentColor) continue;

                for (Position goalPos : entry.getValue()) {
                    if (nodes.size() >= MAX_RELIEF_SUBGOALS) break;

                    // Skip already-satisfied goal.
                    Character boxOnGoal = boxes.get(goalPos);
                    if (boxOnGoal != null && boxOnGoal == goalType) continue;

                    // Find the box that should fill this goal.
                    Position boxPos = findNearestSameTypeBox(goalType, goalPos, state, level);
                    if (boxPos == null) continue;
                    if (boxPos.equals(goalPos)) continue;

                    // Reachability check: agent must reach adj(box) to push it.
                    boolean agentBlocked = !anyAdjacentInSet(boxPos, rActual, level)
                            && !anyAdjacentInSet(goalPos, rActual, level);

                    if (!agentBlocked) {
                        // F5: agent can reach the box, but can the BOX reach the goal?
                        // Check the push-corridor: BFS from boxPos to goalPos in box-permeable
                        // space. If unreachable through actual boxes but reachable through
                        // boxes-as-walls=false, find blockers along that corridor.
                        Set<Position> boxReachActual = bfsReachable(boxPos, state, level,
                                immovableBoxes, /*treatBoxesAsWalls*/ true, agentColor);
                        if (boxReachActual.contains(goalPos)) continue; // corridor clear
                        Set<Position> boxReachThrough = bfsReachable(boxPos, state, level,
                                immovableBoxes, /*treatBoxesAsWalls*/ false, agentColor);
                        if (!boxReachThrough.contains(goalPos)) continue; // wall problem
                        // Find blockers on corridor box -> goal.
                        List<Position> corridorBlockers = findBlockerPathBetween(
                                boxPos, goalPos, state, level, immovableBoxes, agentColor);
                        for (Position blockerPos : corridorBlockers) {
                            if (nodes.size() >= MAX_RELIEF_SUBGOALS) break;
                            // Skip the target box itself.
                            if (blockerPos.equals(boxPos)) continue;
                            ReliefNode n = tryEmitRelief(blockerPos, /*depth*/ 0,
                                    state, level, immovableBoxes, allGoalCells,
                                    staticNoParkingCells, dynamicRiskParkingCells,
                                    ReliefCertificate.boxCorridor(blockerPos, boxPos, goalPos),
                                    agentsByColor, emittedTemps, handledBlockerPositions);
                            if (n != null) {
                                nodes.add(n);
                                toExpand.add(n);
                            }
                        }
                        continue;
                    }

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
                        if (nodes.size() >= MAX_RELIEF_SUBGOALS) break;
                        ReliefNode n = tryEmitRelief(blockerPos, /*depth*/ 0,
                                state, level, immovableBoxes, allGoalCells,
                                staticNoParkingCells, dynamicRiskParkingCells,
                                ReliefCertificate.agentAccess(blockerPos, agentId, boxPos, goalPos),
                                agentsByColor, emittedTemps, handledBlockerPositions);
                        if (n != null) {
                            nodes.add(n);
                            toExpand.add(n);
                        }
                    }
                }
            }
        }

        // ----- D11: Recursive NAMO -----------------------------------------
        // For each emitted relief (helper, blockerType, P_temp), verify the helper
        // can geometrically reach the assigned blocker. If not, find what blocks
        // the helper and synthesize a sub-relief. Sub-reliefs are tagged with a
        // higher depth and will be output BEFORE parent reliefs in the final list.
        while (!toExpand.isEmpty() && nodes.size() < MAX_RELIEF_SUBGOALS) {
            ReliefNode parent = toExpand.poll();
            if (parent.depth >= MAX_RECURSION_DEPTH) continue;

            int helperId = parent.subgoal.agentId;
            Color helperColor = level.getAgentColor(helperId);
            Position helperPos = state.getAgentPosition(helperId);
            Position parentBlocker = parent.blockerPos;

            Set<Position> hReach = bfsReachable(helperPos, state, level, immovableBoxes,
                    /*treatBoxesAsWalls*/ true, helperColor);
            if (anyAdjacentInSet(parentBlocker, hReach, level)) continue; // helper OK

            // Confirm theoretical reachability through boxes, else give up (wall problem).
            Set<Position> hThrough = bfsReachable(helperPos, state, level, immovableBoxes,
                    /*treatBoxesAsWalls*/ false, helperColor);
            if (!anyAdjacentInSet(parentBlocker, hThrough, level)) continue;

            List<Position> subBlockers = findBlockerPath(helperPos, parentBlocker, state, level,
                    immovableBoxes, helperColor);
            for (Position sb : subBlockers) {
                if (nodes.size() >= MAX_RELIEF_SUBGOALS) break;
                ReliefNode child = tryEmitRelief(sb, parent.depth + 1,
                        state, level, immovableBoxes, allGoalCells,
                        staticNoParkingCells, dynamicRiskParkingCells,
                        ReliefCertificate.agentAccess(sb, helperId, parentBlocker, null),
                        agentsByColor, emittedTemps, handledBlockerPositions);
                if (child != null) {
                    nodes.add(child);
                    toExpand.add(child);
                }
            }
        }

        // Output deepest-first so sub-reliefs precede their parent reliefs.
        nodes.sort((a, b) -> Integer.compare(b.depth, a.depth));
        List<Subgoal> reliefs = new ArrayList<>(nodes.size());
        for (ReliefNode n : nodes) reliefs.add(n.subgoal);

        // F2: derive suspended box-goal set from blocker positions. For each emitted
        // relief, find the closest UNSATISFIED same-type goal to the blocker's
        // current position \u2014 that's the goal the planner would naturally try to
        // push the blocker toward, undoing the relief. Suspend it.
        Set<Position> suspended = new HashSet<>();
        Map<Position, Character> stateBoxes = state.getBoxes();
        for (ReliefNode n : nodes) {
            Position blockerPos = n.blockerPos;
            Character blockerType = stateBoxes.get(blockerPos);
            if (blockerType == null) continue;
            List<Position> sameTypeGoals = level.getBoxGoalsByType()
                    .getOrDefault(blockerType, Collections.emptyList());
            Position best = null;
            int bestD = Integer.MAX_VALUE;
            for (Position g : sameTypeGoals) {
                Character occ = stateBoxes.get(g);
                if (occ != null && occ == blockerType) continue; // already satisfied
                int d = Math.abs(g.row - blockerPos.row) + Math.abs(g.col - blockerPos.col);
                if (d < bestD) { bestD = d; best = g; }
            }
            if (best != null) suspended.add(best);
        }
        return new ReliefResult(reliefs, suspended);
    }

    /**
     * Try to emit a single relief Subgoal for {@code blockerPos}.
     * Returns null if blocker is already handled, has no valid helper, or no P_temp.
     */
    private static ReliefNode tryEmitRelief(Position blockerPos, int depth,
                                            State state, Level level,
                                            Set<Position> immovableBoxes,
                                            Set<Position> allGoalCells,
                                            Set<Position> staticNoParkingCells,
                                            Set<Position> dynamicRiskParkingCells,
                                            ReliefCertificate certificate,
                                            Map<Color, List<Integer>> agentsByColor,
                                            Set<Position> emittedTemps,
                                            Set<Position> handledBlockerPositions) {
        if (handledBlockerPositions.contains(blockerPos)) return null;
        Character blockerType = state.getBoxes().get(blockerPos);
        if (blockerType == null) return null;
        Color blockerColor = level.getBoxColor(blockerType);
        if (blockerColor == null) return null;

        List<Integer> helpers = agentsByColor.get(blockerColor);
        if (helpers == null || helpers.isEmpty()) return null;

        // Pick helper closest to blocker (Manhattan; cheap heuristic).
        int bestHelper = -1;
        int bestDist = Integer.MAX_VALUE;
        for (int h : helpers) {
            int d = state.getAgentPosition(h).manhattanDistance(blockerPos);
            if (d < bestDist) { bestDist = d; bestHelper = h; }
        }
        if (bestHelper < 0) return null;

        Position pTemp = findPTempBFS(blockerPos, state, level, immovableBoxes,
                allGoalCells, staticNoParkingCells, dynamicRiskParkingCells, emittedTemps, certificate);
        if (pTemp == null) return null;

        emittedTemps.add(pTemp);
        handledBlockerPositions.add(blockerPos);
        return new ReliefNode(new Subgoal(bestHelper, blockerType, pTemp, false, certificate),
                blockerPos, depth);
    }

    /** Internal tree node carrying recursion depth + original blocker position. */
    private static final class ReliefNode {
        final Subgoal subgoal;
        final Position blockerPos;
        final int depth;
        ReliefNode(Subgoal s, Position b, int d) {
            subgoal = s; blockerPos = b; depth = d;
        }
    }

    // ---------------------------------------------------------------------------
    // D13: NAMO coupling probe (used by PortfolioController for group merging)
    // ---------------------------------------------------------------------------

    /**
     * Probe NAMO (cross-color blocker) coupling without synthesizing subgoals.
     *
     * <p>For each agent A: detect whether A is geometrically blocked from any
     * of its same-color box-goals by other-color boxes, and if so report the
     * COLOR of those blocker boxes. The caller (typically independence
     * detection) can then merge groups whose agents are NAMO-coupled — even
     * when they have no entry in the goal-dependency graph.
     *
     * <p>Returns a map: {@code agentColor -> set of blocker colors}. Only
     * cross-color edges (color != color) are reported. Empty if no blocking.
     */
    public static Map<Color, Set<Color>> probeNAMOCoupling(State state, Level level,
                                                           Set<Position> immovableBoxes) {
        if (immovableBoxes == null) immovableBoxes = Collections.emptySet();
        Map<Color, Set<Color>> coupling = new EnumMap<>(Color.class);
        Map<Position, Character> boxes = state.getBoxes();
        int numAgents = state.getNumAgents();

        for (int agentId = 0; agentId < numAgents; agentId++) {
            Color agentColor = level.getAgentColor(agentId);
            if (agentColor == null) continue;
            Position agentPos = state.getAgentPosition(agentId);
            Set<Position> rActual = bfsReachable(agentPos, state, level, immovableBoxes,
                    true, agentColor);
            Set<Position> rThrough = null;

            for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
                char goalType = entry.getKey();
                Color boxColor = level.getBoxColor(goalType);
                if (boxColor != agentColor) continue;
                for (Position goalPos : entry.getValue()) {
                    Character boxOnGoal = boxes.get(goalPos);
                    if (boxOnGoal != null && boxOnGoal == goalType) continue;
                    Position boxPos = findNearestSameTypeBox(goalType, goalPos, state, level);
                    if (boxPos == null || boxPos.equals(goalPos)) continue;
                    if (anyAdjacentInSet(boxPos, rActual, level)) continue;
                    if (anyAdjacentInSet(goalPos, rActual, level)) continue;
                    if (rThrough == null) {
                        rThrough = bfsReachable(agentPos, state, level, immovableBoxes,
                                false, agentColor);
                    }
                    if (!anyAdjacentInSet(boxPos, rThrough, level)
                            && !anyAdjacentInSet(goalPos, rThrough, level)) continue;
                    List<Position> blockers = findBlockerPath(agentPos, boxPos, state, level,
                            immovableBoxes, agentColor);
                    for (Position bp : blockers) {
                        Character bt = boxes.get(bp);
                        if (bt == null) continue;
                        Color bc = level.getBoxColor(bt);
                        if (bc == null || bc == agentColor) continue;
                        coupling.computeIfAbsent(agentColor, k -> EnumSet.noneOf(Color.class))
                                .add(bc);
                    }
                }
            }
        }
        return coupling;
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
        return bfsReachable(start, state.getBoxes(), level, immovableBoxes, treatBoxesAsWalls);
    }

    private static Set<Position> bfsReachable(Position start, Map<Position, Character> boxes,
                                              Level level, Set<Position> immovableBoxes,
                                              boolean treatBoxesAsWalls) {
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
     * F5: BFS from {@code startBox} to {@code endGoal} in box-permeable space,
     * then walk back the parent chain to enumerate other-color boxes encountered
     * on the corridor. Used to find blockers in the BOX'S push corridor (not the
     * agent's reach corridor). Excludes startBox itself.
     */
    private static List<Position> findBlockerPathBetween(Position startBox, Position endGoal,
                                                         State state, Level level,
                                                         Set<Position> immovableBoxes,
                                                         Color agentColor) {
        Map<Position, Character> boxes = state.getBoxes();
        int rows = level.getRows();
        int cols = level.getCols();

        Map<Position, Position> parent = new HashMap<>();
        Set<Position> visited = new HashSet<>();
        Deque<Position> queue = new ArrayDeque<>();
        queue.add(startBox);
        visited.add(startBox);
        int expansions = 0;
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};

        boolean found = false;
        while (!queue.isEmpty() && expansions < REACH_BFS_CAP) {
            Position p = queue.poll();
            expansions++;
            if (p.equals(endGoal)) { found = true; break; }
            for (int[] d : dirs) {
                int nr = p.row + d[0];
                int nc = p.col + d[1];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                if (level.isWall(nr, nc)) continue;
                Position np = new Position(nr, nc);
                if (visited.contains(np)) continue;
                if (immovableBoxes.contains(np)) continue;
                visited.add(np);
                parent.put(np, p);
                queue.add(np);
            }
        }

        if (!found) return Collections.emptyList();

        List<Position> blockers = new ArrayList<>();
        Position cur = endGoal;
        while (cur != null && !cur.equals(startBox)) {
            Character bx = boxes.get(cur);
            if (bx != null) {
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
     *
     * <p>F4: prefers "dead-end" cells (only 1 free orthogonal neighbor besides the
     * one we'd come from). Such cells are intrinsically out-of-the-way \u2014 they
     * cannot lie on any other agent's required corridor. The search collects all
     * BFS-visited valid candidates and returns the deadest-end one; falls back to
     * the BFS-nearest valid cell if no dead-end exists within the cap.
     */
    private static Position findPTempBFS(Position boxPos, State state, Level level,
                                         Set<Position> immovableBoxes,
                                         Set<Position> goalCells,
                                         Set<Position> staticNoParkingCells,
                                         Set<Position> dynamicRiskParkingCells,
                                         Set<Position> usedTemps,
                                         ReliefCertificate certificate) {
        Map<Position, Character> boxes = state.getBoxes();
        int rows = level.getRows();
        int cols = level.getCols();
        Set<Position> visited = new HashSet<>();
        Deque<Position> queue = new ArrayDeque<>();
        queue.add(boxPos);
        visited.add(boxPos);
        int expansions = 0;
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};

        Position firstValid = null;
        Position bestDeadEnd = null;
        int bestNeighborCount = Integer.MAX_VALUE;
        Position firstDynamicRisk = null;
        Position bestDynamicRiskDeadEnd = null;
        int bestDynamicRiskNeighborCount = Integer.MAX_VALUE;
        Position firstStaticRisk = null;
        Position bestStaticRiskDeadEnd = null;
        int bestStaticRiskNeighborCount = Integer.MAX_VALUE;

        while (!queue.isEmpty() && expansions < PTEMP_BFS_CAP) {
            Position p = queue.poll();
            expansions++;

            if (!p.equals(boxPos)
                    && !goalCells.contains(p)
                    && !usedTemps.contains(p)
                    && !boxes.containsKey(p)
                    && !immovableBoxes.contains(p)
                    && !isCorner(p, level, immovableBoxes)) {
                boolean releasesBlockerResource = certificate == null
                        || candidateReleasesBlockerResource(boxPos, p, state);
                boolean staticRisk = staticNoParkingCells.contains(p);
                boolean dynamicRisk = dynamicRiskParkingCells.contains(p);
                int free = countFreeNeighbors(p, level, immovableBoxes, boxes);
                if (!releasesBlockerResource) {
                    continue;
                } else if (staticRisk) {
                    if (firstStaticRisk == null) firstStaticRisk = p;
                    if (free < bestStaticRiskNeighborCount) {
                        bestStaticRiskNeighborCount = free;
                        bestStaticRiskDeadEnd = p;
                    }
                } else if (dynamicRisk) {
                    if (firstDynamicRisk == null) firstDynamicRisk = p;
                    if (free < bestDynamicRiskNeighborCount) {
                        bestDynamicRiskNeighborCount = free;
                        bestDynamicRiskDeadEnd = p;
                    }
                } else {
                    if (firstValid == null) firstValid = p;
                    if (free < bestNeighborCount) {
                        bestNeighborCount = free;
                        bestDeadEnd = p;
                        if (free <= 1) break; // can't get better than safe dead-end
                    }
                }
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
        if (bestDeadEnd != null) return bestDeadEnd;
        if (firstValid != null) return firstValid;
        if (bestDynamicRiskDeadEnd != null) return bestDynamicRiskDeadEnd;
        if (firstDynamicRisk != null) return firstDynamicRisk;
        if (bestStaticRiskDeadEnd != null) return bestStaticRiskDeadEnd;
        if (firstStaticRisk != null) return firstStaticRisk;
        return null;
    }

    private static boolean candidateReleasesBlockerResource(Position blockerPos, Position candidate,
                                                            State state) {
        return !candidate.equals(blockerPos) && state.getBoxes().containsKey(blockerPos);
    }

    /** Count orthogonally-adjacent cells that are passable (not wall, not box, not immovable). */
    private static int countFreeNeighbors(Position p, Level level,
                                          Set<Position> immovableBoxes,
                                          Map<Position, Character> boxes) {
        int rows = level.getRows();
        int cols = level.getCols();
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};
        int count = 0;
        for (int[] d : dirs) {
            int nr = p.row + d[0];
            int nc = p.col + d[1];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (level.isWall(nr, nc)) continue;
            Position np = new Position(nr, nc);
            if (immovableBoxes.contains(np)) continue;
            if (boxes.containsKey(np)) continue;
            count++;
        }
        return count;
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
