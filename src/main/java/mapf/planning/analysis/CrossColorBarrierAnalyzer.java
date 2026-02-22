package mapf.planning.analysis;

import mapf.domain.*;
import mapf.planning.SearchConfig;

import java.util.*;

/**
 * Detects cross-color reachability barriers: situations where an agent cannot
 * reach its assigned goals because boxes of a DIFFERENT color block the path.
 * 
 * In pull-supporting Sokoban, any single agent can move any same-color box through
 * arbitrarily narrow corridors using pull chains. Cross-color boxes are the only
 * true permanent obstacles from a single-agent perspective.
 * 
 * This analyzer:
 * 1. BFS from each agent treating all boxes as obstacles → "actual reachable set"
 * 2. BFS from each agent treating only walls as obstacles → "theoretical reachable set"  
 * 3. For agents with goals in theoretical but not actual reachable set → barrier detected
 * 4. Finds the minimum-box-clearing path through the barrier (0-1 BFS)
 * 5. Orders clearing sequence from accessible side inward
 */
public class CrossColorBarrierAnalyzer {

    /**
     * A cross-color barrier blocking an agent from reaching its goals.
     */
    public static class Barrier {
        /** Agent that is blocked. */
        public final int blockedAgentId;
        /** Color of the blocked agent. */
        public final Color blockedAgentColor;
        /** Goal positions that the agent cannot reach. */
        public final List<Position> unreachableGoals;
        /** Box types on the goals (for reference). */
        public final List<Character> goalBoxTypes;
        /** Boxes to clear, in order (first = closest to accessible side). */
        public final List<Position> clearingOrder;
        /** Type of the blocking boxes (e.g., 'A'). */
        public final char blockingBoxType;
        /** Color of the blocking boxes. */
        public final Color blockingColor;

        public Barrier(int blockedAgentId, Color blockedAgentColor,
                       List<Position> unreachableGoals, List<Character> goalBoxTypes,
                       List<Position> clearingOrder, char blockingBoxType, Color blockingColor) {
            this.blockedAgentId = blockedAgentId;
            this.blockedAgentColor = blockedAgentColor;
            this.unreachableGoals = unreachableGoals;
            this.goalBoxTypes = goalBoxTypes;
            this.clearingOrder = clearingOrder;
            this.blockingBoxType = blockingBoxType;
            this.blockingColor = blockingColor;
        }

        @Override
        public String toString() {
            return "Barrier{agent=" + blockedAgentId + " (" + blockedAgentColor + ")"
                    + ", goals=" + unreachableGoals
                    + ", clearing=" + clearingOrder.size() + " " + blockingBoxType + "-boxes"
                    + " (" + blockingColor + ")}";
        }
    }

    /**
     * Analyzes the level for cross-color barriers.
     *
     * @param state      current state
     * @param level      level definition
     * @param subgoals   unsatisfied subgoals (to know which goals each agent needs)
     * @return list of detected barriers (empty if none)
     */
    public static List<Barrier> analyzeBarriers(State state, Level level,
                                                 List<? extends Object> subgoals) {
        List<Barrier> barriers = new ArrayList<>();
        int numAgents = state.getNumAgents();

        for (int agentId = 0; agentId < numAgents; agentId++) {
            Position agentPos = state.getAgentPosition(agentId);
            Color agentColor = level.getAgentColor(agentId);

            // BFS with ALL boxes as obstacles (actual reachability)
            Set<Position> actualReachable = bfsReachable(agentPos, state, level, true);

            // Find unsatisfied goals for this agent's color that are unreachable
            List<Position> unreachableGoals = new ArrayList<>();
            List<Character> goalBoxTypes = new ArrayList<>();

            for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
                char goalType = entry.getKey();
                Color boxColor = level.getBoxColor(goalType);
                if (boxColor != agentColor) continue; // not our color

                for (Position goalPos : entry.getValue()) {
                    // Skip already satisfied goals
                    Character actualBox = state.getBoxes().get(goalPos);
                    if (actualBox != null && actualBox == goalType) continue;

                    // Check if agent can reach the goal area
                    // For push goals, agent needs to reach adjacent cells, not the goal itself
                    boolean canReach = false;
                    for (Direction dir : Direction.values()) {
                        Position adj = goalPos.move(dir);
                        if (actualReachable.contains(adj)) {
                            canReach = true;
                            break;
                        }
                    }
                    // Also check if agent can reach the box to push
                    if (!canReach) {
                        // Find the box that would fill this goal
                        Position boxForGoal = findBoxForGoal(goalType, state, agentColor, level);
                        if (boxForGoal != null) {
                            for (Direction dir : Direction.values()) {
                                Position adj = boxForGoal.move(dir);
                                if (actualReachable.contains(adj)) {
                                    canReach = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!canReach) {
                        unreachableGoals.add(goalPos);
                        goalBoxTypes.add(goalType);
                    }
                }
            }

            if (unreachableGoals.isEmpty()) continue;

            // BFS ignoring all dynamic obstacles (theoretical reachability)
            Set<Position> theoreticalReachable = bfsReachable(agentPos, state, level, false);

            // Verify goals ARE reachable theoretically (otherwise it's a wall problem, not a box barrier)
            List<Position> barrierGoals = new ArrayList<>();
            List<Character> barrierBoxTypes = new ArrayList<>();
            for (int i = 0; i < unreachableGoals.size(); i++) {
                Position g = unreachableGoals.get(i);
                boolean theoreticallyReachable = false;
                for (Direction dir : Direction.values()) {
                    if (theoreticalReachable.contains(g.move(dir))) {
                        theoreticallyReachable = true;
                        break;
                    }
                }
                if (theoreticallyReachable) {
                    barrierGoals.add(g);
                    barrierBoxTypes.add(goalBoxTypes.get(i));
                }
            }

            if (barrierGoals.isEmpty()) continue;

            // Find the minimum-cost path from agent to the first unreachable goal area,
            // where cost = number of cross-color boxes to clear
            Position target = findNearestAccessibleGoalCell(barrierGoals, level);
            if (target == null) target = barrierGoals.get(0);

            List<Position> clearingPath = findMinCostClearingPath(agentPos, target, state, level, agentColor);
            if (clearingPath == null || clearingPath.isEmpty()) continue;

            // Extract cross-color boxes on the path, in order from agent side
            List<Position> clearingOrder = new ArrayList<>();
            char blockingType = '\0';
            Color blockingColor = null;

            for (Position pos : clearingPath) {
                Character box = state.getBoxes().get(pos);
                if (box != null && level.getBoxColor(box) != agentColor) {
                    clearingOrder.add(pos);
                    if (blockingType == '\0') {
                        blockingType = box;
                        blockingColor = level.getBoxColor(box);
                    }
                }
            }

            if (clearingOrder.isEmpty()) continue;

            barriers.add(new Barrier(agentId, agentColor, barrierGoals, barrierBoxTypes,
                    clearingOrder, blockingType, blockingColor));

            if (SearchConfig.isNormal()) {
                System.err.println("[BarrierAnalyzer] " + barriers.get(barriers.size() - 1));
            }
        }

        return barriers;
    }

    /**
     * BFS reachability from a position.
     * @param treatBoxesAsWalls if true, all boxes are impassable; if false, only walls block
     */
    private static Set<Position> bfsReachable(Position start, State state, Level level,
                                               boolean treatBoxesAsWalls) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (treatBoxesAsWalls && state.hasBoxAt(next)) continue;
                // Don't block on agents (they can move out of the way)
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    /**
     * 0-1 BFS to find path from start to target minimizing the number of
     * cross-color boxes that must be cleared. Walls are impassable; same-color
     * boxes cost 0 (agent can push them); cross-color boxes cost 1.
     */
    private static List<Position> findMinCostClearingPath(Position start, Position target,
                                                           State state, Level level,
                                                           Color agentColor) {
        // 0-1 BFS: deque-based, cost 0 edges go to front, cost 1 to back
        Map<Position, Integer> dist = new HashMap<>();
        Map<Position, Position> parent = new HashMap<>();
        Deque<Position> deque = new ArrayDeque<>();

        dist.put(start, 0);
        deque.addFirst(start);

        while (!deque.isEmpty()) {
            Position current = deque.pollFirst();
            int currentDist = dist.get(current);

            if (current.equals(target)) {
                // Reconstruct path
                List<Position> path = new ArrayList<>();
                Position p = target;
                while (p != null && !p.equals(start)) {
                    path.add(p);
                    p = parent.get(p);
                }
                path.add(start);
                Collections.reverse(path);
                return path;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (level.isWall(next)) continue;

                int edgeCost = 0;
                Character box = state.getBoxes().get(next);
                if (box != null) {
                    Color boxColor = level.getBoxColor(box);
                    if (boxColor != agentColor) {
                        edgeCost = 1; // cross-color box = must clear
                    }
                    // same-color box = 0 (agent can push/pull it in theory)
                }

                int newDist = currentDist + edgeCost;
                if (newDist < dist.getOrDefault(next, Integer.MAX_VALUE)) {
                    dist.put(next, newDist);
                    parent.put(next, current);
                    if (edgeCost == 0) {
                        deque.addFirst(next);
                    } else {
                        deque.addLast(next);
                    }
                }
            }
        }

        return null; // no path exists even theoretically
    }

    /**
     * Find a cell adjacent to one of the goals that is walkable (not a wall).
     * This is the "entry point" the agent needs to reach.
     */
    private static Position findNearestAccessibleGoalCell(List<Position> goals, Level level) {
        // Return the first free cell adjacent to any goal
        for (Position goal : goals) {
            for (Direction dir : Direction.values()) {
                Position adj = goal.move(dir);
                if (!level.isWall(adj)) return adj;
            }
        }
        return null;
    }

    /**
     * Find a box of the given type that the agent color can manipulate.
     * Returns the nearest non-goal-satisfied box.
     */
    private static Position findBoxForGoal(char boxType, State state, Color agentColor, Level level) {
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                return entry.getKey();
            }
        }
        return null;
    }

    /**
     * Finds temporary parking positions for cleared boxes.
     * Uses BFS from the clearing entry point (first box in clearing order)
     * to find free, non-goal cells in the agent's reachable area.
     *
     * @param numPositions how many parking spots needed
     * @param accessibleArea positions reachable by the clearing agent
     * @param state          current state
     * @param level          level definition
     * @param avoidPositions positions to avoid (e.g., goals, other parkings)
     * @return list of parking positions, or empty if not enough found
     */
    public static List<Position> findParkingPositions(int numPositions,
                                                       Set<Position> accessibleArea,
                                                       State state, Level level,
                                                       Set<Position> avoidPositions) {
        return findParkingPositions(numPositions, accessibleArea, state, level, avoidPositions, null);
    }

    /**
     * Finds temporary parking positions for cleared boxes.
     * When a reference position is given, sorts by proximity to it (so BSP
     * has a shorter box displacement path). Falls back to free-neighbor count.
     *
     * @param numPositions   how many parking spots needed
     * @param accessibleArea positions reachable by the clearing agent
     * @param state          current state
     * @param level          level definition
     * @param avoidPositions positions to avoid (e.g., goals, barrier boxes)
     * @param referencePos   optional reference position for proximity sorting
     * @return list of parking positions
     */
    public static List<Position> findParkingPositions(int numPositions,
                                                       Set<Position> accessibleArea,
                                                       State state, Level level,
                                                       Set<Position> avoidPositions,
                                                       Position referencePos) {
        List<Position> parking = new ArrayList<>();

        // Score positions: prefer ones far from goals and congestion
        List<Position> candidates = new ArrayList<>();
        for (Position pos : accessibleArea) {
            if (level.isWall(pos)) continue;
            if (state.hasBoxAt(pos)) continue;
            if (state.hasAgentAt(pos)) continue;
            if (avoidPositions.contains(pos)) continue;
            // Don't park on goal positions (would create false goal satisfaction)
            if (level.getBoxGoal(pos) != '\0') continue;
            // Don't park on agent goal positions
            if (level.getAgentGoal(pos.row, pos.col) != -1) continue;
            candidates.add(pos);
        }

        // Sort: moderate distance from reference — far enough to not block approach
        // corridor, close enough for BSP to find a displacement path.
        // Filter out positions within Manhattan 1 of reference (directly adjacent to gap).
        // Previous threshold of 4 was too aggressive for large barriers (e.g., ZOOM)
        // where nearby row-10 positions are the only viable parking.
        final Position sortRef = referencePos;
        if (sortRef != null) {
            candidates.removeIf(pos -> pos.manhattanDistance(sortRef) < 2);
        }
        candidates.sort((a, b) -> {
            if (sortRef != null) {
                int distA = a.manhattanDistance(sortRef);
                int distB = b.manhattanDistance(sortRef);
                if (distA != distB) return Integer.compare(distA, distB); // closer (but ≥4) first
            }
            int freeA = countFreeNeighbors(a, state, level);
            int freeB = countFreeNeighbors(b, state, level);
            return Integer.compare(freeB, freeA); // more free neighbors = better
        });

        for (Position c : candidates) {
            parking.add(c);
            if (parking.size() >= numPositions) break;
        }

        return parking;
    }

    private static int countFreeNeighbors(Position pos, State state, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position n = pos.move(dir);
            if (!level.isWall(n) && !state.hasBoxAt(n)) count++;
        }
        return count;
    }

    /**
     * Checks if a barrier box can physically be extracted from its current position.
     * 
     * A box is extractable if the clearing agent can:
     * - Access it from 2+ sides (can push from one side to the other), OR
     * - Access it from 1 side AND after pulling, can bypass the box to continue
     *   moving it (requires an alternative path around the pulled box).
     * 
     * This avoids expensive BSP searches on boxes trapped in dead-end corridors
     * (e.g., a box in a packed area with only one narrow exit that has no bypass).
     *
     * @param boxPos position of the box to extract
     * @param agentReachable cells reachable by the clearing agent (boxes = walls)
     * @param state current state
     * @param level level definition
     * @return true if the box appears extractable
     */
    public static boolean isBoxExtractable(Position boxPos, Set<Position> agentReachable,
                                            State state, Level level) {
        return isBoxExtractable(boxPos, agentReachable, state, level, Collections.emptySet());
    }

    /**
     * Overload for barrier clearing context: otherBarrierBoxes are boxes that will
     * be sequentially cleared, so they should be treated as "removable" (free space)
     * when checking bypass routes. This fixes the case where a single-exit gap (like
     * (9,13) in ZOOM) appears non-extractable because the pulled box blocks the only
     * return path — but in reality, after parking the box aside, the gap re-opens.
     */
    public static boolean isBoxExtractable(Position boxPos, Set<Position> agentReachable,
                                            State state, Level level,
                                            Set<Position> otherBarrierBoxes) {
        // Count how many sides the agent can access
        List<Direction> accessibleDirs = new ArrayList<>();
        for (Direction dir : Direction.values()) {
            Position adj = boxPos.move(dir);
            if (agentReachable.contains(adj)) {
                accessibleDirs.add(dir);
            }
        }

        if (accessibleDirs.isEmpty()) return false;
        if (accessibleDirs.size() >= 2) return true; // Can push from one side to another

        // Single accessible direction: check if the agent can PUSH directly.
        Direction accessDir = accessibleDirs.get(0);
        Position pushTarget = boxPos.move(accessDir.opposite());
        if (!level.isWall(pushTarget) && !state.hasBoxAt(pushTarget)) {
            return true; // Agent can push the box directly from this side
        }
        // Also check if push target is an other-barrier box (will be cleared)
        if (!level.isWall(pushTarget) && otherBarrierBoxes.contains(pushTarget)) {
            return true;
        }

        // Can't push directly. Use pull-chain BFS: simulate a series of pulls
        // to move the box away from boxPos, checking at each step whether the
        // agent can bypass the box and return to boxPos. This handles narrow
        // gap scenarios where the agent needs to pull the box through a 1-cell
        // passage and then walk around via a longer route.
        Position adjPos = boxPos.move(accessDir); // agent stands here to start
        // For barrier clearing: first try with barrier-aware bypass (treats other
        // barrier boxes as removable). If that passes, the box is extractable in
        // the sequential clearing context.
        if (!otherBarrierBoxes.isEmpty()) {
            if (canPullChainExtractBarrier(boxPos, adjPos, state, level, otherBarrierBoxes)) {
                return true;
            }
        }
        return canPullChainExtract(boxPos, adjPos, state, level);
    }

    /**
     * BFS in (agentPos, boxPos) state space to check whether a pull-chain can
     * extract the box from its position through a narrow gap.
     *
     * Starting state: agent adjacent to box (at agentStartPos). Each BFS step
     * is a valid Pull action: agent moves in some direction, box (which must be
     * adjacent) slides into the agent's vacated cell. After each pull we run a
     * bypass BFS to check whether the agent can walk back to originalBoxPos
     * (now empty) without passing through the box.
     *
     * Bounded by MAX_CHAIN_DEPTH to keep the check fast.
     *
     * @param originalBoxPos where the box currently sits
     * @param agentStartPos  where the clearing agent stands (adjacent to box)
     * @param state          current world state (for wall/box checks)
     * @param level          level definition
     * @return true if a feasible pull-chain extraction exists
     */
    private static boolean canPullChainExtract(Position originalBoxPos, Position agentStartPos,
                                                State state, Level level) {
        final int MAX_CHAIN_DEPTH = 10;

        // Encode (agentPos, boxPos) as a single long for fast hashing
        // Position row/col fit in 8 bits each (max grid 50×50)
        Set<Long> visited = new HashSet<>();
        Queue<long[]> queue = new LinkedList<>(); // [encoded state, depth]

        long initState = encodePullState(agentStartPos, originalBoxPos);
        visited.add(initState);
        queue.add(new long[]{initState, 0});

        while (!queue.isEmpty()) {
            long[] entry = queue.poll();
            long enc = entry[0];
            int depth = (int) entry[1];
            if (depth >= MAX_CHAIN_DEPTH) continue;

            Position agentPos = decodeAgent(enc);
            Position boxPos = decodeBox(enc);

            // Box must be adjacent to agent for a Pull
            int dr = boxPos.row - agentPos.row;
            int dc = boxPos.col - agentPos.col;
            if (Math.abs(dr) + Math.abs(dc) != 1) continue;

            // Try all 4 agent movement directions (Pull action)
            for (Direction agentDir : Direction.values()) {
                Position newAgentPos = agentPos.move(agentDir);

                // Agent destination must be free
                if (level.isWall(newAgentPos)) continue;
                if (newAgentPos.equals(boxPos)) continue;
                if (state.hasBoxAt(newAgentPos) && !newAgentPos.equals(originalBoxPos)) continue;

                // After Pull: box slides into agent's vacated cell
                Position newBoxPos = agentPos; // box moves to where agent was

                // Check bypass: can agent at newAgentPos reach originalBoxPos
                // without passing through newBoxPos (the box)?
                if (canBypassToTarget(newAgentPos, newBoxPos, originalBoxPos, state, level)) {
                    return true;
                }

                long nextState = encodePullState(newAgentPos, newBoxPos);
                if (!visited.contains(nextState)) {
                    visited.add(nextState);
                    queue.add(new long[]{nextState, depth + 1});
                }
            }
        }

        return false;
    }

    /**
     * BFS reachability check: can the agent walk from startPos to targetPos
     * without passing through boxPos? Original-state boxes block except
     * targetPos (which was emptied by the pull).
     */
    private static boolean canBypassToTarget(Position startPos, Position boxPos,
                                              Position targetPos, State state, Level level) {
        return canBypassToTarget(startPos, boxPos, targetPos, state, level, Collections.emptySet());
    }

    /**
     * Barrier-aware bypass: otherBarrierBoxes are treated as free (they'll be cleared).
     */
    private static boolean canBypassToTarget(Position startPos, Position boxPos,
                                              Position targetPos, State state, Level level,
                                              Set<Position> otherBarrierBoxes) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> bfsQueue = new LinkedList<>();
        visited.add(startPos);
        bfsQueue.add(startPos);

        while (!bfsQueue.isEmpty()) {
            Position current = bfsQueue.poll();
            if (current.equals(targetPos)) return true;

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (next.equals(boxPos)) continue; // current box position
                // Original boxes block, except targetPos (emptied by extraction)
                // and other barrier boxes (will be cleared sequentially)
                if (state.hasBoxAt(next) && !next.equals(targetPos)
                        && !otherBarrierBoxes.contains(next)) continue;
                visited.add(next);
                bfsQueue.add(next);
            }
        }
        return false;
    }

    /**
     * Barrier-aware pull-chain extraction: like canPullChainExtract but treats
     * other barrier boxes as removable in bypass checks.
     */
    private static boolean canPullChainExtractBarrier(Position originalBoxPos, Position agentStartPos,
                                                      State state, Level level,
                                                      Set<Position> otherBarrierBoxes) {
        final int MAX_CHAIN_DEPTH = 10;
        Set<Long> visited = new HashSet<>();
        Queue<long[]> queue = new LinkedList<>();

        long initState = encodePullState(agentStartPos, originalBoxPos);
        visited.add(initState);
        queue.add(new long[]{initState, 0});

        while (!queue.isEmpty()) {
            long[] entry = queue.poll();
            long enc = entry[0];
            int depth = (int) entry[1];
            if (depth >= MAX_CHAIN_DEPTH) continue;

            Position agentPos = decodeAgent(enc);
            Position boxPos = decodeBox(enc);

            int dr = boxPos.row - agentPos.row;
            int dc = boxPos.col - agentPos.col;
            if (Math.abs(dr) + Math.abs(dc) != 1) continue;

            for (Direction agentDir : Direction.values()) {
                Position newAgentPos = agentPos.move(agentDir);
                if (level.isWall(newAgentPos)) continue;
                if (newAgentPos.equals(boxPos)) continue;
                // Agent can move through other barrier boxes (they'll be cleared)
                if (state.hasBoxAt(newAgentPos) && !newAgentPos.equals(originalBoxPos)
                        && !otherBarrierBoxes.contains(newAgentPos)) continue;

                Position newBoxPos = agentPos;

                // Barrier-aware bypass check
                if (canBypassToTarget(newAgentPos, newBoxPos, originalBoxPos, state, level, otherBarrierBoxes)) {
                    return true;
                }

                long nextState = encodePullState(newAgentPos, newBoxPos);
                if (!visited.contains(nextState)) {
                    visited.add(nextState);
                    queue.add(new long[]{nextState, depth + 1});
                }
            }
        }
        return false;
    }

    private static long encodePullState(Position agent, Position box) {
        return ((long) agent.row << 24) | ((long) agent.col << 16)
             | ((long) box.row << 8) | (long) box.col;
    }

    private static Position decodeAgent(long enc) {
        return Position.of((int) ((enc >> 24) & 0xFF), (int) ((enc >> 16) & 0xFF));
    }

    private static Position decodeBox(long enc) {
        return Position.of((int) ((enc >> 8) & 0xFF), (int) (enc & 0xFF));
    }

    /**
     * Find a clearing agent: agent of the blocking color that can reach the
     * first box in the clearing order (from the accessible side).
     */
    public static int findClearingAgent(Color blockingColor, Position firstBoxPos,
                                         State state, Level level) {
        List<int[]> agents = findAllClearingAgents(blockingColor, firstBoxPos, state, level);
        return agents.isEmpty() ? -1 : agents.get(0)[0];
    }

    /**
     * Finds ALL agents of the matching color that could clear barrier boxes,
     * sorted by BFS distance to the first barrier box (closest first).
     *
     * @return list of [agentId, distance] pairs, sorted by distance ascending
     */
    public static List<int[]> findAllClearingAgents(Color blockingColor, Position firstBoxPos,
                                                      State state, Level level) {
        List<int[]> agents = new ArrayList<>();
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (level.getAgentColor(i) != blockingColor) continue;
            Position agentPos = state.getAgentPosition(i);
            int dist = bfsDistance(agentPos, firstBoxPos, state, level);
            if (dist < Integer.MAX_VALUE) {
                agents.add(new int[]{i, dist});
            }
        }
        agents.sort((a, b) -> Integer.compare(a[1], b[1]));
        return agents;
    }

    /**
     * BFS distance treating boxes as obstacles (but the agent can walk around them
     * if there's a path). Returns Integer.MAX_VALUE if unreachable.
     */
    private static int bfsDistance(Position start, Position target, State state, Level level) {
        if (start.equals(target)) return 0;

        Map<Position, Integer> dist = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        dist.put(start, 0);
        queue.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int d = dist.get(current);

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (dist.containsKey(next)) continue;
                if (level.isWall(next)) continue;
                if (state.hasBoxAt(next) && !next.equals(target)) continue;
                dist.put(next, d + 1);
                if (next.equals(target)) return d + 1;
                queue.add(next);
            }
        }
        return Integer.MAX_VALUE;
    }
}
