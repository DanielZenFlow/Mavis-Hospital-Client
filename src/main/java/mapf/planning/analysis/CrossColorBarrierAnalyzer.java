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

        // Sort by distance from the barrier area (prefer peripheral positions)
        // Simple heuristic: prefer cells with more free neighbors (less congested)
        candidates.sort((a, b) -> {
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
        // Push: agent at adjPos pushes box toward the opposite side.
        // If the opposite side is free, push is trivially possible.
        Direction accessDir = accessibleDirs.get(0);
        Position pushTarget = boxPos.move(accessDir.opposite()); // opposite side of box
        if (!level.isWall(pushTarget) && !state.hasBoxAt(pushTarget)) {
            return true; // Agent can push the box directly from this side
        }

        // Can't push directly. Check if a PULL + bypass is feasible.
        // After pulling: box moves to adjPos (agent's cell), agent moves one further.
        // Agent must be able to reach the original boxPos via bypass to continue.
        Position adjPos = boxPos.move(accessDir);       // box goes here after pull
        Position agentAfter = adjPos.move(accessDir);   // agent ends up here after pull

        // Can the agent actually complete the pull? (agentAfter must be free)
        if (level.isWall(agentAfter) || state.hasBoxAt(agentAfter)) {
            // Check other pull directions from adjPos
            boolean canPull = false;
            for (Direction d : Direction.values()) {
                Position altAgent = adjPos.move(d);
                if (d.opposite() == accessDir.opposite()) continue; // same as going toward box
                if (!level.isWall(altAgent) && !state.hasBoxAt(altAgent) && !altAgent.equals(boxPos)) {
                    agentAfter = altAgent;
                    canPull = true;
                    break;
                }
            }
            if (!canPull) return false;
        }

        // After pull: box at adjPos, agent at agentAfter, boxPos is now free.
        // BFS from agentAfter: can agent reach boxPos without going through adjPos?
        Set<Position> bypass = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        bypass.add(agentAfter);
        queue.add(agentAfter);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (bypass.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (next.equals(adjPos)) continue;  // box is here, can't pass
                // Other boxes still block EXCEPT boxPos which is now empty
                if (state.hasBoxAt(next) && !next.equals(boxPos)) continue;
                bypass.add(next);
                queue.add(next);
            }
        }

        return bypass.contains(boxPos);
    }

    /**
     * Find a clearing agent: agent of the blocking color that can reach the
     * first box in the clearing order (from the accessible side).
     */
    public static int findClearingAgent(Color blockingColor, Position firstBoxPos,
                                         State state, Level level) {
        int bestAgent = -1;
        int bestDist = Integer.MAX_VALUE;

        for (int i = 0; i < state.getNumAgents(); i++) {
            if (level.getAgentColor(i) != blockingColor) continue;
            Position agentPos = state.getAgentPosition(i);

            // BFS distance from agent to adjacent cells of the first box
            // (agent needs to be adjacent to pull/push)
            int dist = bfsDistance(agentPos, firstBoxPos, state, level);
            if (dist < bestDist) {
                bestDist = dist;
                bestAgent = i;
            }
        }
        return bestAgent;
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
