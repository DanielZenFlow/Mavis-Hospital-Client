package mapf.planning.coordination;

import mapf.domain.*;
import mapf.planning.SearchConfig;

import java.util.*;

/**
 * Detects and resolves deadlocks in multi-agent pathfinding.
 * 
 * Deadlock types handled:
 * 1. Circular agent dependency: A waits for B, B waits for C, C waits for A
 * 2. Box blocking: Multiple boxes block each other's paths
 * 3. Mixed blocking: Agents and boxes form a blocking cycle
 * 
 * Resolution strategies:
 * 1. Temporary displacement: Move a blocking box to a temporary location
 * 2. Agent yielding: Force an agent to move out of the way
 * 3. Path rerouting: Find alternative paths that avoid the deadlock
 */
public class DeadlockResolver {

    /**
     * Represents a blocking relationship.
     */
    public static class BlockingInfo {
        public final int blockedAgentId;
        public final Position blockedGoal;
        public final Position blockingPosition;  // What's blocking
        public final boolean isBlockedByBox;     // true = box, false = agent
        public final char blockingBoxType;       // If blocked by box
        public final int blockingAgentId;        // If blocked by agent

        public BlockingInfo(int blockedAgentId, Position blockedGoal, Position blockingPos,
                           boolean isBlockedByBox, char boxType, int blockingAgentId) {
            this.blockedAgentId = blockedAgentId;
            this.blockedGoal = blockedGoal;
            this.blockingPosition = blockingPos;
            this.isBlockedByBox = isBlockedByBox;
            this.blockingBoxType = boxType;
            this.blockingAgentId = blockingAgentId;
        }

        @Override
        public String toString() {
            if (isBlockedByBox) {
                return "Agent " + blockedAgentId + " blocked by Box " + blockingBoxType + 
                       " at " + blockingPosition;
            } else {
                return "Agent " + blockedAgentId + " blocked by Agent " + blockingAgentId +
                       " at " + blockingPosition;
            }
        }
    }

    /**
     * Represents a displacement action - moving a box temporarily to unblock others.
     */
    public static class DisplacementPlan {
        public final int agentId;           // Agent that will do the displacement
        public final Position boxPosition;  // Current box position (or agent position if agent displacement)
        public final char boxType;          // Box type (or \0 if agent displacement)
        public final Position tempPosition; // Temporary destination
        public final String reason;         // Why this displacement
        public final boolean isAgentDisplacement;

        public DisplacementPlan(int agentId, Position boxPos, char boxType, 
                               Position tempPos, String reason) {
            this(agentId, boxPos, boxType, tempPos, reason, false);
        }

        public DisplacementPlan(int agentId, Position pos, char boxType, 
                               Position tempPos, String reason, boolean isAgent) {
            this.agentId = agentId;
            this.boxPosition = pos;
            this.boxType = boxType;
            this.tempPosition = tempPos;
            this.reason = reason;
            this.isAgentDisplacement = isAgent;
        }

        @Override
        public String toString() {
            if (isAgentDisplacement) {
                return "Displace Agent " + agentId + " from " + boxPosition + " to " + tempPosition +
                       ": " + reason;
            }
            return "Displace Box " + boxType + " from " + boxPosition + " to " + tempPosition +
                   " (Agent " + agentId + "): " + reason;
        }
    }

    // Logging control
    private boolean verboseLogging = false;

    public DeadlockResolver() {
    }

    public void setVerboseLogging(boolean verbose) {
        this.verboseLogging = verbose;
    }

    /**
     * Analyzes the current state to find blocking relationships.
     * 
     * @param state Current state
     * @param level Level information
     * @param blockedAgents List of agent IDs that are currently blocked (can't make progress)
     * @return List of blocking relationships
     */
    public List<BlockingInfo> analyzeBlocking(State state, Level level, List<Integer> blockedAgents) {
        return analyzeBlockingForAgents(state, level, blockedAgents, Collections.emptySet());
    }
    
    /**
     * Analyzes the current state to find blocking relationships using explicit subgoals.
     * Checks both agent-to-box and box-to-goal paths for box tasks.
     * Immovable boxes are treated as walls and skipped.
     */
    public List<BlockingInfo> analyzeBlocking(State state, Level level, 
            List<mapf.planning.strategy.PriorityPlanningStrategy.Subgoal> subgoals,
            Set<Position> immovableBoxes) {
        
        List<BlockingInfo> blockingInfos = new ArrayList<>();
        mapf.planning.strategy.SubgoalManager outputHelper = new mapf.planning.strategy.SubgoalManager(null); // Helper for box finding

        for (mapf.planning.strategy.PriorityPlanningStrategy.Subgoal sg : subgoals) {
            Position agentPos = state.getAgentPosition(sg.agentId);
            
            if (sg.isAgentGoal) {
                // Agent goal: simple A->B check
                BlockingInfo blocking = findBlockingObstacle(sg.agentId, agentPos, sg.goalPos, state, level, immovableBoxes, null);
                if (blocking != null) blockingInfos.add(blocking);
            } else {
                // Box goal: Two-stage check
                // 1. Agent -> Box
                Position boxPos = outputHelper.findBestBoxForGoal(sg, state, level, Collections.emptySet());
                if (boxPos == null) continue; // Can't find box, can't check blocking
                                
                // Allow agent to reach the box (don't treat target box as obstacle)
                BlockingInfo agentToBox = findBlockingObstacle(sg.agentId, agentPos, boxPos, state, level, immovableBoxes, boxPos);
                if (agentToBox != null) {
                    blockingInfos.add(agentToBox);
                } else {
                    // Agent can reach box (or is at box). Now check Box -> Goal
                    // Note: Agent pushes/pulls box, so path is roughly same. 
                    // Treating as "Box moves to Goal" for obstacle detection.
                    // Pass agentId because the agent is "driving" the box.
                    BlockingInfo boxToGoal = findBlockingObstacle(sg.agentId, boxPos, sg.goalPos, state, level, immovableBoxes, boxPos);
                    
                    if (boxToGoal != null) {
                        blockingInfos.add(boxToGoal);
                    }
                }
            }
        }

        return blockingInfos;
    }
    
    /** Legacy adapter for backward compatibility (if needed) */
    public List<BlockingInfo> analyzeBlockingForAgents(State state, Level level, List<Integer> blockedAgents,
                                              Set<Position> immovableBoxes) {
        // ... legacy implementation or throw ...
        return new ArrayList<>(); 
    }

    /**
     * Finds the current goal for an agent (either a box goal or agent goal).
     */
    private Position findAgentCurrentGoal(int agentId, State state, Level level) {
        Color agentColor = level.getAgentColor(agentId);

        // First, check for unsatisfied box goals this agent can handle
        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            Color boxColor = level.getBoxColor(goalType);
            if (boxColor != agentColor) continue;
            for (Position goalPos : entry.getValue()) {
                char currentBox = state.getBoxAt(goalPos);
                if (currentBox != goalType) {
                    // This goal is unsatisfied and this agent can handle it
                    return goalPos;
                }
            }
        }

        // Then check agent goal
        Position goalPos = level.getAgentGoalPositionMap().get(agentId);
        if (goalPos != null && !state.getAgentPosition(agentId).equals(goalPos)) {
            return goalPos;
        }

        return null;
    }

    /**
     * Finds what's blocking an agent's path to a goal.
     * Two-phase BFS:
     *   Phase 1: Check if a clear path exists (treating movable obstacles as passable).
     *            If yes → NOT blocked, return null.
     *   Phase 2: If no clear path, find the first movable obstacle on the shortest
     *            "obstacle-passable" path (the one that must be displaced).
     * 
     * Immovable boxes are always treated as walls.
     * 
     * @param targetBoxPos Optional: if set, this specific box position is NOT considered an obstacle
     *                     (used when checking path TO a box, or path OF a box)
     */
    private BlockingInfo findBlockingObstacle(int agentId, Position start, Position goal, 
                                              State state, Level level, Set<Position> immovableBoxes,
                                              Position targetBoxPos) {
        // Precompute obstacle positions (movable boxes + other agents), excluding targetBox
        Set<Position> movableObstacles = new HashSet<>();
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            if (targetBoxPos != null && box.getKey().equals(targetBoxPos)) continue;
            if (immovableBoxes.contains(box.getKey())) continue;
            movableObstacles.add(box.getKey());
        }
        for (int otherId = 0; otherId < state.getNumAgents(); otherId++) {
            if (otherId != agentId) {
                movableObstacles.add(state.getAgentPosition(otherId));
            }
        }

        // === Phase 1: BFS treating movable obstacles AS WALLS ===
        // If goal is reachable without touching any obstacle → not blocked
        {
            Queue<Position> queue = new LinkedList<>();
            Set<Position> visited = new HashSet<>();
            queue.add(start);
            visited.add(start);

            while (!queue.isEmpty()) {
                Position current = queue.poll();
                if (current.equals(goal)) return null; // Clear path exists!

                for (Direction dir : Direction.values()) {
                    Position next = current.move(dir);
                    if (visited.contains(next)) continue;
                    if (level.isWall(next)) continue;
                    if (immovableBoxes.contains(next)) continue;
                    if (movableObstacles.contains(next)) continue; // Treat as wall
                    visited.add(next);
                    queue.add(next);
                }
            }
        }

        // === Phase 2: BFS treating movable obstacles as PASSABLE ===
        // Goal was NOT reachable via clear path.  Find which obstacle is most CRITICAL.
        // We collect ALL reachable obstacles and choose the one closest to the GOAL.
        // This avoids blaming an agent floating in open space when the real blocker is a box at the choke point.
        {
            Queue<Position> queue = new LinkedList<>();
            Set<Position> visited = new HashSet<>();
            List<BlockingInfo> candidates = new ArrayList<>();
            
            queue.add(start);
            visited.add(start);

            // Safety limit to prevent scanning entire map if unreachable
            int explored = 0;
            int maxExplored = level.getRows() * level.getCols();

            while (!queue.isEmpty() && explored < maxExplored) {
                Position current = queue.poll();
                explored++;
                
                // If we reached the goal (ignoring obstacles), we found a path.
                // The candidates list has the obstacles on/near that path.
                if (current.equals(goal)) break; 

                for (Direction dir : Direction.values()) {
                    Position next = current.move(dir);
                    if (visited.contains(next)) continue;
                    if (level.isWall(next)) continue;
                    if (immovableBoxes.contains(next)) continue;

                    visited.add(next);

                    // If this is a movable obstacle, record it but continue searching
                    if (movableObstacles.contains(next)) {
                        BlockingInfo info = null;
                        Character box = state.getBoxes().get(next);
                        if (box != null && (targetBoxPos == null || !next.equals(targetBoxPos))) {
                            info = new BlockingInfo(agentId, goal, next, true, box, -1);
                        } else {
                             for (int otherId = 0; otherId < state.getNumAgents(); otherId++) {
                                if (otherId != agentId && state.getAgentPosition(otherId).equals(next)) {
                                    info = new BlockingInfo(agentId, goal, next, false, '\0', otherId);
                                    break;
                                }
                            }
                        }
                        
                        if (info != null) {
                            candidates.add(info);
                        }
                    }

                    // Treat obstacles as transparent for connectivity check
                    queue.add(next);
                }
            }
            
            // Return the obstacle closest to the goal (heuristic: the most critical bottleneck)
            if (!candidates.isEmpty()) {
                candidates.sort(Comparator.comparingInt(b -> b.blockingPosition.manhattanDistance(goal)));
                return candidates.get(0);
            }
        }

        // No path found even through obstacles (completely walled off)
        return null;
    }

    /**
     * Detects circular dependencies in the blocking graph.
     * 
     * @param blockingInfos List of blocking relationships
     * @return List of agent IDs forming a cycle, or empty if no cycle
     */
    public List<Integer> detectCycle(List<BlockingInfo> blockingInfos) {
        // Build dependency graph: agent -> set of agents it depends on
        Map<Integer, Set<Integer>> dependsOn = new HashMap<>();

        for (BlockingInfo info : blockingInfos) {
            if (!info.isBlockedByBox) {
                // Direct agent-to-agent blocking
                dependsOn.computeIfAbsent(info.blockedAgentId, k -> new HashSet<>())
                         .add(info.blockingAgentId);
            }
        }

        // DFS to find cycle
        Set<Integer> visited = new HashSet<>();
        Set<Integer> inStack = new HashSet<>();
        List<Integer> cycle = new ArrayList<>();

        for (int agent : dependsOn.keySet()) {
            if (findCycleDFS(agent, dependsOn, visited, inStack, cycle)) {
                return cycle;
            }
        }

        return Collections.emptyList();
    }

    private boolean findCycleDFS(int current, Map<Integer, Set<Integer>> graph,
                                 Set<Integer> visited, Set<Integer> inStack, List<Integer> cycle) {
        if (inStack.contains(current)) {
            cycle.add(current);
            return true;
        }
        if (visited.contains(current)) {
            return false;
        }

        visited.add(current);
        inStack.add(current);

        Set<Integer> neighbors = graph.get(current);
        if (neighbors != null) {
            for (int neighbor : neighbors) {
                if (findCycleDFS(neighbor, graph, visited, inStack, cycle)) {
                    if (cycle.size() == 1 || !cycle.get(0).equals(cycle.get(cycle.size() - 1))) {
                        cycle.add(current);
                    }
                    return true;
                }
            }
        }

        inStack.remove(current);
        return false;
    }

    /**
     * Creates a displacement plan to resolve a deadlock.
     * Strategy: Find a blocking box that can be temporarily moved to a "parking" spot.
     * 
     * @param blockingInfos Blocking relationships
     * @param state Current state
     * @param level Level information
     * @param displacementHistory Set of already-attempted displacements
     * @return A displacement plan, or null if none found
     */
    public DisplacementPlan createDisplacementPlan(List<BlockingInfo> blockingInfos, 
                                                   State state, Level level,
                                                   Set<String> displacementHistory) {
        // 1. Analyze what is blocking (Boxes AND Agents)
        Map<Position, Integer> blockingCount = new HashMap<>(); // Position -> count
        Map<Integer, Integer> blockingAgentCount = new HashMap<>(); // AgentID -> count
        
        for (BlockingInfo info : blockingInfos) {
            if (info.isBlockedByBox) {
                blockingCount.merge(info.blockingPosition, 1, Integer::sum);
            } else {
                blockingAgentCount.merge(info.blockingAgentId, 1, Integer::sum);
            }
        }

        // 2. Try to move Blocking Agents FIRST (they are easier to move)
        List<Map.Entry<Integer, Integer>> sortedAgents = new ArrayList<>(blockingAgentCount.entrySet());
        sortedAgents.sort((a, b) -> Integer.compare(b.getValue(), a.getValue()));

        for (Map.Entry<Integer, Integer> entry : sortedAgents) {
            int agentId = entry.getKey();
            Position agentPos = state.getAgentPosition(agentId);
            
            // Check history
            String historyKey = "Agent" + agentId + "@" + agentPos;
            if (displacementHistory.contains(historyKey)) continue;

            // Find parking spot for agent
            // Agent moves ITSELF.
            Position parkingSpot = findParkingSpot(agentPos, state, level, blockingInfos);
            
            if (parkingSpot != null) {
                String reason = "Agent " + agentId + " blocking " + entry.getValue() + " others";
                return new DisplacementPlan(agentId, agentPos, '\0', parkingSpot, reason, true);
            }
        }

        // 3. Try to move Blocking Boxes (Existing Logic)
        List<Map.Entry<Position, Integer>> sortedBoxes = new ArrayList<>(blockingCount.entrySet());
        sortedBoxes.sort((a, b) -> Integer.compare(b.getValue(), a.getValue())); // Most blocking first

        for (Map.Entry<Position, Integer> entry : sortedBoxes) {
            Position boxPos = entry.getKey();
            char boxType = state.getBoxAt(boxPos);
            
            if (boxType == '\0') continue;

            // Skip if already tried this displacement
            String historyKey = boxType + "@" + boxPos;
            if (displacementHistory.contains(historyKey)) {
                continue;
            }

            // Find an agent that can move this box
            Color boxColor = level.getBoxColor(boxType);
            int pushingAgent = -1;
            
            for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
                if (level.getAgentColor(agentId) == boxColor) {
                    // Check if this agent can reach the box
                    Position agentPos = state.getAgentPosition(agentId);
                    if (canReachBox(agentPos, boxPos, state, level, boxColor)) {
                        pushingAgent = agentId;
                        break;
                    }
                }
            }

            if (pushingAgent == -1) {
                continue;
            }

            // Find a temporary parking spot for this box
            Position parkingSpot = findParkingSpot(boxPos, state, level, blockingInfos);
            
            if (parkingSpot != null) {
                String reason = "Blocking " + entry.getValue() + " agent(s)";
                return new DisplacementPlan(pushingAgent, boxPos, boxType, parkingSpot, reason, false);
            }
        }

        return null;
    }

    /**
     * Checks if an agent can reach a box position.
     * In push-pull domain, same-color boxes can be pushed/pulled out of the way,
     * so they are treated as passable. Only different-color boxes block.
     */
    private boolean canReachBox(Position agentPos, Position boxPos, State state, Level level, Color agentColor) {
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();

        queue.add(agentPos);
        visited.add(agentPos);

        int maxSteps = level.getRows() * level.getCols();
        int steps = 0;

        while (!queue.isEmpty() && steps < maxSteps) {
            Position current = queue.poll();
            steps++;

            // Check if adjacent to box
            for (Direction dir : Direction.values()) {
                if (current.move(dir).equals(boxPos)) {
                    return true;
                }
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!visited.contains(next) && !level.isWall(next)) {
                    // Agents are temporary obstacles (they can yield)
                    boolean agentBlocking = false;
                    for (int i = 0; i < state.getNumAgents(); i++) {
                        if (state.getAgentPosition(i).equals(next)) {
                            agentBlocking = true;
                            break;
                        }
                    }
                    if (agentBlocking) continue;
                    
                    // Same-color boxes: passable (agent can push/pull them out of the way)
                    // Different-color boxes: impassable (agent cannot interact with them)
                    Character boxAtNext = state.getBoxes().get(next);
                    if (boxAtNext != null) {
                        Color nextBoxColor = level.getBoxColor(boxAtNext);
                        if (nextBoxColor != agentColor) {
                            continue; // Different color = permanent obstacle
                        }
                        // Same color = passable (can be pushed/pulled aside)
                    }
                    
                    visited.add(next);
                    queue.add(next);
                }
            }
        }

        return false;
    }

    /**
     * Finds a parking spot for a displaced box.
     * Requirements:
     * 1. Not a goal position (for any box type)
     * 2. Not blocking other agents' current paths
     * 3. Preferably in a "wide" area (3+ free neighbors)
     * 4. Reachable from the box's current position
     */
    private Position findParkingSpot(Position boxPos, State state, Level level,
                                     List<BlockingInfo> blockingInfos) {
        // Collect all positions that would be bad parking spots
        Set<Position> badPositions = new HashSet<>();
        
        // All goal positions are bad
        badPositions.addAll(level.getAllBoxGoalPositions());
        for (Position agentGoalPos : level.getAgentGoalPositionMap().values()) {
            badPositions.add(agentGoalPos);
        }

        // BFS from box position to find good parking spots
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distance = new HashMap<>();

        queue.add(boxPos);
        distance.put(boxPos, 0);

        Position bestSpot = null;
        int bestScore = Integer.MIN_VALUE;
        int maxSearch = 100;
        int searched = 0;

        while (!queue.isEmpty() && searched < maxSearch) {
            Position current = queue.poll();
            int dist = distance.get(current);
            searched++;

            // Skip the box's current position
            if (!current.equals(boxPos)) {
                // Check if this is a valid parking spot
                if (!badPositions.contains(current) && 
                    !state.getBoxes().containsKey(current) &&
                    !isAgentPosition(current, state)) {
                    
                    int freeNeighbors = countFreeNeighbors(current, level);
                    
                    // Score: prefer wide areas, penalize distance
                    int score = freeNeighbors * 10 - dist;
                    
                    // Bonus for dead-ends (won't block future paths)
                    if (freeNeighbors == 1) {
                        score += 20;
                    }
                    
                    if (score > bestScore) {
                        bestScore = score;
                        bestSpot = current;
                    }
                }
            }

            // Explore neighbors (through empty spaces only)
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!distance.containsKey(next) && !level.isWall(next) &&
                    !state.getBoxes().containsKey(next) && !isAgentPosition(next, state)) {
                    distance.put(next, dist + 1);
                    queue.add(next);
                }
            }
        }

        return bestSpot;
    }

    private boolean isAgentPosition(Position pos, State state) {
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (state.getAgentPosition(i).equals(pos)) {
                return true;
            }
        }
        return false;
    }

    private int countFreeNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            if (!level.isWall(pos.move(dir))) {
                count++;
            }
        }
        return count;
    }

    private void log(String msg) {
        if (verboseLogging || SearchConfig.isNormal()) {
            System.err.println(msg);
        }
    }
}
