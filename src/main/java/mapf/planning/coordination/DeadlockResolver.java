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
        public final Position boxPosition;  // Current box position
        public final char boxType;          // Box type
        public final Position tempPosition; // Temporary destination
        public final String reason;         // Why this displacement

        public DisplacementPlan(int agentId, Position boxPos, char boxType, 
                               Position tempPos, String reason) {
            this.agentId = agentId;
            this.boxPosition = boxPos;
            this.boxType = boxType;
            this.tempPosition = tempPos;
            this.reason = reason;
        }

        @Override
        public String toString() {
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
        return analyzeBlocking(state, level, blockedAgents, Collections.emptySet());
    }
    
    /**
     * Analyzes the current state to find blocking relationships.
     * Immovable boxes are treated as walls and skipped.
     */
    public List<BlockingInfo> analyzeBlocking(State state, Level level, List<Integer> blockedAgents,
                                              Set<Position> immovableBoxes) {
        List<BlockingInfo> blockingInfos = new ArrayList<>();

        for (int agentId : blockedAgents) {
            Position agentPos = state.getAgentPosition(agentId);
            
            // Find this agent's goal (box goal or agent goal)
            Position goalPos = findAgentCurrentGoal(agentId, state, level);
            if (goalPos == null) continue;

            // Find what's blocking the path (immovable boxes treated as walls)
            BlockingInfo blocking = findBlockingObstacle(agentId, agentPos, goalPos, state, level, immovableBoxes);
            if (blocking != null) {
                blockingInfos.add(blocking);
            }
        }

        return blockingInfos;
    }

    /**
     * Finds the current goal for an agent (either a box goal or agent goal).
     */
    private Position findAgentCurrentGoal(int agentId, State state, Level level) {
        Color agentColor = level.getAgentColor(agentId);

        // First, check for unsatisfied box goals this agent can handle
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Color boxColor = level.getBoxColor(goalType);
                    if (boxColor == agentColor) {
                        char currentBox = state.getBoxAt(new Position(row, col));
                        if (currentBox != goalType) {
                            // This goal is unsatisfied and this agent can handle it
                            return new Position(row, col);
                        }
                    }
                }
            }
        }

        // Then check agent goal
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    Position goalPos = new Position(row, col);
                    if (!state.getAgentPosition(agentId).equals(goalPos)) {
                        return goalPos;
                    }
                }
            }
        }

        return null;
    }

    /**
     * Finds what's blocking an agent's path to a goal.
     * Uses BFS to find the first movable obstacle on the shortest path.
     * Immovable boxes are treated as walls and skipped.
     */
    private BlockingInfo findBlockingObstacle(int agentId, Position start, Position goal, 
                                              State state, Level level, Set<Position> immovableBoxes) {
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> cameFrom = new HashMap<>();
        
        queue.add(start);
        cameFrom.put(start, null);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (current.equals(goal)) {
                // Path found without obstacles (shouldn't happen if agent is blocked)
                return null;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (cameFrom.containsKey(next)) continue;
                if (level.isWall(next)) continue;
                
                // Treat immovable boxes as walls - skip them
                if (immovableBoxes.contains(next)) continue;

                // Check if blocked by movable box
                Character box = state.getBoxes().get(next);
                if (box != null) {
                    return new BlockingInfo(agentId, goal, next, true, box, -1);
                }

                // Check if blocked by another agent
                for (int otherId = 0; otherId < state.getNumAgents(); otherId++) {
                    if (otherId != agentId && state.getAgentPosition(otherId).equals(next)) {
                        return new BlockingInfo(agentId, goal, next, false, '\0', otherId);
                    }
                }

                // Position is free
                cameFrom.put(next, current);
                queue.add(next);
            }
        }

        // No path found at all
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
        // Sort by: prefer boxes that block the most agents
        Map<Position, Integer> blockingCount = new HashMap<>();
        for (BlockingInfo info : blockingInfos) {
            if (info.isBlockedByBox) {
                blockingCount.merge(info.blockingPosition, 1, Integer::sum);
            }
        }

        // Try each blocking box
        List<Map.Entry<Position, Integer>> sortedBoxes = new ArrayList<>(blockingCount.entrySet());
        sortedBoxes.sort((a, b) -> Integer.compare(b.getValue(), a.getValue())); // Most blocking first

        for (Map.Entry<Position, Integer> entry : sortedBoxes) {
            Position boxPos = entry.getKey();
            char boxType = state.getBoxAt(boxPos);
            
            if (boxType == '\0') continue;

            // Skip if already tried this displacement
            String historyKey = boxType + "@" + boxPos;
            if (displacementHistory.contains(historyKey)) {
                log("[DEADLOCK] Skipping already-attempted displacement: " + historyKey);
                continue;
            }

            // Find an agent that can move this box
            Color boxColor = level.getBoxColor(boxType);
            int pushingAgent = -1;
            
            for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
                if (level.getAgentColor(agentId) == boxColor) {
                    // Check if this agent can reach the box
                    Position agentPos = state.getAgentPosition(agentId);
                    if (canReachBox(agentPos, boxPos, state, level)) {
                        pushingAgent = agentId;
                        break;
                    }
                }
            }

            if (pushingAgent == -1) {
                log("[DEADLOCK] No agent can reach blocking box " + boxType + " at " + boxPos);
                continue;
            }

            // Find a temporary parking spot for this box
            Position parkingSpot = findParkingSpot(boxPos, state, level, blockingInfos);
            
            if (parkingSpot != null) {
                String reason = "Blocking " + entry.getValue() + " agent(s)";
                return new DisplacementPlan(pushingAgent, boxPos, boxType, parkingSpot, reason);
            }
        }

        return null;
    }

    /**
     * Checks if an agent can reach a box position.
     */
    private boolean canReachBox(Position agentPos, Position boxPos, State state, Level level) {
        // Simple BFS reachability check
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
                    // Can pass through if not blocked by another agent
                    // (boxes might move, so we don't consider them as permanent obstacles)
                    boolean agentBlocking = false;
                    for (int i = 0; i < state.getNumAgents(); i++) {
                        if (state.getAgentPosition(i).equals(next)) {
                            agentBlocking = true;
                            break;
                        }
                    }
                    
                    if (!agentBlocking && !state.getBoxes().containsKey(next)) {
                        visited.add(next);
                        queue.add(next);
                    }
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
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getBoxGoal(row, col) != '\0' || level.getAgentGoal(row, col) >= 0) {
                    badPositions.add(new Position(row, col));
                }
            }
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
