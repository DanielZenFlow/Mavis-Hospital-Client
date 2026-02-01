package mapf.planning.coordination;

import mapf.domain.*;
import mapf.planning.SearchConfig;

import java.util.*;

/**
 * Manages agent yielding behavior for multi-agent pathfinding.
 * 
 * Agent Yielding is needed when:
 * 1. An agent is blocking another agent's critical path (reactive yielding)
 * 2. An agent has completed its task but is blocking others (proactive yielding)
 * 
 * Key concepts:
 * - "Safe Position": A position where the agent doesn't block any corridor or critical path
 * - "Critical Path": The shortest path from an agent to its goal
 * - "Corridor": A position with only 2 free neighbors (linear passage)
 * 
 * This class follows Single Responsibility Principle - only handles yielding logic.
 */
public class AgentYieldingManager {

    /**
     * Tracks agents that are currently yielding.
     * Maps yieldingAgentId -> beneficiaryAgentId (who they're yielding for)
     */
    private final Map<Integer, Integer> yieldingAgents = new HashMap<>();

    /**
     * Tracks agents that have completed their assigned tasks.
     * These agents should proactively move to safe positions.
     */
    private final Set<Integer> completedAgents = new HashSet<>();

    /**
     * Cache for safe positions per agent.
     * Key: agentId, Value: last known safe position for that agent
     */
    private final Map<Integer, Position> agentSafePositions = new HashMap<>();

    // Logging control
    private boolean verboseLogging = false;

    public AgentYieldingManager() {
    }

    public void setVerboseLogging(boolean verbose) {
        this.verboseLogging = verbose;
    }

    // ========== State Management ==========

    /**
     * Resets all yielding state. Call at the start of a new search.
     */
    public void reset() {
        yieldingAgents.clear();
        completedAgents.clear();
        agentSafePositions.clear();
    }

    /**
     * Marks an agent as yielding to another agent.
     */
    public void setYielding(int yieldingAgentId, int beneficiaryAgentId) {
        yieldingAgents.put(yieldingAgentId, beneficiaryAgentId);
        log("[YIELD] Agent " + yieldingAgentId + " now yielding for Agent " + beneficiaryAgentId);
    }

    /**
     * Removes an agent from yielding state.
     */
    public void clearYielding(int agentId) {
        if (yieldingAgents.containsKey(agentId)) {
            log("[YIELD] Agent " + agentId + " no longer yielding");
            yieldingAgents.remove(agentId);
        }
    }

    /**
     * Clears all yielding agents (e.g., on timeout).
     */
    public void clearAllYielding() {
        if (!yieldingAgents.isEmpty()) {
            log("[YIELD] Clearing all " + yieldingAgents.size() + " yielding agents");
            yieldingAgents.clear();
        }
    }

    /**
     * Marks an agent as having completed its assigned task.
     */
    public void markTaskCompleted(int agentId) {
        completedAgents.add(agentId);
        log("[YIELD] Agent " + agentId + " marked as task-completed");
    }

    /**
     * Checks if an agent is currently yielding.
     */
    public boolean isYielding(int agentId) {
        return yieldingAgents.containsKey(agentId);
    }

    /**
     * Gets the beneficiary of a yielding agent.
     */
    public Integer getBeneficiary(int yieldingAgentId) {
        return yieldingAgents.get(yieldingAgentId);
    }

    /**
     * Gets all yielding agents and their beneficiaries.
     */
    public Map<Integer, Integer> getYieldingAgents() {
        return new HashMap<>(yieldingAgents);
    }

    /**
     * Checks if there are any yielding agents.
     */
    public boolean hasYieldingAgents() {
        return !yieldingAgents.isEmpty();
    }

    /**
     * Checks if an agent has completed its task.
     */
    public boolean hasCompletedTask(int agentId) {
        return completedAgents.contains(agentId);
    }

    // ========== Blocking Detection ==========

    /**
     * Checks if an agent is blocking another agent's critical path.
     * 
     * @param agentId Agent to check
     * @param state   Current state
     * @param level   Level information
     * @return The ID of the blocked agent, or -1 if not blocking anyone
     */
    public int isBlockingAnyAgent(int agentId, State state, Level level) {
        Position myPos = state.getAgentPosition(agentId);
        int numAgents = state.getNumAgents();

        for (int otherId = 0; otherId < numAgents; otherId++) {
            if (otherId == agentId)
                continue;

            Position otherPos = state.getAgentPosition(otherId);
            Position otherGoal = findAgentGoalPosition(otherId, level);

            if (otherGoal == null)
                continue;

            // Check if my position is on their critical path
            Set<Position> criticalPath = findCriticalPath(otherPos, otherGoal, level, state);
            if (criticalPath.contains(myPos)) {
                return otherId;
            }
        }

        return -1;
    }

    /**
     * Checks if a specific agent is blocking a specific other agent.
     */
    public boolean isBlockingAgent(int potentialBlocker, int blockedAgent, State state, Level level) {
        Position blockerPos = state.getAgentPosition(potentialBlocker);
        Position blockedPos = state.getAgentPosition(blockedAgent);
        Position blockedGoal = findAgentGoalPosition(blockedAgent, level);

        if (blockedGoal == null)
            return false;

        Set<Position> criticalPath = findCriticalPath(blockedPos, blockedGoal, level, state);
        return criticalPath.contains(blockerPos);
    }

    /**
     * Finds agents that a completed agent is blocking.
     * This is used for proactive yielding after task completion.
     */
    public List<Integer> findBlockedAgents(int completedAgentId, State state, Level level) {
        List<Integer> blocked = new ArrayList<>();
        Position myPos = state.getAgentPosition(completedAgentId);
        int numAgents = state.getNumAgents();

        for (int otherId = 0; otherId < numAgents; otherId++) {
            if (otherId == completedAgentId)
                continue;
            if (completedAgents.contains(otherId))
                continue; // Skip other completed agents

            Position otherPos = state.getAgentPosition(otherId);
            Position otherGoal = findAgentGoalPosition(otherId, level);

            if (otherGoal == null)
                continue;

            Set<Position> criticalPath = findCriticalPath(otherPos, otherGoal, level, state);
            if (criticalPath.contains(myPos)) {
                blocked.add(otherId);
            }
        }

        return blocked;
    }

    // ========== Safe Position Finding ==========

    /**
     * Finds a safe position where the agent won't block any corridors or critical
     * paths.
     * 
     * A position is "safe" if:
     * 1. It's not in a corridor (has 3+ free neighbors)
     * 2. It doesn't block any other agent's critical path
     * 3. It's reachable from the current position
     * 
     * @param agentId Agent that needs to move
     * @param state   Current state
     * @param level   Level information
     * @return The nearest safe position, or null if none found
     */
    public Position findNearestSafePosition(int agentId, State state, Level level) {
        Position currentPos = state.getAgentPosition(agentId);

        // BFS to find nearest safe position
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();

        queue.add(currentPos);
        visited.add(currentPos);

        int maxSearchDistance = Math.max(level.getRows(), level.getCols()) * 2;
        int searched = 0;

        while (!queue.isEmpty() && searched < maxSearchDistance * 10) {
            Position pos = queue.poll();
            searched++;

            // Skip current position
            if (!pos.equals(currentPos) && isSafePosition(pos, agentId, state, level)) {
                log("[YIELD] Found safe position for Agent " + agentId + ": " + pos);
                agentSafePositions.put(agentId, pos);
                return pos;
            }

            // Explore neighbors
            for (Direction dir : Direction.values()) {
                Position next = pos.move(dir);

                if (!visited.contains(next) && !level.isWall(next)) {
                    // Don't path through other agents (they might move)
                    boolean occupiedByAgent = false;
                    for (int i = 0; i < state.getNumAgents(); i++) {
                        if (i != agentId && state.getAgentPosition(i).equals(next)) {
                            occupiedByAgent = true;
                            break;
                        }
                    }

                    if (!occupiedByAgent) {
                        visited.add(next);
                        queue.add(next);
                    }
                }
            }
        }

        log("[YIELD] No safe position found for Agent " + agentId);
        return null;
    }

    /**
     * Checks if a position is "safe" for parking.
     * Safe = not in corridor + not blocking anyone's critical path
     */
    public boolean isSafePosition(Position pos, int agentId, State state, Level level) {
        // Check 1: Not a wall
        if (level.isWall(pos))
            return false;

        // Check 2: Not occupied by a box
        if (state.getBoxes().containsKey(pos))
            return false;

        // Check 3: Not in a corridor (has at least 3 free neighbors)
        int freeNeighbors = countFreeNeighbors(pos, level);
        if (freeNeighbors < 3) {
            return false; // In a corridor or dead-end
        }

        // Check 4: Not blocking any other agent's critical path
        int numAgents = state.getNumAgents();
        for (int otherId = 0; otherId < numAgents; otherId++) {
            if (otherId == agentId)
                continue;

            Position otherPos = state.getAgentPosition(otherId);
            Position otherGoal = findAgentGoalPosition(otherId, level);

            if (otherGoal == null)
                continue;

            // Would this position block the other agent?
            Set<Position> criticalPath = findCriticalPath(otherPos, otherGoal, level, state);
            if (criticalPath.contains(pos)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Counts free (non-wall) neighbors of a position.
     */
    private int countFreeNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (!level.isWall(neighbor)) {
                count++;
            }
        }
        return count;
    }

    // ========== Movement Planning ==========

    /**
     * Plans a path for an agent to move to a target position.
     * Returns a list of actions, or empty list if no path found.
     */
    public List<Action> planPathToPosition(int agentId, Position target, State state, Level level) {
        Position start = state.getAgentPosition(agentId);

        if (start.equals(target)) {
            return Collections.emptyList();
        }

        // BFS to find path
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> cameFrom = new HashMap<>();
        Map<Position, Direction> directionTo = new HashMap<>();

        queue.add(start);
        cameFrom.put(start, null);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (current.equals(target)) {
                // Reconstruct path as actions
                List<Action> actions = new ArrayList<>();
                Position pos = target;
                List<Direction> directions = new ArrayList<>();

                while (cameFrom.get(pos) != null) {
                    directions.add(directionTo.get(pos));
                    pos = cameFrom.get(pos);
                }

                Collections.reverse(directions);
                for (Direction dir : directions) {
                    actions.add(Action.move(dir));
                }

                return actions;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!cameFrom.containsKey(next) && !level.isWall(next)) {
                    // Check not occupied by box
                    if (!state.getBoxes().containsKey(next)) {
                        // Check not occupied by other agent
                        boolean agentOccupied = false;
                        for (int i = 0; i < state.getNumAgents(); i++) {
                            if (i != agentId && state.getAgentPosition(i).equals(next)) {
                                agentOccupied = true;
                                break;
                            }
                        }

                        if (!agentOccupied) {
                            cameFrom.put(next, current);
                            directionTo.put(next, dir);
                            queue.add(next);
                        }
                    }
                }
            }
        }

        return Collections.emptyList(); // No path found
    }

    /**
     * Generates a single move action to get closer to a target, avoiding the
     * beneficiary's goal direction.
     * This is used when full path planning isn't possible.
     */
    public Action generateYieldMove(int yieldingAgentId, int beneficiaryId, State state, Level level) {
        Position currentPos = state.getAgentPosition(yieldingAgentId);
        Position beneficiaryGoal = findAgentGoalPosition(beneficiaryId, level);

        if (beneficiaryGoal == null) {
            return null;
        }

        // Sort directions: prefer moving AWAY from beneficiary's goal
        List<Direction> sortedDirs = new ArrayList<>(Arrays.asList(Direction.values()));
        sortedDirs.sort((d1, d2) -> {
            Position p1 = currentPos.move(d1);
            Position p2 = currentPos.move(d2);
            int dist1 = p1.manhattanDistance(beneficiaryGoal);
            int dist2 = p2.manhattanDistance(beneficiaryGoal);
            return Integer.compare(dist2, dist1); // Descending - farther is better
        });

        for (Direction dir : sortedDirs) {
            Position newPos = currentPos.move(dir);

            // Skip walls
            if (level.isWall(newPos))
                continue;

            // Skip if occupied by another agent
            boolean agentOccupied = false;
            for (int i = 0; i < state.getNumAgents(); i++) {
                if (state.getAgentPosition(i).equals(newPos)) {
                    agentOccupied = true;
                    break;
                }
            }
            if (agentOccupied)
                continue;

            // Skip if occupied by box
            if (state.getBoxes().containsKey(newPos))
                continue;

            // Found a valid move
            Action moveAction = Action.move(dir);
            if (state.isApplicable(moveAction, yieldingAgentId, level)) {
                return moveAction;
            }
        }

        return null; // No valid move
    }

    /**
     * Generates a push action to clear a blocking box, if possible.
     */
    public Action generatePushToYield(int agentId, Direction moveDir, State state, Level level) {
        Position currentPos = state.getAgentPosition(agentId);
        Position boxPos = currentPos.move(moveDir);

        Character boxChar = state.getBoxes().get(boxPos);
        if (boxChar == null)
            return null;

        Color agentColor = level.getAgentColor(agentId);
        Color boxColor = level.getBoxColor(boxChar);

        if (agentColor != boxColor)
            return null;

        // Find a direction to push the box
        for (Direction pushDir : Direction.values()) {
            Position boxDest = boxPos.move(pushDir);

            if (!level.isWall(boxDest) &&
                    !state.getBoxes().containsKey(boxDest) &&
                    !isPositionOccupiedByAgent(boxDest, state)) {

                Action pushAction = Action.push(moveDir, pushDir);
                if (state.isApplicable(pushAction, agentId, level)) {
                    return pushAction;
                }
            }
        }

        return null;
    }

    // ========== Helper Methods ==========

    /**
     * Finds the critical path from start to goal using BFS.
     * The critical path is ONE shortest path (there may be others).
     */
    private Set<Position> findCriticalPath(Position start, Position goal, Level level, State state) {
        Set<Position> critical = new HashSet<>();

        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> cameFrom = new HashMap<>();

        queue.add(start);
        cameFrom.put(start, null);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (current.equals(goal)) {
                // Trace back
                Position pos = goal;
                while (pos != null) {
                    critical.add(pos);
                    pos = cameFrom.get(pos);
                }
                return critical;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!cameFrom.containsKey(next) && !level.isWall(next)) {
                    cameFrom.put(next, current);
                    queue.add(next);
                }
            }
        }

        return critical;
    }

    /**
     * Finds an agent's goal position.
     */
    private Position findAgentGoalPosition(int agentId, Level level) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    return new Position(row, col);
                }
            }
        }
        return null;
    }

    /**
     * Checks if a position is occupied by any agent.
     */
    private boolean isPositionOccupiedByAgent(Position pos, State state) {
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (state.getAgentPosition(i).equals(pos)) {
                return true;
            }
        }
        return false;
    }

    private void log(String msg) {
        if (verboseLogging || SearchConfig.isNormal()) {
            System.err.println(msg);
        }
    }
}
