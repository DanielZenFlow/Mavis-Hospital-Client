package mapf.planning.strategy;

import mapf.domain.*;

import java.util.*;

/**
 * Handles deadlock detection and breaking strategies.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Responsibilities:
 * - Deadlock detection
 * - Cycle breaking
 * - Box displacement for deadlock resolution
 * - Path clearing strategies
 */
public class DeadlockBreaker {

    private static final int MAX_DEADLOCK_BREAKING_ATTEMPTS = 5;

    /**
     * Detects if agents are in a deadlock situation.
     */
    public boolean isDeadlocked(List<Action[]> plan, int windowSize) {
        if (plan.size() < windowSize) {
            return false;
        }

        // Check if recent actions are all NOOPs
        int noopCount = 0;
        for (int i = plan.size() - windowSize; i < plan.size(); i++) {
            Action[] jointAction = plan.get(i);
            boolean allNoop = true;
            for (Action action : jointAction) {
                if (action.type != Action.ActionType.NOOP) {
                    allNoop = false;
                    break;
                }
            }
            if (allNoop) {
                noopCount++;
            }
        }

        return noopCount >= windowSize - 1;
    }

    /**
     * Detects cyclic blocking between agents.
     * Simplified version - uses Map of goal positions.
     */
    public List<Integer> detectBlockingCycle(State state, Level level, int numAgents,
            Map<Integer, Position> goalPositions) {
        
        // Build blocking graph
        Map<Integer, Integer> blockedBy = new HashMap<>();

        for (int agentId = 0; agentId < numAgents; agentId++) {
            Position targetPos = goalPositions.get(agentId);
            if (targetPos == null)
                continue;

            Position agentPos = state.getAgentPosition(agentId);

            // Check if another agent blocks the path
            for (Direction dir : Direction.values()) {
                Position nextPos = agentPos.move(dir);

                for (int otherAgent = 0; otherAgent < numAgents; otherAgent++) {
                    if (otherAgent == agentId)
                        continue;

                    if (state.getAgentPosition(otherAgent).equals(nextPos)) {
                        blockedBy.put(agentId, otherAgent);
                        break;
                    }
                }

                if (blockedBy.containsKey(agentId))
                    break;
            }
        }

        // Detect cycle using tortoise and hare
        for (int start = 0; start < numAgents; start++) {
            if (!blockedBy.containsKey(start))
                continue;

            Set<Integer> visited = new HashSet<>();
            List<Integer> path = new ArrayList<>();
            int current = start;

            while (current != -1 && !visited.contains(current)) {
                visited.add(current);
                path.add(current);
                current = blockedBy.getOrDefault(current, -1);
            }

            if (current != -1 && path.contains(current)) {
                // Found cycle
                int cycleStart = path.indexOf(current);
                return path.subList(cycleStart, path.size());
            }
        }

        return null;
    }

    /**
     * Attempts to break a detected cycle.
     */
    public boolean attemptCycleBreaking(List<Action[]> plan, State state, Level level,
            int numAgents, List<Integer> cycle, PathAnalyzer pathAnalyzer,
            ConflictResolver conflictResolver) {
        
        if (cycle == null || cycle.isEmpty()) {
            return false;
        }

        // Try to move one agent out of the cycle
        for (int agentId : cycle) {
            Position agentPos = state.getAgentPosition(agentId);
            Set<Position> cyclePositions = new HashSet<>();
            for (int a : cycle) {
                cyclePositions.add(state.getAgentPosition(a));
            }

            // Find escape direction
            for (Direction dir : Direction.values()) {
                Position escapePos = agentPos.move(dir);

                if (!level.isWall(escapePos) &&
                        !state.getBoxes().containsKey(escapePos) &&
                        !cyclePositions.contains(escapePos)) {

                    Action moveAction = Action.move(dir);
                    if (state.isApplicable(moveAction, agentId, level)) {
                        Action[] jointAction = new Action[numAgents];
                        Arrays.fill(jointAction, Action.noOp());
                        jointAction[agentId] = moveAction;
                        jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                        plan.add(jointAction);
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /**
     * Tries to push a blocking box out of the way.
     */
    public boolean tryPushBoxOutOfWay(List<Action[]> plan, State state, Level level,
            int numAgents, int agentId, Position boxPos, Set<Position> criticalPositions,
            Set<Position> satisfiedGoalPositions, ConflictResolver conflictResolver) {
        
        Position agentPos = state.getAgentPosition(agentId);
        char boxChar = state.getBoxes().get(boxPos);
        Color agentColor = level.getAgentColor(agentId);

        if (level.getBoxColor(boxChar) != agentColor) {
            return false;
        }

        // Find direction agent to box
        Direction agentToBox = getDirection(agentPos, boxPos);
        if (agentToBox == null) {
            return false;
        }

        // Try pushing in various directions
        for (Direction pushDir : Direction.values()) {
            Position newBoxPos = boxPos.move(pushDir);

            if (level.isWall(newBoxPos))
                continue;
            if (state.getBoxes().containsKey(newBoxPos))
                continue;
            if (criticalPositions.contains(newBoxPos))
                continue;
            if (satisfiedGoalPositions.contains(newBoxPos))
                continue;

            // Check if agent can push
            if (agentToBox == pushDir.opposite()) {
                // Agent is behind box, can push
                Action pushAction = Action.push(agentToBox, pushDir);
                if (state.isApplicable(pushAction, agentId, level)) {
                    Action[] jointAction = new Action[numAgents];
                    Arrays.fill(jointAction, Action.noOp());
                    jointAction[agentId] = pushAction;
                    jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                    plan.add(jointAction);
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Finds a safe displacement position for a box.
     */
    public Position findSafeDisplacementPosition(Position boxPos, State state, Level level,
            Set<Position> criticalPositions, Set<Position> satisfiedGoalPositions, int numAgents) {
        
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        queue.add(boxPos);
        visited.add(boxPos);

        Set<Position> agentPositions = new HashSet<>();
        for (int i = 0; i < numAgents; i++) {
            agentPositions.add(state.getAgentPosition(i));
        }

        int maxSearchDistance = 20;
        int distance = 0;

        while (!queue.isEmpty() && distance < maxSearchDistance) {
            int levelSize = queue.size();
            distance++;

            for (int i = 0; i < levelSize; i++) {
                Position current = queue.poll();

                for (Direction dir : Direction.values()) {
                    Position next = current.move(dir);

                    if (visited.contains(next))
                        continue;
                    if (level.isWall(next))
                        continue;

                    visited.add(next);

                    if (!state.getBoxes().containsKey(next) &&
                            !criticalPositions.contains(next) &&
                            !satisfiedGoalPositions.contains(next) &&
                            !agentPositions.contains(next)) {
                        return next;
                    }

                    queue.add(next);
                }
            }
        }

        return null;
    }

    /**
     * Preemptively clears the path for an agent.
     */
    public boolean tryPreemptivePathClearing(List<Action[]> plan, State state, Level level,
            int numAgents, int agentId, List<Position> plannedPath,
            ConflictResolver conflictResolver, PathAnalyzer pathAnalyzer) {
        
        if (plannedPath == null || plannedPath.isEmpty()) {
            return false;
        }

        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);
        Set<Position> pathSet = new HashSet<>(plannedPath);

        // Find first blocking position on path
        for (Position pos : plannedPath) {
            // Check for blocking agent
            for (int otherAgent = 0; otherAgent < numAgents; otherAgent++) {
                if (otherAgent == agentId)
                    continue;

                if (state.getAgentPosition(otherAgent).equals(pos)) {
                    // Try to clear this agent
                    Position parkingPos = pathAnalyzer.findParkingPosition(otherAgent, state, level,
                            numAgents, pathSet, satisfiedGoals);

                    if (parkingPos != null) {
                        List<Action> clearPath = pathAnalyzer.planAgentPath(otherAgent, parkingPos,
                                state, level, numAgents);

                        if (clearPath != null && !clearPath.isEmpty()) {
                            Action[] jointAction = new Action[numAgents];
                            Arrays.fill(jointAction, Action.noOp());
                            jointAction[otherAgent] = clearPath.get(0);
                            jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                            plan.add(jointAction);
                            return true;
                        }
                    }
                }
            }

            // Check for blocking box (that agent can move)
            if (state.getBoxes().containsKey(pos)) {
                char boxChar = state.getBoxes().get(pos);
                Color agentColor = level.getAgentColor(agentId);

                if (level.getBoxColor(boxChar) == agentColor) {
                    return tryPushBoxOutOfWay(plan, state, level, numAgents, agentId, pos,
                            pathSet, satisfiedGoals, conflictResolver);
                }
            }
        }

        return false;
    }

    /**
     * Gets direction from one position to adjacent position.
     */
    private Direction getDirection(Position from, Position to) {
        int drow = to.row - from.row;
        int dcol = to.col - from.col;

        if (drow == -1 && dcol == 0)
            return Direction.N;
        if (drow == 1 && dcol == 0)
            return Direction.S;
        if (drow == 0 && dcol == -1)
            return Direction.W;
        if (drow == 0 && dcol == 1)
            return Direction.E;

        return null;
    }
}
