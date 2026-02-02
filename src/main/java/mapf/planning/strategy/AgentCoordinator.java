package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.coordination.AgentYieldingManager;

import java.util.*;

/**
 * Manages agent coordination including yielding, clearing, and priority handling.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Responsibilities:
 * - Agent yielding management
 * - Idle agent clearing
 * - Blocking agent clearing
 * - Proactive yielding
 * - Priority-based coordination
 */
public class AgentCoordinator {

    // Yielding state
    private final Map<Integer, Boolean> agentYielding = new HashMap<>();
    private final Map<Integer, Integer> yieldingBeneficiary = new HashMap<>();
    private final Map<Integer, Position> yieldingTargetPosition = new HashMap<>();
    private final Map<Integer, Integer> yieldStartStep = new HashMap<>();
    private final Map<Integer, Integer> maxYieldSteps = new HashMap<>();

    // Completed task tracking
    private final Map<Integer, Boolean> agentCompletedTask = new HashMap<>();
    
    // Priority tracking
    private final Map<Integer, Integer> agentPriorities = new HashMap<>();

    // Constants
    private static final int DEFAULT_YIELD_STEPS = 30;
    private static final int MAX_YIELD_WAIT = 50;

    public AgentCoordinator() {
    }

    // ========== Yielding Management ==========

    /**
     * Set agent yielding state.
     */
    public void setAgentYielding(int agentId, boolean yielding, int beneficiary, int currentStep) {
        agentYielding.put(agentId, yielding);
        if (yielding) {
            yieldingBeneficiary.put(agentId, beneficiary);
            yieldStartStep.put(agentId, currentStep);
            maxYieldSteps.put(agentId, DEFAULT_YIELD_STEPS);
        } else {
            yieldingBeneficiary.remove(agentId);
            yieldStartStep.remove(agentId);
            maxYieldSteps.remove(agentId);
            yieldingTargetPosition.remove(agentId);
        }
    }

    /**
     * Check if agent is yielding.
     */
    public boolean isYielding(int agentId) {
        return agentYielding.getOrDefault(agentId, false);
    }

    /**
     * Get beneficiary of yielding agent.
     */
    public int getYieldingBeneficiary(int agentId) {
        return yieldingBeneficiary.getOrDefault(agentId, -1);
    }

    /**
     * Set target position for yielding agent.
     */
    public void setYieldingTargetPosition(int agentId, Position pos) {
        yieldingTargetPosition.put(agentId, pos);
    }

    /**
     * Get target position for yielding agent.
     */
    public Position getYieldingTargetPosition(int agentId) {
        return yieldingTargetPosition.get(agentId);
    }

    /**
     * Check if yield has expired.
     */
    public boolean hasYieldExpired(int agentId, int currentStep) {
        if (!isYielding(agentId)) {
            return false;
        }
        int startStep = yieldStartStep.getOrDefault(agentId, currentStep);
        int maxSteps = maxYieldSteps.getOrDefault(agentId, DEFAULT_YIELD_STEPS);
        return (currentStep - startStep) >= maxSteps;
    }

    /**
     * Update all yielding statuses.
     */
    public void updateYieldingStatuses(int currentStep, int numAgents) {
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (hasYieldExpired(agentId, currentStep)) {
                setAgentYielding(agentId, false, -1, currentStep);
            }
        }
    }

    // ========== Task Completion ==========

    /**
     * Mark agent as having completed their task.
     */
    public void setTaskCompleted(int agentId, boolean completed) {
        agentCompletedTask.put(agentId, completed);
    }

    /**
     * Check if agent has completed task.
     */
    public boolean hasCompletedTask(int agentId) {
        return agentCompletedTask.getOrDefault(agentId, false);
    }

    // ========== Priority Management ==========

    /**
     * Set agent priority.
     */
    public void setAgentPriority(int agentId, int priority) {
        agentPriorities.put(agentId, priority);
    }

    /**
     * Get agent priority.
     */
    public int getAgentPriority(int agentId) {
        return agentPriorities.getOrDefault(agentId, agentId);
    }

    /**
     * Compute dynamic priorities based on topological analysis.
     * Simplified version - uses Map of current goals.
     */
    public void computeDynamicPriorities(State state, Level level, int numAgents,
            TopologicalAnalyzer topoAnalyzer, Map<Integer, Position> currentGoalPositions) {
        
        // TODO: Fix computeTopologicalDepths to return Map<Integer, Integer>
        // Map<Integer, Integer> depths = topoAnalyzer.computeTopologicalDepths(level);

        List<Integer> sortedAgents = new ArrayList<>();
        for (int i = 0; i < numAgents; i++) {
            sortedAgents.add(i);
        }

        // Simplified sorting by agent ID for now
        sortedAgents.sort((a, b) -> {
            Position goalA = currentGoalPositions.get(a);
            Position goalB = currentGoalPositions.get(b);
            if (goalA != null && goalB != null) {
                int distA = manhattan(state.getAgentPosition(a), goalA);
                int distB = manhattan(state.getAgentPosition(b), goalB);
                return Integer.compare(distA, distB);
            }
            
            return Integer.compare(a, b);
        });

        for (int priority = 0; priority < sortedAgents.size(); priority++) {
            agentPriorities.put(sortedAgents.get(priority), priority);
        }
    }

    /**
     * Get execution order based on priorities.
     */
    public List<Integer> getExecutionOrder(int numAgents) {
        List<Integer> order = new ArrayList<>();
        for (int i = 0; i < numAgents; i++) {
            order.add(i);
        }
        order.sort(Comparator.comparingInt(this::getAgentPriority));
        return order;
    }

    // ========== Idle Agent Clearing ==========

    /**
     * Result of a clearing operation, including which agent was cleared.
     */
    public static class ClearingResult {
        public final boolean success;
        public final int clearedAgentId;
        public final Position targetPosition;  // Where the cleared agent should move to
        
        public ClearingResult(boolean success, int clearedAgentId, Position targetPosition) {
            this.success = success;
            this.clearedAgentId = clearedAgentId;
            this.targetPosition = targetPosition;
        }
        
        public static ClearingResult failure() {
            return new ClearingResult(false, -1, null);
        }
        
        public static ClearingResult success(int clearedAgentId, Position targetPosition) {
            return new ClearingResult(true, clearedAgentId, targetPosition);
        }
    }

    /**
     * Try to clear idle agents blocking the path. Returns which agent was cleared.
     */
    public ClearingResult tryIdleAgentClearingWithResult(List<Action[]> plan, State state, Level level,
            int numAgents, int activeAgent, Set<Position> criticalPositions,
            PathAnalyzer pathAnalyzer, ConflictResolver conflictResolver) {
        
        Position activePos = state.getAgentPosition(activeAgent);
        Set<Position> satisfiedGoalPositions = GoalChecker.computeSatisfiedGoalPositions(state, level);

        for (int idleAgent = 0; idleAgent < numAgents; idleAgent++) {
            if (idleAgent == activeAgent)
                continue;
            // Only clear agents that have completed their task (not just any idle agent)
            if (!hasCompletedTask(idleAgent))
                continue;
            // Skip agents already yielding (they're already being handled)
            if (isYielding(idleAgent))
                continue;

            Position idlePos = state.getAgentPosition(idleAgent);

            if (criticalPositions.contains(idlePos)) {
                Position parkingPos = pathAnalyzer.findParkingPosition(idleAgent, state, level,
                        numAgents, criticalPositions, satisfiedGoalPositions);

                if (parkingPos != null) {
                    List<Action> path = pathAnalyzer.planAgentPath(idleAgent, parkingPos, state, level, numAgents);

                    if (path != null && !path.isEmpty()) {
                        Action clearAction = path.get(0);
                        Action[] jointAction = new Action[numAgents];
                        Arrays.fill(jointAction, Action.noOp());
                        jointAction[idleAgent] = clearAction;
                        jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                        plan.add(jointAction);
                        
                        // Store the target position for continued movement
                        setYieldingTargetPosition(idleAgent, parkingPos);
                        
                        return ClearingResult.success(idleAgent, parkingPos);
                    }
                }
            }
        }

        return ClearingResult.failure();
    }

    /**
     * Try to clear idle agents blocking the path.
     */
    public boolean tryIdleAgentClearing(List<Action[]> plan, State state, Level level,
            int numAgents, int activeAgent, Set<Position> criticalPositions,
            PathAnalyzer pathAnalyzer, ConflictResolver conflictResolver) {
        
        Position activePos = state.getAgentPosition(activeAgent);
        Set<Position> satisfiedGoalPositions = GoalChecker.computeSatisfiedGoalPositions(state, level);

        for (int idleAgent = 0; idleAgent < numAgents; idleAgent++) {
            if (idleAgent == activeAgent)
                continue;
            if (!hasCompletedTask(idleAgent) && !isYielding(idleAgent))
                continue;

            Position idlePos = state.getAgentPosition(idleAgent);

            if (criticalPositions.contains(idlePos)) {
                Position parkingPos = pathAnalyzer.findParkingPosition(idleAgent, state, level,
                        numAgents, criticalPositions, satisfiedGoalPositions);

                if (parkingPos != null) {
                    List<Action> path = pathAnalyzer.planAgentPath(idleAgent, parkingPos, state, level, numAgents);

                    if (path != null && !path.isEmpty()) {
                        Action clearAction = path.get(0);
                        Action[] jointAction = new Action[numAgents];
                        Arrays.fill(jointAction, Action.noOp());
                        jointAction[idleAgent] = clearAction;
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
     * Clear a blocking agent from a specific position.
     */
    public boolean clearBlockingAgent(List<Action[]> plan, State state, Level level,
            int numAgents, int blockingAgent, Set<Position> criticalPositions,
            PathAnalyzer pathAnalyzer, ConflictResolver conflictResolver, int currentStep) {
        
        Position blockingPos = state.getAgentPosition(blockingAgent);
        Set<Position> satisfiedGoalPositions = GoalChecker.computeSatisfiedGoalPositions(state, level);

        Position parkingPos = pathAnalyzer.findParkingPosition(blockingAgent, state, level,
                numAgents, criticalPositions, satisfiedGoalPositions);

        if (parkingPos != null) {
            List<Action> path = pathAnalyzer.planAgentPath(blockingAgent, parkingPos, state, level, numAgents);

            if (path != null && !path.isEmpty()) {
                Action clearAction = path.get(0);
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[blockingAgent] = clearAction;
                jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                plan.add(jointAction);
                return true;
            }
        }

        // Fallback: try any valid move
        for (Direction dir : Direction.values()) {
            Position newPos = blockingPos.move(dir);

            if (!level.isWall(newPos) &&
                    !state.getBoxes().containsKey(newPos) &&
                    !criticalPositions.contains(newPos) &&
                    !satisfiedGoalPositions.contains(newPos)) {

                boolean occupied = false;
                for (int i = 0; i < numAgents; i++) {
                    if (state.getAgentPosition(i).equals(newPos)) {
                        occupied = true;
                        break;
                    }
                }

                if (!occupied) {
                    Action moveAction = Action.move(dir);
                    if (state.isApplicable(moveAction, blockingAgent, level)) {
                        Action[] jointAction = new Action[numAgents];
                        Arrays.fill(jointAction, Action.noOp());
                        jointAction[blockingAgent] = moveAction;
                        jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                        plan.add(jointAction);
                        return true;
                    }
                }
            }
        }

        return false;
    }

    // ========== Proactive Yielding ==========

    /**
     * Check if agent should proactively yield.
     * Simplified version - uses Map of goal positions.
     */
    public boolean shouldProactivelyYield(int agentId, State state, Level level,
            int numAgents, Map<Integer, Position> goalPositions, PathAnalyzer pathAnalyzer) {
        
        if (hasCompletedTask(agentId))
            return false;
        if (isYielding(agentId))
            return false;

        Position agentPos = state.getAgentPosition(agentId);

        for (int otherAgent = 0; otherAgent < numAgents; otherAgent++) {
            if (otherAgent == agentId)
                continue;
            if (getAgentPriority(otherAgent) >= getAgentPriority(agentId))
                continue;

            Position targetPos = goalPositions.get(otherAgent);
            if (targetPos == null)
                continue;

            Position otherPos = state.getAgentPosition(otherAgent);
            List<Position> path = pathAnalyzer.findPathIgnoringDynamicObstacles(otherPos, targetPos, level);

            if (path != null && path.contains(agentPos)) {
                return true;
            }
        }

        return false;
    }

    /**
     * Find best yield position for an agent.
     */
    public Position findBestYieldPosition(int agentId, State state, Level level,
            int numAgents, int higherPriorityAgent, PathAnalyzer pathAnalyzer,
            Set<Position> criticalPositions) {
        
        Position agentPos = state.getAgentPosition(agentId);
        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);

        // Find corridor exit if in corridor
        if (pathAnalyzer.isInCorridor(agentPos, level)) {
            Position exitPos = findCorridorExit(agentPos, level, pathAnalyzer);
            if (exitPos != null && isValidYieldPosition(exitPos, state, level, numAgents,
                    criticalPositions, satisfiedGoals)) {
                return exitPos;
            }
        }

        // Otherwise find nearest valid parking position
        return pathAnalyzer.findParkingPosition(agentId, state, level, numAgents,
                criticalPositions, satisfiedGoals);
    }

    /**
     * Find corridor exit position.
     */
    public Position findCorridorExit(Position start, Level level, PathAnalyzer pathAnalyzer) {
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (!pathAnalyzer.isInCorridor(current, level)) {
                return current;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }

        return null;
    }

    /**
     * Force yielding agent to move toward target.
     */
    public boolean forceYieldingAgentToMove(List<Action[]> plan, State state, Level level,
            int numAgents, int agentId, PathAnalyzer pathAnalyzer, ConflictResolver conflictResolver) {
        
        Position targetPos = yieldingTargetPosition.get(agentId);
        Position currentPos = state.getAgentPosition(agentId);
        
        // If no target set or already at target, try to find a new parking position
        if (targetPos == null || currentPos.equals(targetPos)) {
            Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);
            Set<Position> criticalPositions = new HashSet<>();
            // Avoid blocking any agent's path
            for (int i = 0; i < numAgents; i++) {
                if (i == agentId) continue;
                Position agentGoal = GoalChecker.findAgentGoalPosition(i, level);
                if (agentGoal != null) {
                    criticalPositions.add(agentGoal);
                }
            }
            
            targetPos = pathAnalyzer.findParkingPosition(agentId, state, level,
                    numAgents, criticalPositions, satisfiedGoals);
            
            if (targetPos != null && !targetPos.equals(currentPos)) {
                yieldingTargetPosition.put(agentId, targetPos);
            } else {
                // No valid parking position, clear this agent's yielding state
                return false;
            }
        }

        List<Action> path = pathAnalyzer.planAgentPath(agentId, targetPos, state, level, numAgents);

        if (path != null && !path.isEmpty()) {
            Action moveAction = path.get(0);
            Action[] jointAction = new Action[numAgents];
            Arrays.fill(jointAction, Action.noOp());
            jointAction[agentId] = moveAction;
            jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
            plan.add(jointAction);
            return true;
        }

        return false;
    }

    private boolean isValidYieldPosition(Position pos, State state, Level level,
            int numAgents, Set<Position> criticalPositions, Set<Position> satisfiedGoals) {
        
        if (level.isWall(pos))
            return false;
        if (state.getBoxes().containsKey(pos))
            return false;
        if (criticalPositions.contains(pos))
            return false;
        if (satisfiedGoals.contains(pos))
            return false;

        for (int i = 0; i < numAgents; i++) {
            if (state.getAgentPosition(i).equals(pos)) {
                return false;
            }
        }

        return true;
    }

    // ========== Utility ==========
    
    /**
     * Clears the yielding state for all agents that were yielding for a specific beneficiary.
     */
    public void clearYieldingForBeneficiary(int beneficiaryId, AgentYieldingManager yieldingManager) {
        Map<Integer, Integer> yieldingAgents = yieldingManager.getYieldingAgents();
        for (Map.Entry<Integer, Integer> entry : yieldingAgents.entrySet()) {
            if (entry.getValue() == beneficiaryId) {
                int agentId = entry.getKey();
                yieldingManager.clearYielding(agentId);
                if (SearchConfig.isVerbose()) {
                    System.err.println("[YIELD] Agent " + agentId + " RELEASED from yielding (beneficiary Agent " + beneficiaryId + " completed)");
                }
            }
        }
    }

    private int manhattan(Position a, Position b) {
        return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
    }

    /**
     * Reset all coordinator state.
     */
    public void reset() {
        agentYielding.clear();
        yieldingBeneficiary.clear();
        yieldingTargetPosition.clear();
        yieldStartStep.clear();
        maxYieldSteps.clear();
        agentCompletedTask.clear();
        agentPriorities.clear();
    }
}
