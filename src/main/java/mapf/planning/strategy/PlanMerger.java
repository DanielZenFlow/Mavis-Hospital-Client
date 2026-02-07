package mapf.planning.strategy;

import mapf.domain.*;

import java.util.*;

/**
 * Handles plan merging and joint action creation.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Responsibilities:
 * - createJointActionWithMerging: Create joint actions with plan merging
 * - applyJointAction: Apply joint actions and update state
 * - addOtherAgentMoves: Add compatible moves from other agents
 */
public class PlanMerger {

    private final Map<Integer, List<Action>> storedPlans;
    private final Map<Integer, Integer> planIndexes;
    private final Map<Integer, Integer> planUsageAtStep;
    
    public PlanMerger() {
        this.storedPlans = new HashMap<>();
        this.planIndexes = new HashMap<>();
        this.planUsageAtStep = new HashMap<>();
    }

    /**
     * Store a plan for an agent.
     */
    public void storePlan(int agentId, List<Action> plan, int currentStep) {
        storedPlans.put(agentId, plan);
        planIndexes.put(agentId, 0);
        planUsageAtStep.put(agentId, currentStep);
    }

    /**
     * Get stored plan for an agent.
     */
    public List<Action> getStoredPlan(int agentId) {
        return storedPlans.get(agentId);
    }

    /**
     * Check if agent has a stored plan.
     */
    public boolean hasStoredPlan(int agentId) {
        return storedPlans.containsKey(agentId);
    }

    /**
     * Get current plan index for an agent.
     */
    public int getPlanIndex(int agentId) {
        return planIndexes.getOrDefault(agentId, 0);
    }

    /**
     * Set plan index for an agent.
     */
    public void setPlanIndex(int agentId, int index) {
        planIndexes.put(agentId, index);
    }

    /**
     * Invalidate an agent's stored plan.
     */
    public void invalidatePlan(int agentId) {
        storedPlans.remove(agentId);
        planIndexes.remove(agentId);
        planUsageAtStep.remove(agentId);
    }

    /**
     * Check if plan has remaining actions.
     */
    public boolean hasPlanRemaining(int agentId) {
        List<Action> plan = storedPlans.get(agentId);
        int index = planIndexes.getOrDefault(agentId, 0);
        return plan != null && index < plan.size();
    }

    /**
     * Get next action from stored plan.
     */
    public Action getNextAction(int agentId) {
        List<Action> plan = storedPlans.get(agentId);
        int index = planIndexes.getOrDefault(agentId, 0);
        if (plan != null && index < plan.size()) {
            return plan.get(index);
        }
        return Action.noOp();
    }

    /**
     * Advance plan index for an agent.
     */
    public void advancePlanIndex(int agentId) {
        int index = planIndexes.getOrDefault(agentId, 0);
        planIndexes.put(agentId, index + 1);
    }

    /**
     * Creates a joint action with plan merging from other agents.
     */
    public Action[] createJointActionWithMerging(int primaryAgent, Action primaryAction,
            State state, Level level, int numAgents, boolean isAgentGoalPhase) {
        
        Action[] jointAction = new Action[numAgents];
        Arrays.fill(jointAction, Action.noOp());
        jointAction[primaryAgent] = primaryAction;

        if (numAgents > 1) {
            addOtherAgentMoves(jointAction, primaryAgent, state, level, numAgents, isAgentGoalPhase);
        }

        return jointAction;
    }

    /**
     * Adds moves from other agents to the joint action.
     */
    public void addOtherAgentMoves(Action[] jointAction, int primaryAgent, State state,
            Level level, int numAgents, boolean isAgentGoalPhase) {
        
        Set<Position> reservedPositions = new HashSet<>();
        Set<Position> reservedBoxPositions = new HashSet<>();
        Set<Position> satisfiedGoalPositions = GoalChecker.computeSatisfiedGoalPositions(state, level);

        // Reserve positions from primary agent's action
        Position primaryPos = state.getAgentPosition(primaryAgent);
        reservedPositions.add(primaryPos);

        Action primaryAction = jointAction[primaryAgent];
        if (primaryAction.type == Action.ActionType.MOVE) {
            Position newPos = primaryPos.move(primaryAction.agentDir);
            reservedPositions.add(newPos);
        } else if (primaryAction.type == Action.ActionType.PUSH) {
            Position boxPos = primaryPos.move(primaryAction.agentDir);
            Position newBoxPos = boxPos.move(primaryAction.boxDir);
            reservedPositions.add(boxPos);
            reservedBoxPositions.add(newBoxPos);
        } else if (primaryAction.type == Action.ActionType.PULL) {
            Position boxPos = primaryPos.move(primaryAction.boxDir.opposite());
            Position newAgentPos = primaryPos.move(primaryAction.agentDir);
            reservedPositions.add(newAgentPos);
            reservedBoxPositions.add(primaryPos);
        }

        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgent)
                continue;

            Action storedAction = getNextActionIfApplicable(agentId, state, level,
                    reservedPositions, reservedBoxPositions, satisfiedGoalPositions, isAgentGoalPhase);

            if (storedAction != null) {
                jointAction[agentId] = storedAction;
                updateReservedPositions(agentId, storedAction, state, reservedPositions, reservedBoxPositions);
            }
        }
    }

    private Action getNextActionIfApplicable(int agentId, State state, Level level,
            Set<Position> reservedPositions, Set<Position> reservedBoxPositions,
            Set<Position> satisfiedGoalPositions, boolean isAgentGoalPhase) {
        
        List<Action> plan = storedPlans.get(agentId);
        int index = planIndexes.getOrDefault(agentId, 0);

        if (plan == null || index >= plan.size()) {
            return null;
        }

        Action action = plan.get(index);
        Position agentPos = state.getAgentPosition(agentId);

        // Check if action is applicable
        if (!state.isApplicable(action, agentId, level)) {
            return null;
        }

        // Check for conflicts with reserved positions
        if (action.type == Action.ActionType.MOVE) {
            Position newPos = agentPos.move(action.agentDir);
            if (reservedPositions.contains(newPos) || reservedBoxPositions.contains(newPos)) {
                return null;
            }
        } else if (action.type == Action.ActionType.PUSH) {
            Position boxPos = agentPos.move(action.agentDir);
            Position newBoxPos = boxPos.move(action.boxDir);
            if (reservedPositions.contains(boxPos) || reservedBoxPositions.contains(newBoxPos)) {
                return null;
            }
            if (!isAgentGoalPhase && satisfiedGoalPositions.contains(boxPos)) {
                return null;
            }
        } else if (action.type == Action.ActionType.PULL) {
            Position boxPos = agentPos.move(action.boxDir.opposite());
            Position newAgentPos = agentPos.move(action.agentDir);
            if (reservedPositions.contains(newAgentPos)) {
                return null;
            }
            if (!isAgentGoalPhase && satisfiedGoalPositions.contains(boxPos)) {
                return null;
            }
        }

        return action;
    }

    private void updateReservedPositions(int agentId, Action action, State state,
            Set<Position> reservedPositions, Set<Position> reservedBoxPositions) {
        
        Position agentPos = state.getAgentPosition(agentId);
        reservedPositions.add(agentPos);

        if (action.type == Action.ActionType.MOVE) {
            Position newPos = agentPos.move(action.agentDir);
            reservedPositions.add(newPos);
        } else if (action.type == Action.ActionType.PUSH) {
            Position boxPos = agentPos.move(action.agentDir);
            Position newBoxPos = boxPos.move(action.boxDir);
            reservedPositions.add(boxPos);
            reservedBoxPositions.add(newBoxPos);
        } else if (action.type == Action.ActionType.PULL) {
            Position newAgentPos = agentPos.move(action.agentDir);
            reservedPositions.add(newAgentPos);
            reservedBoxPositions.add(agentPos);
        }
    }

    /**
     * Apply a joint action and return the new state.
     * Per CLAUDE.md: actions are simultaneous — cell occupancy evaluated at START.
     */
    public State applyJointAction(Action[] jointAction, State state, int numAgents) {
        return state.applyJointAction(jointAction, null);
    }
    
    /**
     * Apply a joint action with level context for simultaneous evaluation.
     */
    public State applyJointAction(Action[] jointAction, State state, int numAgents, Level level) {
        return state.applyJointAction(jointAction, level);
    }

    /**
     * Updates plan indexes after applying a joint action.
     */
    public void updatePlanIndexes(Action[] jointAction, int numAgents, int primaryAgent) {
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgent) {
                continue;
            }
            
            List<Action> plan = storedPlans.get(agentId);
            int index = planIndexes.getOrDefault(agentId, 0);

            if (plan != null && index < plan.size()) {
                Action expectedAction = plan.get(index);
                if (jointAction[agentId] != null &&
                        jointAction[agentId].type == expectedAction.type &&
                        jointAction[agentId].agentDir == expectedAction.agentDir &&
                        jointAction[agentId].boxDir == expectedAction.boxDir) {
                    planIndexes.put(agentId, index + 1);
                }
            }
        }
    }

    /**
     * Clear all stored plans.
     */
    public void clearAllPlans() {
        storedPlans.clear();
        planIndexes.clear();
        planUsageAtStep.clear();
    }

    /**
     * Get count of stored plans.
     */
    public int getStoredPlanCount() {
        return storedPlans.size();
    }
}
