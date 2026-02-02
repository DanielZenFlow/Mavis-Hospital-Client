package mapf.planning.strategy;

import mapf.domain.*;

import java.util.*;

/**
 * Handles greedy planning and action selection strategies.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Responsibilities:
 * - tryGreedyStep: Make a single greedy step when stuck
 * - tryGreedyStepWithMerging: Greedy step with plan merging
 * - findBestGreedyAction: Find best action using heuristic
 * - tryRandomEscapeMove: Random escape when stuck
 */
public class GreedyPlanner {

    /**
     * Tries to make a greedy single step when stuck.
     */
    public boolean tryGreedyStep(List<Action[]> plan, State state, Level level, int numAgents,
            GoalChecker goalChecker, AgentStateProvider stateProvider,
            ActionEvaluator evaluator, ConflictResolver conflictResolver) {
        
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (GoalChecker.isAgentGoalSatisfied(agentId, state, level)) {
                continue;
            }

            if (stateProvider.isYielding(agentId)) {
                continue;
            }

            Action bestAction = findBestGreedyAction(agentId, state, level, evaluator, goalChecker);

            if (bestAction != null && bestAction.type != Action.ActionType.NOOP) {
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[agentId] = bestAction;

                jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
                plan.add(jointAction);
                return true;
            }
        }

        return false;
    }

    /**
     * Tries to make a greedy step with plan merging.
     */
    public boolean tryGreedyStepWithMerging(List<Action[]> plan, State state, Level level, int numAgents,
            AgentStateProvider stateProvider, ActionEvaluator evaluator,
            ConflictResolver conflictResolver, PlanMerger planMerger) {
        
        int primaryAgent = -1;
        Action primaryAction = null;

        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (GoalChecker.isAgentGoalSatisfied(agentId, state, level))
                continue;

            if (stateProvider.isYielding(agentId)) {
                continue;
            }

            Action bestAction = findBestGreedyAction(agentId, state, level, evaluator, null);
            if (bestAction != null && bestAction.type != Action.ActionType.NOOP) {
                primaryAgent = agentId;
                primaryAction = bestAction;
                break;
            }
        }

        if (primaryAgent == -1) {
            for (int agentId = 0; agentId < numAgents; agentId++) {
                if (GoalChecker.isAgentGoalSatisfied(agentId, state, level))
                    continue;

                if (stateProvider.isYielding(agentId))
                    continue;

                for (Action action : PlanningUtils.getAllActions()) {
                    if (action.type == Action.ActionType.MOVE &&
                            state.isApplicable(action, agentId, level)) {
                        primaryAgent = agentId;
                        primaryAction = action;
                        break;
                    }
                }
                if (primaryAgent != -1)
                    break;
            }
        }

        if (primaryAgent != -1 && primaryAction != null) {
            boolean isAgentGoalPhase = GoalChecker.allBoxGoalsSatisfied(state, level);
            Action[] jointAction = planMerger.createJointActionWithMerging(
                    primaryAgent, primaryAction, state, level, numAgents, isAgentGoalPhase);
            jointAction = conflictResolver.resolveConflicts(jointAction, state, level);
            plan.add(jointAction);
            return true;
        }

        return false;
    }

    /**
     * Finds the best greedy action for an agent using the heuristic.
     */
    public Action findBestGreedyAction(int agentId, State state, Level level,
            ActionEvaluator evaluator, GoalChecker goalChecker) {
        Action bestAction = Action.noOp();
        int bestH = evaluator.estimateAgentCost(agentId, state, level);

        Set<Position> frozenGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);

        for (Action action : PlanningUtils.getAllActions()) {
            if (action.type == Action.ActionType.NOOP) {
                continue;
            }

            if (!state.isApplicable(action, agentId, level)) {
                continue;
            }

            if (wouldDisturbSatisfiedGoal(action, agentId, state, frozenGoals)) {
                continue;
            }

            State newState = state.apply(action, agentId);
            int newH = evaluator.estimateAgentCost(agentId, newState, level);

            if (newH < bestH) {
                bestH = newH;
                bestAction = action;
            }
        }

        return bestAction;
    }

    /**
     * Fallback method: try any valid move to escape.
     */
    public boolean tryRandomEscapeMove(List<Action[]> plan, State state, Level level,
            int numAgents, int agentId, Set<Position> criticalPositions,
            ConflictResolver conflictResolver) {
        Position currentPos = state.getAgentPosition(agentId);

        for (Direction dir : Direction.values()) {
            Position newPos = currentPos.move(dir);

            if (!level.isWall(newPos) &&
                    !state.getBoxes().containsKey(newPos) &&
                    !criticalPositions.contains(newPos)) {

                boolean occupied = false;
                for (int i = 0; i < numAgents; i++) {
                    if (state.getAgentPosition(i).equals(newPos)) {
                        occupied = true;
                        break;
                    }
                }

                if (!occupied) {
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

    private boolean wouldDisturbSatisfiedGoal(Action action, int agentId, State state, Set<Position> satisfiedGoals) {
        if (satisfiedGoals.isEmpty()) {
            return false;
        }

        Position agentPos = state.getAgentPosition(agentId);

        if (action.type == Action.ActionType.PUSH) {
            Position boxPos = agentPos.move(action.agentDir);
            return satisfiedGoals.contains(boxPos);
        }

        if (action.type == Action.ActionType.PULL) {
            Position boxPos = agentPos.move(action.boxDir.opposite());
            return satisfiedGoals.contains(boxPos);
        }

        return false;
    }

    // ========== Interfaces ==========

    /**
     * Interface for agent state queries.
     */
    public interface AgentStateProvider {
        boolean isYielding(int agentId);
        boolean hasCompletedTask(int agentId);
        int getBeneficiary(int agentId);
    }

    /**
     * Interface for action cost evaluation.
     */
    public interface ActionEvaluator {
        int estimateAgentCost(int agentId, State state, Level level);
    }
}
