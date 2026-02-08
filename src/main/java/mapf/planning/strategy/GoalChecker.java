package mapf.planning.strategy;

import mapf.domain.*;
import java.util.*;

/**
 * Goal state checking utilities.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class GoalChecker {
    
    /**
     * Checks if all box goals are satisfied.
     */
    public static boolean allBoxGoalsSatisfied(State state, Level level) {
        for (Position goalPos : level.getAllBoxGoalPositions()) {
            char goalType = level.getBoxGoal(goalPos);
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox == null || actualBox != goalType) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * Checks if a specific agent has completed all box tasks.
     */
    public static boolean hasCompletedBoxTasks(int agentId, State state, Level level) {
        Color agentColor = level.getAgentColor(agentId);
        
        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            if (level.getBoxColor(goalType) != agentColor) continue;
            for (Position goalPos : entry.getValue()) {
                Character actualBox = state.getBoxes().get(goalPos);
                if (actualBox == null || actualBox != goalType) {
                    return false;
                }
            }
        }
        return true;
    }
    
    /**
     * Finds all agents who have completed their box tasks.
     */
    public static Set<Integer> findCompletedAgents(State state, Level level, int numAgents) {
        Set<Integer> completed = new HashSet<>();
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (hasCompletedBoxTasks(agentId, state, level)) {
                completed.add(agentId);
            }
        }
        return completed;
    }
    
    /**
     * Checks if an agent's position goal is satisfied.
     */
    public static boolean isAgentGoalSatisfied(int agentId, State state, Level level) {
        Position agentPos = state.getAgentPosition(agentId);
        
        Position goalPos = level.getAgentGoalPositionMap().get(agentId);
        if (goalPos != null) {
            return agentPos.equals(goalPos);
        }
        
        return true; // No agent goal defined
    }
    
    /**
     * Computes all currently satisfied goal positions.
     */
    public static Set<Position> computeSatisfiedGoalPositions(State state, Level level) {
        Set<Position> satisfied = new HashSet<>();
        
        for (Position goalPos : level.getAllBoxGoalPositions()) {
            char goalType = level.getBoxGoal(goalPos);
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox != null && actualBox == goalType) {
                satisfied.add(goalPos);
            }
        }
        
        return satisfied;
    }
    
    /**
     * Finds the goal position for an agent.
     */
    public static Position findAgentGoalPosition(int agentId, Level level) {
        return level.getAgentGoalPositionMap().get(agentId);
    }
    
    /**
     * Finds which agent can push boxes of a given color.
     */
    public static int findAgentForColor(Color color, Level level, int numAgents) {
        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) {
                return i;
            }
        }
        return -1;
    }
    
    /**
     * Checks if an action would disturb a satisfied goal.
     */
    public static boolean wouldDisturbSatisfiedGoal(Action action, int agentId, State state, Set<Position> satisfiedGoals) {
        if (satisfiedGoals.isEmpty()) {
            return false;
        }

        Position agentPos = state.getAgentPosition(agentId);
        if (agentPos == null) {
            return false;
        }

        switch (action.type) {
            case PUSH: {
                // The box is in front of the agent (in agentDir)
                Position boxPos = agentPos.move(action.agentDir);
                return satisfiedGoals.contains(boxPos);
            }
            case PULL: {
                // The box is behind the agent (opposite of boxDir)
                Position boxPos = agentPos.move(action.boxDir.opposite());
                return satisfiedGoals.contains(boxPos);
            }
            default:
                return false;
        }
    }
}
