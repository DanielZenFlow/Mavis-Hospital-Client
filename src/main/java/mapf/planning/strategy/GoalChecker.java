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
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position goalPos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(goalPos);
                    if (actualBox == null || actualBox != goalType) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    /**
     * Checks if a specific agent has completed all box tasks.
     */
    public static boolean hasCompletedBoxTasks(int agentId, State state, Level level) {
        Color agentColor = level.getAgentColor(agentId);
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0' && level.getBoxColor(goalType) == agentColor) {
                    Position goalPos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(goalPos);
                    if (actualBox == null || actualBox != goalType) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    /**
     * Checks if an agent's position goal is satisfied.
     */
    public static boolean isAgentGoalSatisfied(int agentId, State state, Level level) {
        Position agentPos = state.getAgentPosition(agentId);
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    Position goalPos = new Position(row, col);
                    return agentPos.equals(goalPos);
                }
            }
        }
        
        return true; // No agent goal defined
    }
    
    /**
     * Computes all currently satisfied goal positions.
     */
    public static Set<Position> computeSatisfiedGoalPositions(State state, Level level) {
        Set<Position> satisfied = new HashSet<>();
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position goalPos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(goalPos);
                    if (actualBox != null && actualBox == goalType) {
                        satisfied.add(goalPos);
                    }
                }
            }
        }
        
        return satisfied;
    }
    
    /**
     * Finds the goal position for an agent.
     */
    public static Position findAgentGoalPosition(int agentId, Level level) {
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
}
