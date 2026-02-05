package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.heuristic.Heuristic;

import java.util.*;

/**
 * Manages subgoal identification, ordering, and difficulty estimation.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class SubgoalManager {
    
    private final Heuristic heuristic;
    private final ImmovableBoxDetector immovableDetector;
    
    public SubgoalManager(Heuristic heuristic) {
        this.heuristic = heuristic;
        this.immovableDetector = new ImmovableBoxDetector();
    }
    
    /**
     * Gets all unsatisfied subgoals in priority order.
     * Phase 1: Box goals (excluding already completed ones)
     * Phase 2: Remaining agent goals (after all boxes placed)
     */
    public List<PriorityPlanningStrategy.Subgoal> getUnsatisfiedSubgoals(State state, Level level, Set<Position> completedBoxGoals) {
        List<PriorityPlanningStrategy.Subgoal> unsatisfied = new ArrayList<>();
        
        Set<Position> staticGoals = immovableDetector.findPreSatisfiedStaticGoals(state, level);
        
        // Phase 1: Box goals (skip completed ones - MAPF permanent obstacle)
        addBoxGoals(unsatisfied, state, level, staticGoals, completedBoxGoals);
        
        // Phase 2: Agent goals (only when no box goals remain)
        if (unsatisfied.isEmpty()) {
            addAllAgentGoals(unsatisfied, state, level);
        }
        
        return unsatisfied;
    }
    
    /** Legacy overload for backward compatibility. */
    public List<PriorityPlanningStrategy.Subgoal> getUnsatisfiedSubgoals(State state, Level level) {
        return getUnsatisfiedSubgoals(state, level, Collections.emptySet());
    }
    
    private void addBoxGoals(List<PriorityPlanningStrategy.Subgoal> unsatisfied, 
                            State state, Level level, Set<Position> staticGoals, Set<Position> completedBoxGoals) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType == '\0') continue;
                
                Position goalPos = new Position(row, col);
                
                // Skip pre-satisfied static goals (decorations)
                if (staticGoals.contains(goalPos)) continue;
                
                // Skip completed goals (MAPF: permanent obstacle)
                if (completedBoxGoals.contains(goalPos)) continue;
                
                Character actualBox = state.getBoxes().get(goalPos);
                if (actualBox == null || actualBox != goalType) {
                    Color boxColor = level.getBoxColor(goalType);
                    // MAPF FIX: Choose NEAREST agent instead of first available (index-based)
                    int agentId = findNearestAgentForColor(boxColor, goalPos, level, state);
                    if (agentId != -1) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentId, goalType, goalPos, false));
                    }
                }
            }
        }
    }
    
    private void addCompletedAgentGoals(List<PriorityPlanningStrategy.Subgoal> unsatisfied,
                                       State state, Level level) {
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            if (hasCompletedBoxTasks(agentId, state, level)) {
                Position agentGoal = findAgentGoalPosition(agentId, level);
                if (agentGoal != null) {
                    Position agentPos = state.getAgentPosition(agentId);
                    if (!agentPos.equals(agentGoal)) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentId, '\0', agentGoal, true));
                    }
                }
            }
        }
    }
    
    private void addAllAgentGoals(List<PriorityPlanningStrategy.Subgoal> unsatisfied,
                                 State state, Level level) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                int agentGoal = level.getAgentGoal(row, col);
                if (agentGoal >= 0 && agentGoal < state.getNumAgents()) {
                    Position goalPos = new Position(row, col);
                    Position agentPos = state.getAgentPosition(agentGoal);
                    if (!agentPos.equals(goalPos)) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentGoal, '\0', goalPos, true));
                    }
                }
            }
        }
    }
    
    public int estimateSubgoalDifficulty(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level) {
        if (subgoal.isAgentGoal) {
            Position agentPos = state.getAgentPosition(subgoal.agentId);
            return immovableDetector.getDistanceWithImmovableBoxes(agentPos, subgoal.goalPos, state, level);
        }
        
        Position closestBox = findBestBoxForGoal(subgoal, state, level);
        if (closestBox == null) return Integer.MAX_VALUE;
        
        int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(closestBox, subgoal.goalPos, state, level);
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        int agentToBox = immovableDetector.getDistanceWithImmovableBoxes(agentPos, closestBox, state, level);
        
        if (agentToBox == Integer.MAX_VALUE || boxToGoal == Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        
        return boxToGoal + agentToBox;
    }
    
    public Position findBestBoxForGoal(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level) {
        Position bestBox = null;
        int bestTotalDist = Integer.MAX_VALUE;
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        
        Set<Position> immovableBoxes = immovableDetector.getImmovableBoxes(state, level);
        
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() != subgoal.boxType) continue;
            
            Position boxPos = entry.getKey();
            
            // Skip if box already at satisfied goal
            if (level.getBoxGoal(boxPos) == subgoal.boxType) continue;
            
            // Skip if box is immovable
            if (immovableBoxes.contains(boxPos)) continue;
            
            // Check if agent can reach box
            int agentToBox = immovableDetector.getDistanceWithImmovableBoxes(agentPos, boxPos, state, level);
            if (agentToBox == Integer.MAX_VALUE) continue;
            
            int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(boxPos, subgoal.goalPos, state, level);
            if (boxToGoal == Integer.MAX_VALUE) continue;
            
            int totalDist = agentToBox + boxToGoal;
            if (totalDist < bestTotalDist) {
                bestTotalDist = totalDist;
                bestBox = boxPos;
            }
        }
        
        return bestBox;
    }
    
    /**
     * Finds the nearest agent of the matching color to the target position.
     * Uses heuristic distance (Manhattan) for efficiency.
     */
    private int findNearestAgentForColor(Color color, Position target, Level level, State state) {
        int bestAgentId = -1;
        int minDistance = Integer.MAX_VALUE;
        int numAgents = state.getNumAgents();

        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) {
                Position agentPos = state.getAgentPosition(i);
                // Use heuristic distance (Manhattan) - accurate enough for assignment and fast
                int dist = agentPos.manhattanDistance(target);
                
                if (dist < minDistance) {
                    minDistance = dist;
                    bestAgentId = i;
                }
            }
        }
        return bestAgentId;
    }
    
    /** Legacy method maintained for compilation if referenced elsewhere, but discouraged. */
    private int findAgentForColor(Color color, Level level, int numAgents) {
        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) return i;
        }
        return -1;
    }
    
    private boolean hasCompletedBoxTasks(int agentId, State state, Level level) {
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
}
