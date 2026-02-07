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
     * Phase 2: Agent goals for agents whose own-color box tasks are done
     *          (not deferred until ALL box goals complete)
     */
    public List<PriorityPlanningStrategy.Subgoal> getUnsatisfiedSubgoals(State state, Level level, Set<Position> completedBoxGoals) {
        List<PriorityPlanningStrategy.Subgoal> unsatisfied = new ArrayList<>();
        
        Set<Position> staticGoals = immovableDetector.findPreSatisfiedStaticGoals(state, level);
        
        // Phase 1: Box goals (skip completed ones - MAPF permanent obstacle)
        addBoxGoals(unsatisfied, state, level, staticGoals, completedBoxGoals);
        
        // Phase 2: Agent goals for agents that have completed their own box tasks.
        // This allows agent goals to be attempted in parallel with other agents' box goals,
        // rather than waiting for ALL box goals to complete first.
        addCompletedAgentGoals(unsatisfied, state, level);
        
        // Phase 3: If no box goals remain at all, add ALL remaining agent goals
        // (catches agent goals for agents that had no box tasks)
        if (unsatisfied.isEmpty() || !hasAnyBoxGoals(unsatisfied)) {
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
        // Collect agent IDs already present in the list to avoid duplicates
        Set<Integer> existingAgentGoals = new HashSet<>();
        for (PriorityPlanningStrategy.Subgoal sg : unsatisfied) {
            if (sg.isAgentGoal) {
                existingAgentGoals.add(sg.agentId);
            }
        }
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                int agentGoal = level.getAgentGoal(row, col);
                if (agentGoal >= 0 && agentGoal < state.getNumAgents()) {
                    if (existingAgentGoals.contains(agentGoal)) continue; // Already added
                    Position goalPos = new Position(row, col);
                    Position agentPos = state.getAgentPosition(agentGoal);
                    if (!agentPos.equals(goalPos)) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentGoal, '\0', goalPos, true));
                    }
                }
            }
        }
    }
    
    /** Checks if the subgoal list contains any box goals. */
    private boolean hasAnyBoxGoals(List<PriorityPlanningStrategy.Subgoal> subgoals) {
        for (PriorityPlanningStrategy.Subgoal sg : subgoals) {
            if (!sg.isAgentGoal) return true;
        }
        return false;
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
            
            // Pukoban fix: check box has at least one direction where push or pull is possible
            // Push: agent on one side, empty on the other -> needs two opposite-side free neighbors
            // Pull: agent adjacent, agent can move away -> needs at least one free neighbor for agent
            if (!isBoxMovable(boxPos, state, level)) continue;
            
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
     * Checks if a box can be pushed or pulled in at least one direction.
     * Push: requires agent-side free AND push-direction free (opposite neighbors along an axis)
     * Pull: requires agent adjacent AND agent has somewhere to move while pulling
     * Returns true if box is not stuck.
     */
    private boolean isBoxMovable(Position boxPos, State state, Level level) {
        int freeNeighborCount = 0;
        boolean hasPushAxis = false;
        
        Direction[] dirs = Direction.values();
        for (Direction dir : dirs) {
            Position neighbor = boxPos.move(dir);
            if (!level.isWall(neighbor) && !state.hasBoxAt(neighbor)) {
                freeNeighborCount++;
                // Check opposite direction for push possibility
                Position opposite = boxPos.move(dir.opposite());
                if (!level.isWall(opposite) && !state.hasBoxAt(opposite)) {
                    hasPushAxis = true; // Can push along this axis
                }
            }
        }
        
        // Pull only needs 1 free neighbor (agent stands adjacent, pulls box toward itself)
        // Push needs 2 opposite free neighbors
        // Either is sufficient to move the box
        return hasPushAxis || freeNeighborCount >= 1;
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
