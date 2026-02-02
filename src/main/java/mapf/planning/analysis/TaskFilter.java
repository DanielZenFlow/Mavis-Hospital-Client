package mapf.planning.analysis;

import mapf.domain.*;
import java.util.*;

/**
 * Filters tasks to identify active goals that actually need solving.
 * Excludes: immovable boxes (no matching agent) and already-satisfied goals.
 */
public class TaskFilter {
    
    /** Result of task filtering analysis. */
    public static class FilterResult {
        /** Boxes that cannot be moved (no agent of same color). */
        public final Set<Position> immovableBoxes;
        
        /** Goals already satisfied (box already at goal position). */
        public final Set<Position> satisfiedGoals;
        
        /** Active goals that need to be solved. */
        public final List<Position> activeGoals;
        
        /** Active boxes that need to be moved. */
        public final Set<Position> activeBoxes;
        
        /** Summary report for debugging. */
        public final String report;
        
        public FilterResult(Set<Position> immovableBoxes, Set<Position> satisfiedGoals,
                           List<Position> activeGoals, Set<Position> activeBoxes, String report) {
            this.immovableBoxes = Collections.unmodifiableSet(immovableBoxes);
            this.satisfiedGoals = Collections.unmodifiableSet(satisfiedGoals);
            this.activeGoals = Collections.unmodifiableList(activeGoals);
            this.activeBoxes = Collections.unmodifiableSet(activeBoxes);
            this.report = report;
        }
        
        public int getActiveGoalCount() {
            return activeGoals.size();
        }
        
        public boolean isImmovable(Position boxPos) {
            return immovableBoxes.contains(boxPos);
        }
        
        public boolean isSatisfied(Position goalPos) {
            return satisfiedGoals.contains(goalPos);
        }
    }
    
    /**
     * Analyzes the level to filter out non-tasks.
     * 
     * @param level The level definition
     * @param state The current state
     * @return FilterResult with categorized goals and boxes
     */
    public static FilterResult filter(Level level, State state) {
        Set<Position> immovableBoxes = new HashSet<>();
        Set<Position> satisfiedGoals = new HashSet<>();
        List<Position> activeGoals = new ArrayList<>();
        Set<Position> activeBoxes = new HashSet<>();
        
        // Step 1: Find all agent colors available
        Set<Color> availableColors = findAgentColors(level, state);
        
        // Step 2: Identify immovable boxes (no matching agent color)
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position boxPos = entry.getKey();
            char boxType = entry.getValue();
            Color boxColor = level.getBoxColor(boxType);
            
            if (!availableColors.contains(boxColor)) {
                immovableBoxes.add(boxPos);
            }
        }
        
        // Step 3: Categorize box goals
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                char goalType = level.getBoxGoal(r, c);
                if (goalType == '\0') continue;
                
                Position goalPos = new Position(r, c);
                Character boxAtGoal = state.getBoxAt(goalPos);
                
                // Check if goal is already satisfied
                if (boxAtGoal != null && boxAtGoal == goalType) {
                    satisfiedGoals.add(goalPos);
                    continue;
                }
                
                // Check if any movable box of this type exists
                Color goalColor = level.getBoxColor(goalType);
                if (!availableColors.contains(goalColor)) {
                    // No agent can move boxes to this goal - treat as satisfied (skip)
                    satisfiedGoals.add(goalPos);
                    continue;
                }
                
                // This is an active goal
                activeGoals.add(goalPos);
            }
        }
        
        // Step 4: Identify active boxes (boxes that need to move)
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position boxPos = entry.getKey();
            char boxType = entry.getValue();
            
            // Skip immovable boxes
            if (immovableBoxes.contains(boxPos)) continue;
            
            // Check if this box is already at its goal
            if (level.getBoxGoal(boxPos) == boxType) continue;
            
            // Check if there's an unsatisfied goal for this box type
            for (Position goal : activeGoals) {
                if (level.getBoxGoal(goal) == boxType) {
                    activeBoxes.add(boxPos);
                    break;
                }
            }
        }
        
        // Step 5: Handle agent goals (always active if not satisfied)
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                int agentGoal = level.getAgentGoal(r, c);
                if (agentGoal < 0) continue;
                
                Position goalPos = new Position(r, c);
                Position agentPos = state.getAgentPosition(agentGoal);
                
                if (!goalPos.equals(agentPos)) {
                    activeGoals.add(goalPos);
                }
            }
        }
        
        String report = generateReport(immovableBoxes, satisfiedGoals, activeGoals, activeBoxes);
        return new FilterResult(immovableBoxes, satisfiedGoals, activeGoals, activeBoxes, report);
    }
    
    /** Finds all colors that have at least one agent. */
    private static Set<Color> findAgentColors(Level level, State state) {
        Set<Color> colors = new HashSet<>();
        for (int i = 0; i < state.getNumAgents(); i++) {
            colors.add(level.getAgentColor(i));
        }
        return colors;
    }
    
    private static String generateReport(Set<Position> immovable, Set<Position> satisfied,
                                        List<Position> active, Set<Position> activeBoxes) {
        StringBuilder sb = new StringBuilder();
        sb.append("\n--- TASK FILTER ---\n");
        
        if (!immovable.isEmpty()) {
            sb.append("Immovable boxes (no matching agent): ").append(immovable.size()).append("\n");
        }
        if (!satisfied.isEmpty()) {
            sb.append("Already satisfied goals: ").append(satisfied.size()).append("\n");
        }
        sb.append("Active goals: ").append(active.size()).append("\n");
        sb.append("Active boxes: ").append(activeBoxes.size()).append("\n");
        
        return sb.toString();
    }
}
