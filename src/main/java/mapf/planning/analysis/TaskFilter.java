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
     * Uses reachability analysis: boxes/goals unreachable by any matching agent are excluded.
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
        
        // Step 1: Find all agent colors and positions
        Set<Color> availableColors = findAgentColors(level, state);
        Map<Color, List<Integer>> agentsByColor = groupAgentsByColor(level, state);
        
        // Step 2: Identify immovable boxes (no matching agent color)
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position boxPos = entry.getKey();
            char boxType = entry.getValue();
            Color boxColor = level.getBoxColor(boxType);
            
            if (!availableColors.contains(boxColor)) {
                immovableBoxes.add(boxPos);
            }
        }
        
        // Step 3: Compute reachable areas for each agent (treating immovable boxes as walls)
        Map<Integer, Set<Position>> agentReachableAreas = computeAgentReachability(level, state, immovableBoxes);
        
        // Step 4: Compute reachable area per color (union of all agents of that color)
        Map<Color, Set<Position>> colorReachableAreas = new HashMap<>();
        for (Map.Entry<Color, List<Integer>> entry : agentsByColor.entrySet()) {
            Set<Position> colorArea = new HashSet<>();
            for (int agentId : entry.getValue()) {
                colorArea.addAll(agentReachableAreas.getOrDefault(agentId, Collections.emptySet()));
            }
            colorReachableAreas.put(entry.getKey(), colorArea);
        }
        
        // Step 5: Mark boxes as immovable if unreachable by any matching agent
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position boxPos = entry.getKey();
            if (immovableBoxes.contains(boxPos)) continue; // Already marked
            
            char boxType = entry.getValue();
            Color boxColor = level.getBoxColor(boxType);
            Set<Position> reachable = colorReachableAreas.get(boxColor);
            
            // Box is reachable if agent can reach an adjacent cell
            boolean canReach = false;
            for (Direction dir : Direction.values()) {
                Position adjacent = boxPos.move(dir);
                if (reachable != null && reachable.contains(adjacent)) {
                    canReach = true;
                    break;
                }
            }
            
            if (!canReach) {
                immovableBoxes.add(boxPos);
            }
        }
        
        // Step 6: Categorize box goals
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
                
                // Check if goal is reachable by matching agents
                Set<Position> reachable = colorReachableAreas.get(goalColor);
                if (reachable == null || !reachable.contains(goalPos)) {
                    // Goal position is unreachable - skip
                    satisfiedGoals.add(goalPos);
                    continue;
                }
                
                // Check if any reachable box of this type exists IN THE SAME REACHABLE AREA
                boolean hasReachableBox = false;
                for (Map.Entry<Position, Character> boxEntry : state.getBoxes().entrySet()) {
                    Position boxPos = boxEntry.getKey();
                    if (boxEntry.getValue() != goalType) continue;
                    if (immovableBoxes.contains(boxPos)) continue;
                    
                    // Box must be adjacent to a position in the same reachable area
                    for (Direction dir : Direction.values()) {
                        Position adjacent = boxPos.move(dir);
                        if (reachable.contains(adjacent)) {
                            hasReachableBox = true;
                            break;
                        }
                    }
                    if (hasReachableBox) break;
                }
                
                if (!hasReachableBox) {
                    // No reachable box for this goal (box and goal in different areas)
                    satisfiedGoals.add(goalPos);
                    continue;
                }
                
                // This is an active goal
                activeGoals.add(goalPos);
            }
        }
        
        // Step 7: Identify active boxes (boxes that need to move)
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
        
        // Step 8: Handle agent goals (always active if not satisfied and reachable)
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                int agentGoal = level.getAgentGoal(r, c);
                if (agentGoal < 0) continue;
                
                Position goalPos = new Position(r, c);
                Position agentPos = state.getAgentPosition(agentGoal);
                
                if (!goalPos.equals(agentPos)) {
                    // Check if agent can reach its goal
                    Set<Position> reachable = agentReachableAreas.get(agentGoal);
                    if (reachable != null && reachable.contains(goalPos)) {
                        activeGoals.add(goalPos);
                    }
                }
            }
        }
        
        String report = generateReport(immovableBoxes, satisfiedGoals, activeGoals, activeBoxes);
        return new FilterResult(immovableBoxes, satisfiedGoals, activeGoals, activeBoxes, report);
    }
    
    /** Groups agents by their color. */
    private static Map<Color, List<Integer>> groupAgentsByColor(Level level, State state) {
        Map<Color, List<Integer>> result = new HashMap<>();
        for (int i = 0; i < state.getNumAgents(); i++) {
            Color color = level.getAgentColor(i);
            result.computeIfAbsent(color, k -> new ArrayList<>()).add(i);
        }
        return result;
    }
    
    /**
     * Computes reachable areas for each agent using BFS.
     * Immovable boxes are treated as walls (permanent obstacles).
     */
    private static Map<Integer, Set<Position>> computeAgentReachability(
            Level level, State state, Set<Position> immovableBoxes) {
        Map<Integer, Set<Position>> result = new HashMap<>();
        
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            Position startPos = state.getAgentPosition(agentId);
            Set<Position> reachable = bfsReachability(startPos, level, immovableBoxes);
            result.put(agentId, reachable);
        }
        
        return result;
    }
    
    /**
     * BFS to find all positions reachable from start, treating immovable boxes as walls.
     */
    private static Set<Position> bfsReachability(Position start, Level level, Set<Position> obstacles) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        
        queue.add(start);
        visited.add(start);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                // Skip if wall, obstacle, or already visited
                if (level.isWall(next) || obstacles.contains(next) || visited.contains(next)) {
                    continue;
                }
                
                visited.add(next);
                queue.add(next);
            }
        }
        
        return visited;
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
