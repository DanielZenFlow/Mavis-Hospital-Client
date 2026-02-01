package mapf.planning.strategy;

import mapf.domain.*;
import java.util.*;

/**
 * Detects immovable boxes and computes distances considering them as walls.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class ImmovableBoxDetector {
    
    private Set<Position> immovableBoxPositions = null;
    private State cachedStateForImmovable = null;
    
    private Set<Position> preSatisfiedStaticGoals = null;
    private State cachedStateForStaticGoals = null;
    
    /**
     * Gets immovable box positions (cached per state).
     */
    public Set<Position> getImmovableBoxes(State state, Level level) {
        if (immovableBoxPositions != null && cachedStateForImmovable == state) {
            return immovableBoxPositions;
        }
        
        cachedStateForImmovable = state;
        immovableBoxPositions = new HashSet<>();
        
        // Find which colors have agents
        Set<Color> pushableColors = new HashSet<>();
        for (int i = 0; i < state.getNumAgents(); i++) {
            pushableColors.add(level.getAgentColor(i));
        }
        
        // Mark boxes with no matching agent color as immovable
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            char boxType = entry.getValue();
            Color boxColor = level.getBoxColor(boxType);
            if (!pushableColors.contains(boxColor)) {
                immovableBoxPositions.add(entry.getKey());
            }
        }
        
        return immovableBoxPositions;
    }
    
    /**
     * Finds pre-satisfied static goals (boxes at goals that cannot be moved).
     */
    public Set<Position> findPreSatisfiedStaticGoals(State state, Level level) {
        if (preSatisfiedStaticGoals != null && cachedStateForStaticGoals == state) {
            return preSatisfiedStaticGoals;
        }
        
        cachedStateForStaticGoals = state;
        preSatisfiedStaticGoals = new HashSet<>();
        
        Set<Position> immovableBoxes = getImmovableBoxes(state, level);
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType == '\0') continue;
                
                Position goalPos = new Position(row, col);
                Character actualBox = state.getBoxes().get(goalPos);
                
                if (actualBox != null && actualBox == goalType && immovableBoxes.contains(goalPos)) {
                    preSatisfiedStaticGoals.add(goalPos);
                }
            }
        }
        
        return preSatisfiedStaticGoals;
    }
    
    /**
     * Computes distance treating immovable boxes as walls (BFS).
     */
    public int getDistanceWithImmovableBoxes(Position from, Position to, State state, Level level) {
        if (from.equals(to)) return 0;
        
        Set<Position> immovableBoxes = getImmovableBoxes(state, level);
        
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distances = new HashMap<>();
        
        queue.add(from);
        distances.put(from, 0);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int dist = distances.get(current);
            
            if (current.equals(to)) return dist;
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                // Skip if visited
                if (distances.containsKey(next)) continue;
                
                // Skip if wall
                if (level.isWall(next)) continue;
                
                // Skip if immovable box (treat as wall)
                if (immovableBoxes.contains(next)) continue;
                
                distances.put(next, dist + 1);
                queue.add(next);
            }
        }
        
        return Integer.MAX_VALUE; // Unreachable
    }
}
