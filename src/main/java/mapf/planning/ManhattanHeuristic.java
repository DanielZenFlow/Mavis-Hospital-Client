package mapf.planning;

import mapf.domain.*;

import java.util.*;

/**
 * Manhattan distance heuristic.
 * 
 * Calculates the sum of Manhattan distances from each box to its nearest
 * matching goal position. Also considers agent positions relative to their goals.
 * 
 * This is an admissible heuristic (never overestimates) because:
 * - Manhattan distance is the minimum number of moves to get from A to B
 *   in a grid without obstacles
 * - We use the minimum distance to any matching goal
 * 
 * Note: This heuristic ignores walls, so it may underestimate significantly
 * in mazes. Use TrueDistanceHeuristic for better estimates.
 */
public class ManhattanHeuristic implements Heuristic {
    
    /** Cached goal positions for each box type */
    private Map<Character, List<Position>> boxGoalPositions = null;
    
    /** Cached goal positions for each agent */
    private Map<Integer, Position> agentGoalPositions = null;
    
    /**
     * Creates a new ManhattanHeuristic.
     * Goal positions are cached on first use.
     */
    public ManhattanHeuristic() {
        // Goal positions will be cached on first estimate call
    }
    
    @Override
    public int estimate(State state, Level level) {
        // Cache goal positions if not already done
        if (boxGoalPositions == null) {
            cacheGoalPositions(level);
        }
        
        int totalDistance = 0;
        
        // Calculate distance for each box to its nearest goal
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position boxPos = entry.getKey();
            char boxType = entry.getValue();
            
            List<Position> goals = boxGoalPositions.get(boxType);
            if (goals != null && !goals.isEmpty()) {
                int minDist = Integer.MAX_VALUE;
                for (Position goal : goals) {
                    int dist = boxPos.manhattanDistance(goal);
                    minDist = Math.min(minDist, dist);
                }
                if (minDist < Integer.MAX_VALUE) {
                    totalDistance += minDist;
                }
            }
        }
        
        // Calculate distance for each agent to its goal (if any)
        for (Map.Entry<Integer, Position> entry : agentGoalPositions.entrySet()) {
            int agentId = entry.getKey();
            Position goalPos = entry.getValue();
            Position agentPos = state.getAgentPosition(agentId);
            
            if (agentPos != null && goalPos != null) {
                totalDistance += agentPos.manhattanDistance(goalPos);
            }
        }
        
        return totalDistance;
    }
    
    /**
     * Caches the goal positions for each box type and agent.
     * 
     * @param level the level to extract goals from
     */
    private void cacheGoalPositions(Level level) {
        boxGoalPositions = new HashMap<>();
        agentGoalPositions = new HashMap<>();
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                Position pos = new Position(row, col);
                
                // Check for box goal
                char boxGoal = level.getBoxGoal(row, col);
                if (boxGoal != '\0') {
                    boxGoalPositions.computeIfAbsent(boxGoal, k -> new ArrayList<>()).add(pos);
                }
                
                // Check for agent goal
                int agentGoal = level.getAgentGoal(row, col);
                if (agentGoal != -1) {
                    agentGoalPositions.put(agentGoal, pos);
                }
            }
        }
    }
    
    /**
     * Calculates the Manhattan distance for a single box to its nearest goal.
     * 
     * @param boxPos the box position
     * @param boxType the box type
     * @param level the level
     * @return the minimum Manhattan distance to a matching goal
     */
    public int distanceToGoal(Position boxPos, char boxType, Level level) {
        if (boxGoalPositions == null) {
            cacheGoalPositions(level);
        }
        
        List<Position> goals = boxGoalPositions.get(boxType);
        if (goals == null || goals.isEmpty()) {
            return 0; // No goal for this box type
        }
        
        int minDist = Integer.MAX_VALUE;
        for (Position goal : goals) {
            int dist = boxPos.manhattanDistance(goal);
            minDist = Math.min(minDist, dist);
        }
        
        return minDist == Integer.MAX_VALUE ? 0 : minDist;
    }
}
