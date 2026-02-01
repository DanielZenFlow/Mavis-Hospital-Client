package mapf.planning.strategy;

import mapf.domain.*;
import java.util.*;

/**
 * Utility methods for planning calculations.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class PlanningUtils {
    
    /**
     * BFS distance between two positions, avoiding specified positions.
     */
    public static int bfsDistance(Position from, Position to, Level level, Set<Position> avoid) {
        if (from.equals(to)) return 0;
        
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
                
                if (distances.containsKey(next)) continue;
                if (level.isWall(next)) continue;
                if (avoid != null && avoid.contains(next)) continue;
                
                distances.put(next, dist + 1);
                queue.add(next);
            }
        }
        
        return Integer.MAX_VALUE;
    }
    
    /**
     * Finds path positions ignoring dynamic obstacles (boxes, agents).
     */
    public static Set<Position> findPathIgnoringDynamicObstacles(Position start, Position goal, Level level) {
        Set<Position> pathPositions = new HashSet<>();
        
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> parent = new HashMap<>();
        
        queue.add(start);
        parent.put(start, null);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            if (current.equals(goal)) {
                // Reconstruct path
                Position p = current;
                while (p != null) {
                    pathPositions.add(p);
                    p = parent.get(p);
                }
                return pathPositions;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (level.isWall(next) || parent.containsKey(next)) {
                    continue;
                }
                
                parent.put(next, current);
                queue.add(next);
            }
        }
        
        return pathPositions;
    }
    
    /**
     * Checks if a box is on the path between two positions.
     */
    public static boolean isBoxOnPath(Position boxPos, Position from, Position to, State state, Level level) {
        Set<Position> pathPositions = findPathIgnoringDynamicObstacles(from, to, level);
        return pathPositions.contains(boxPos);
    }
    
    /**
     * Finds the position of a box of given type.
     */
    public static Position findBoxPosition(State state, char boxType, Position hint) {
        // Try hint first
        if (hint != null) {
            Character boxAtHint = state.getBoxes().get(hint);
            if (boxAtHint != null && boxAtHint == boxType) {
                return hint;
            }
        }
        
        // Search all boxes
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                return entry.getKey();
            }
        }
        
        return null;
    }
    
    /**
     * Counts passable neighbors (not walls, not other agents).
     */
    public static int countPassableNeighbors(Position pos, State state, Level level, int excludeAgentId) {
        int count = 0;
        
        for (Direction dir : Direction.values()) {
            Position next = pos.move(dir);
            
            if (level.isWall(next)) continue;
            
            boolean hasOtherAgent = false;
            for (int i = 0; i < state.getNumAgents(); i++) {
                if (i != excludeAgentId && next.equals(state.getAgentPosition(i))) {
                    hasOtherAgent = true;
                    break;
                }
            }
            
            if (!hasOtherAgent) count++;
        }
        
        return count;
    }
    
    /**
     * Checks if a position is occupied by any agent.
     */
    public static boolean isPositionOccupiedByAgent(Position pos, State state, int numAgents) {
        for (int i = 0; i < numAgents; i++) {
            if (pos.equals(state.getAgentPosition(i))) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Gets all possible actions.
     */
    public static List<Action> getAllActions() {
        List<Action> actions = new ArrayList<>();
        
        // NoOp
        actions.add(Action.noOp());
        
        // Move
        for (Direction dir : Direction.values()) {
            actions.add(Action.move(dir));
        }
        
        // Push
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                actions.add(Action.push(agentDir, boxDir));
            }
        }
        
        // Pull
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                actions.add(Action.pull(agentDir, boxDir));
            }
        }
        
        return actions;
    }
}
