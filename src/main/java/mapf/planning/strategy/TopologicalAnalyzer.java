package mapf.planning.strategy;

import mapf.domain.*;
import java.util.*;

/**
 * Computes topological information about the level for goal ordering.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Key insight: Goals in dead-end corridors should be filled FIRST (reverse execution order).
 */
public class TopologicalAnalyzer {
    
    private Map<Position, Integer> goalTopologicalDepths = null;
    private Map<Position, Integer> reverseExecutionOrder = null;
    private Level cachedLevel = null;
    
    /**
     * Gets topological depth for a goal position (cached per level).
     * Higher depth = deeper in dead-end = fill first.
     */
    public int getTopologicalDepth(Position goalPos, Level level) {
        ensureTopologicalDepthsComputed(level);
        return goalTopologicalDepths.getOrDefault(goalPos, 0);
    }
    
    /**
     * Gets reverse execution priority (cached per level).
     * Higher value = deeper in corridor = execute first.
     */
    public int getReverseExecutionPriority(Position goalPos, Level level) {
        ensureReverseOrderComputed(level);
        return reverseExecutionOrder.getOrDefault(goalPos, 0);
    }
    
    private void ensureTopologicalDepthsComputed(Level level) {
        if (goalTopologicalDepths != null && cachedLevel == level) {
            return;
        }
        
        cachedLevel = level;
        goalTopologicalDepths = computeTopologicalDepths(level);
    }
    
    private void ensureReverseOrderComputed(Level level) {
        if (reverseExecutionOrder != null && cachedLevel == level) {
            return;
        }
        
        cachedLevel = level;
        reverseExecutionOrder = computeReverseExecutionOrder(level);
    }
    
    /**
     * Computes topological depths for all goal positions using BFS from exits.
     */
    private Map<Position, Integer> computeTopologicalDepths(Level level) {
        Map<Position, Integer> depths = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        // Start from all "exit" positions (positions with 3+ free neighbors)
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.isWall(row, col)) continue;
                
                Position pos = new Position(row, col);
                int freeNeighbors = countFreeNeighbors(pos, level);
                
                if (freeNeighbors >= 3) {
                    queue.add(pos);
                    visited.add(pos);
                    depths.put(pos, 0);
                }
            }
        }
        
        // BFS to assign depths
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int currentDepth = depths.get(current);
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (level.isWall(next) || visited.contains(next)) {
                    continue;
                }
                
                visited.add(next);
                depths.put(next, currentDepth + 1);
                queue.add(next);
            }
        }
        
        return depths;
    }
    
    /**
     * Computes reverse execution order based on corridor structure.
     */
    private Map<Position, Integer> computeReverseExecutionOrder(Level level) {
        Map<Position, Integer> order = new HashMap<>();
        
        // Find all goal positions
        List<Position> goalPositions = new ArrayList<>();
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getBoxGoal(row, col) != '\0') {
                    goalPositions.add(new Position(row, col));
                }
            }
        }
        
        // Compute corridor depth for each goal
        for (Position goal : goalPositions) {
            int depth = computeCorridorDepth(goal, level);
            order.put(goal, depth);
        }
        
        return order;
    }
    
    /**
     * Computes how deep a position is in a corridor.
     */
    private int computeCorridorDepth(Position start, Level level) {
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distances = new HashMap<>();
        
        queue.add(start);
        distances.put(start, 0);
        
        int maxDepth = 0;
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int dist = distances.get(current);
            maxDepth = Math.max(maxDepth, dist);
            
            int freeNeighbors = countFreeNeighbors(current, level);
            
            // Stop at intersections (3+ neighbors)
            if (freeNeighbors >= 3 && dist > 0) {
                continue;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (level.isWall(next) || distances.containsKey(next)) {
                    continue;
                }
                
                distances.put(next, dist + 1);
                queue.add(next);
            }
        }
        
        return maxDepth;
    }
    
    private int countFreeNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position next = pos.move(dir);
            if (!level.isWall(next)) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Counts blocking boxes between box and goal positions.
     */
    public int countBlockingBoxes(Position boxPos, Position goalPos, State state, Level level) {
        Set<Position> pathPositions = findPath(boxPos, goalPos, level);
        
        int blockingCount = 0;
        for (Position pos : pathPositions) {
            if (state.getBoxes().containsKey(pos) && !pos.equals(boxPos)) {
                blockingCount++;
            }
        }
        
        return blockingCount;
    }
    
    private Set<Position> findPath(Position from, Position to, Level level) {
        Set<Position> path = new HashSet<>();
        
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> parent = new HashMap<>();
        
        queue.add(from);
        parent.put(from, null);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            if (current.equals(to)) {
                // Reconstruct path
                Position p = current;
                while (p != null) {
                    path.add(p);
                    p = parent.get(p);
                }
                return path;
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
        
        return path;
    }
}
