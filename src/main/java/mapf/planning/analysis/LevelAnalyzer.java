package mapf.planning.analysis;

import mapf.domain.*;
import java.util.*;

/**
 * Pre-analyzes level characteristics to guide strategy selection.
 * Computes goal dependencies, structural features, and recommends strategies.
 */
public class LevelAnalyzer {
    
    // ========== Analysis Result ==========
    
    public static class LevelFeatures {
        // Basic metrics
        public final int numAgents;
        public final int numBoxes;
        public final int numGoals;
        public final int freeSpaces;
        public final double density;
        
        // Task filtering (P0: identifies real tasks)
        public final TaskFilter.FilterResult taskFilter;
        
        // Goal dependency analysis
        public final Map<Position, Set<Position>> goalBlockedBy;  // goal -> goals that block it
        public final List<Position> executionOrder;  // topologically sorted goals
        public final int maxDependencyDepth;
        public final boolean hasCircularDependency;
        
        // Structural features
        public final int corridorCells;      // cells with ≤2 neighbors
        public final int junctionCells;      // cells with ≥3 neighbors
        public final double corridorRatio;   // corridorCells / freeSpaces
        
        // Strategy recommendation
        public final StrategyType recommendedStrategy;
        public final String analysisReport;
        
        public LevelFeatures(int numAgents, int numBoxes, int numGoals, int freeSpaces,
                            TaskFilter.FilterResult taskFilter,
                            Map<Position, Set<Position>> goalBlockedBy,
                            List<Position> executionOrder, int maxDepth, boolean hasCycle,
                            int corridorCells, int junctionCells,
                            StrategyType recommended, String report) {
            this.numAgents = numAgents;
            this.numBoxes = numBoxes;
            this.numGoals = numGoals;
            this.freeSpaces = freeSpaces;
            this.density = (double)(numAgents + numBoxes) / Math.max(1, freeSpaces);
            this.taskFilter = taskFilter;
            this.goalBlockedBy = goalBlockedBy;
            this.executionOrder = executionOrder;
            this.maxDependencyDepth = maxDepth;
            this.hasCircularDependency = hasCycle;
            this.corridorCells = corridorCells;
            this.junctionCells = junctionCells;
            this.corridorRatio = (double) corridorCells / Math.max(1, freeSpaces);
            this.recommendedStrategy = recommended;
            this.analysisReport = report;
        }
    }
    
    public enum StrategyType {
        SINGLE_AGENT,           // 1 agent: simple A*
        JOINT_SEARCH,           // 2-3 agents, low coupling: joint A*
        STRICT_ORDER,           // Strong dependencies: execute in order
        GREEDY_WITH_RETRY,      // General case: greedy + retry on failure
        CYCLE_BREAKER           // Circular dependencies: break cycle first
    }
    
    // ========== Main Analysis Entry Point ==========
    
    /**
     * Analyzes the level and returns features for strategy selection.
     */
    public static LevelFeatures analyze(Level level, State state) {
        // 1. Basic metrics
        int numAgents = state.getNumAgents();
        int numBoxes = state.getBoxes().size();
        int freeSpaces = countFreeSpaces(level);
        
        // 2. Task filtering (P0: identify real tasks)
        TaskFilter.FilterResult taskFilter = TaskFilter.filter(level, state);
        List<Position> activeGoals = taskFilter.activeGoals;
        int numGoals = activeGoals.size();
        
        // 3. Structural analysis
        int[] corridorJunction = countCorridorsAndJunctions(level);
        int corridorCells = corridorJunction[0];
        int junctionCells = corridorJunction[1];
        
        // 4. Goal dependency analysis (only on active goals)
        Map<Position, Set<Position>> goalBlockedBy = computeGoalDependencies(activeGoals, level);
        boolean hasCycle = detectCycle(goalBlockedBy, activeGoals);
        List<Position> executionOrder = hasCycle ? activeGoals : topologicalSort(goalBlockedBy, activeGoals);
        int maxDepth = computeMaxDependencyDepth(goalBlockedBy, activeGoals);
        
        // 5. Strategy recommendation
        StrategyType recommended = recommendStrategy(numAgents, maxDepth, hasCycle, 
                                                     (double) corridorCells / Math.max(1, freeSpaces));
        
        // 6. Generate report
        String report = generateReport(numAgents, numBoxes, numGoals, freeSpaces, taskFilter,
                                       goalBlockedBy, maxDepth, hasCycle, 
                                       corridorCells, junctionCells, recommended);
        
        return new LevelFeatures(numAgents, numBoxes, numGoals, freeSpaces, taskFilter,
                                goalBlockedBy, executionOrder, maxDepth, hasCycle,
                                corridorCells, junctionCells, recommended, report);
    }
    
    // ========== Goal Dependency Analysis ==========
    
    /**
     * Computes which goals block which other goals.
     * Goal A blocks Goal B if A is on the shortest path to B.
     */
    private static Map<Position, Set<Position>> computeGoalDependencies(List<Position> goals, Level level) {
        Map<Position, Set<Position>> blockedBy = new HashMap<>();
        
        for (Position goalB : goals) {
            blockedBy.put(goalB, new HashSet<>());
            
            // Find shortest distance to goalB from any entry point
            int directDist = bfsDistanceFromEdge(goalB, level, Collections.emptySet());
            
            for (Position goalA : goals) {
                if (goalA.equals(goalB)) continue;
                
                // Check if avoiding goalA increases distance to goalB
                int avoidDist = bfsDistanceFromEdge(goalB, level, Collections.singleton(goalA));
                
                if (avoidDist > directDist) {
                    // goalA blocks goalB - must handle goalA first
                    blockedBy.get(goalB).add(goalA);
                }
            }
        }
        
        return blockedBy;
    }
    
    /**
     * BFS distance from level edges to target, avoiding specified positions.
     */
    private static int bfsDistanceFromEdge(Position target, Level level, Set<Position> avoid) {
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distances = new HashMap<>();
        
        // Start from all edge cells (potential entry points)
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                if (level.isWall(r, c)) continue;
                Position pos = new Position(r, c);
                if (avoid.contains(pos)) continue;
                
                // Check if this is an "open" cell (has space to maneuver)
                int neighbors = countFreeNeighbors(pos, level);
                if (neighbors >= 3) {
                    queue.add(pos);
                    distances.put(pos, 0);
                }
            }
        }
        
        // If no open cells, start from all free cells
        if (queue.isEmpty()) {
            for (int r = 0; r < level.getRows(); r++) {
                for (int c = 0; c < level.getCols(); c++) {
                    if (level.isWall(r, c)) continue;
                    Position pos = new Position(r, c);
                    if (avoid.contains(pos)) continue;
                    queue.add(pos);
                    distances.put(pos, 0);
                }
            }
        }
        
        // BFS to target
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int dist = distances.get(current);
            
            if (current.equals(target)) {
                return dist;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (level.isWall(next) || avoid.contains(next) || distances.containsKey(next)) {
                    continue;
                }
                distances.put(next, dist + 1);
                queue.add(next);
            }
        }
        
        return Integer.MAX_VALUE;
    }
    
    // ========== Cycle Detection ==========
    
    /**
     * Detects if there's a circular dependency among goals.
     */
    private static boolean detectCycle(Map<Position, Set<Position>> blockedBy, List<Position> goals) {
        Set<Position> visited = new HashSet<>();
        Set<Position> inStack = new HashSet<>();
        
        for (Position goal : goals) {
            if (hasCycleDFS(goal, blockedBy, visited, inStack)) {
                return true;
            }
        }
        return false;
    }
    
    private static boolean hasCycleDFS(Position node, Map<Position, Set<Position>> blockedBy,
                                       Set<Position> visited, Set<Position> inStack) {
        if (inStack.contains(node)) return true;
        if (visited.contains(node)) return false;
        
        visited.add(node);
        inStack.add(node);
        
        for (Position blocker : blockedBy.getOrDefault(node, Collections.emptySet())) {
            if (hasCycleDFS(blocker, blockedBy, visited, inStack)) {
                return true;
            }
        }
        
        inStack.remove(node);
        return false;
    }
    
    // ========== Topological Sort ==========
    
    /**
     * Returns goals in execution order (dependencies first).
     */
    private static List<Position> topologicalSort(Map<Position, Set<Position>> blockedBy, List<Position> goals) {
        List<Position> result = new ArrayList<>();
        Set<Position> visited = new HashSet<>();
        
        for (Position goal : goals) {
            topologicalDFS(goal, blockedBy, visited, result);
        }
        
        return result;
    }
    
    private static void topologicalDFS(Position node, Map<Position, Set<Position>> blockedBy,
                                       Set<Position> visited, List<Position> result) {
        if (visited.contains(node)) return;
        visited.add(node);
        
        // Visit all blockers first (they must be completed before this goal)
        for (Position blocker : blockedBy.getOrDefault(node, Collections.emptySet())) {
            topologicalDFS(blocker, blockedBy, visited, result);
        }
        
        result.add(node);
    }
    
    /**
     * Computes maximum depth in dependency graph.
     */
    private static int computeMaxDependencyDepth(Map<Position, Set<Position>> blockedBy, List<Position> goals) {
        Map<Position, Integer> depths = new HashMap<>();
        int maxDepth = 0;
        
        for (Position goal : goals) {
            int depth = computeDepth(goal, blockedBy, depths);
            maxDepth = Math.max(maxDepth, depth);
        }
        
        return maxDepth;
    }
    
    private static int computeDepth(Position node, Map<Position, Set<Position>> blockedBy,
                                    Map<Position, Integer> cache) {
        if (cache.containsKey(node)) return cache.get(node);
        
        int maxBlockerDepth = 0;
        for (Position blocker : blockedBy.getOrDefault(node, Collections.emptySet())) {
            maxBlockerDepth = Math.max(maxBlockerDepth, computeDepth(blocker, blockedBy, cache) + 1);
        }
        
        cache.put(node, maxBlockerDepth);
        return maxBlockerDepth;
    }
    
    // ========== Structural Analysis ==========
    
    private static int countFreeSpaces(Level level) {
        int count = 0;
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                if (!level.isWall(r, c)) count++;
            }
        }
        return count;
    }
    
    private static int[] countCorridorsAndJunctions(Level level) {
        int corridors = 0;
        int junctions = 0;
        
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                if (level.isWall(r, c)) continue;
                
                int neighbors = countFreeNeighbors(new Position(r, c), level);
                if (neighbors <= 2) corridors++;
                else if (neighbors >= 3) junctions++;
            }
        }
        
        return new int[]{corridors, junctions};
    }
    
    private static int countFreeNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position next = pos.move(dir);
            if (!level.isWall(next)) count++;
        }
        return count;
    }
    
    private static List<Position> findAllGoalPositions(Level level) {
        List<Position> goals = new ArrayList<>();
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                if (level.getBoxGoal(r, c) != '\0' || level.getAgentGoal(r, c) >= 0) {
                    goals.add(new Position(r, c));
                }
            }
        }
        return goals;
    }
    
    // ========== Strategy Recommendation ==========
    
    private static StrategyType recommendStrategy(int numAgents, int maxDepth, 
                                                   boolean hasCycle, double corridorRatio) {
        if (numAgents == 1) {
            return StrategyType.SINGLE_AGENT;
        }
        
        if (hasCycle) {
            return StrategyType.CYCLE_BREAKER;
        }
        
        if (maxDepth >= 3 || corridorRatio > 0.5) {
            // Strong sequential dependencies or corridor-heavy level
            return StrategyType.STRICT_ORDER;
        }
        
        if (numAgents <= 3) {
            return StrategyType.JOINT_SEARCH;
        }
        
        return StrategyType.GREEDY_WITH_RETRY;
    }
    
    // ========== Report Generation ==========
    
    private static String generateReport(int numAgents, int numBoxes, int numGoals, int freeSpaces,
                                        TaskFilter.FilterResult taskFilter,
                                        Map<Position, Set<Position>> blockedBy, int maxDepth,
                                        boolean hasCycle, int corridors, int junctions,
                                        StrategyType recommended) {
        StringBuilder sb = new StringBuilder();
        sb.append("\n========== LEVEL ANALYSIS ==========\n");
        sb.append(String.format("Agents: %d, Boxes: %d, Active goals: %d\n", numAgents, numBoxes, numGoals));
        
        // Task filter summary
        if (!taskFilter.immovableBoxes.isEmpty()) {
            sb.append(String.format("Immovable boxes (walls): %d\n", taskFilter.immovableBoxes.size()));
        }
        if (!taskFilter.satisfiedGoals.isEmpty()) {
            sb.append(String.format("Already satisfied: %d\n", taskFilter.satisfiedGoals.size()));
        }
        
        sb.append(String.format("Free spaces: %d, Density: %.2f%%\n", 
                               freeSpaces, (numAgents + numBoxes) * 100.0 / freeSpaces));
        sb.append(String.format("Corridors: %d (%.1f%%), Junctions: %d\n", 
                               corridors, corridors * 100.0 / freeSpaces, junctions));
        sb.append(String.format("Max dependency depth: %d, Has cycle: %s\n", maxDepth, hasCycle));
        
        // Show dependencies if any
        int depCount = blockedBy.values().stream().mapToInt(Set::size).sum();
        if (depCount > 0) {
            sb.append("\nGoal dependencies (").append(depCount).append(" total):\n");
            for (Map.Entry<Position, Set<Position>> entry : blockedBy.entrySet()) {
                if (!entry.getValue().isEmpty()) {
                    sb.append("  ").append(entry.getKey()).append(" blocked by: ");
                    sb.append(entry.getValue()).append("\n");
                }
            }
        }
        
        sb.append("\nRecommended strategy: ").append(recommended).append("\n");
        sb.append("=====================================\n");
        
        return sb.toString();
    }
}
