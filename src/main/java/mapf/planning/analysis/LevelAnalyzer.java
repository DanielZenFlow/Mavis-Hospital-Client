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
        public final Map<Position, Set<Position>> goalDependsOn;  // goal -> goals it depends on
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
                            Map<Position, Set<Position>> goalDependsOn,
                            List<Position> executionOrder, int maxDepth, boolean hasCycle,
                            int corridorCells, int junctionCells,
                            StrategyType recommended, String report) {
            this.numAgents = numAgents;
            this.numBoxes = numBoxes;
            this.numGoals = numGoals;
            this.freeSpaces = freeSpaces;
            this.density = (double)(numAgents + numBoxes) / Math.max(1, freeSpaces);
            this.taskFilter = taskFilter;
            this.goalDependsOn = goalDependsOn;
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
        
        // 4. Goal dependency analysis - includes Box-on-Goal dependencies
        Map<Position, Set<Position>> goalDependsOn = computeGoalDependencies(activeGoals, level, state);
        boolean hasCycle = detectCycle(goalDependsOn, activeGoals);
        List<Position> executionOrder = hasCycle ? activeGoals : topologicalSort(goalDependsOn, activeGoals);
        int maxDepth = computeMaxDependencyDepth(goalDependsOn, activeGoals);
        
        // 5. Strategy recommendation
        StrategyType recommended = recommendStrategy(numAgents, maxDepth, hasCycle, 
                                                     (double) corridorCells / Math.max(1, freeSpaces));
        
        // 6. Generate report
        String report = generateReport(numAgents, numBoxes, numGoals, freeSpaces, taskFilter,
                                       goalDependsOn, maxDepth, hasCycle, 
                                       corridorCells, junctionCells, recommended);
        
        return new LevelFeatures(numAgents, numBoxes, numGoals, freeSpaces, taskFilter,
                                goalDependsOn, executionOrder, maxDepth, hasCycle,
                                corridorCells, junctionCells, recommended, report);
    }
    
    // ========== Goal Dependency Analysis ==========
    
    /**
     * Computes goal dependencies using Global Reachability analysis.
     * 
     * MAPF Standard Terminology:
     *   - dependsOn.get(A) contains B means: A depends on B (B must complete before A)
     *   - If filling goal A blocks access to goal B from open space,
     *     then A depends on B (B must be filled first, while path is still open)
     *   - Topological sort outputs: dependencies first, then dependents
     */
    private static Map<Position, Set<Position>> computeGoalDependencies(
            List<Position> goals, Level level, State state) {
        Map<Position, Set<Position>> dependsOn = new HashMap<>();
        
        // Initialize dependency map
        for (Position g : goals) {
            dependsOn.put(g, new HashSet<>());
        }
        
        // Find root position representing "Main Open Space"
        Position root = findOpenSpaceRoot(level, goals);
        if (root == null) {
            return dependsOn; // Degenerate case
        }
        
        // Get current box positions as obstacles (MAPF: consider dynamic state)
        Set<Position> boxPositions = state.getBoxes().keySet();

        // Build dependency graph
        for (Position goalA : goals) {
            for (Position goalB : goals) {
                if (goalA.equals(goalB)) continue;
                
                // Check: Does filling goalA block access to goalB?
                // If yes, goalB must be filled BEFORE goalA.
                // So goalA depends on goalB.
                if (!isReachable(root, goalB, level, goalA, boxPositions)) {
                    dependsOn.get(goalA).add(goalB);
                }
            }
        }
        
        return dependsOn;
    }
    
    private static Position findOpenSpaceRoot(Level level, List<Position> goals) {
        Set<Position> goalSet = new HashSet<>(goals);
        Set<Position> visited = new HashSet<>();
        
        Position bestRoot = null;
        int maxComponentSize = -1;
        
        // Scan all cells to find the largest connected component
        for (int r=0; r<level.getRows(); r++) {
            for (int c=0; c<level.getCols(); c++) {
                if (level.isWall(r, c)) continue;
                Position p = new Position(r, c);
                if (visited.contains(p)) continue;
                
                // BFS to explore component
                int componentSize = 0;
                Position bestInComponent = null;
                int maxNeighbors = -1;
                
                Queue<Position> q = new LinkedList<>();
                q.add(p);
                visited.add(p);
                
                while (!q.isEmpty()) {
                    Position curr = q.poll();
                    componentSize++;
                    
                    // Track best candidate root in this component (non-goal, high degree)
                    if (!goalSet.contains(curr)) {
                        int neighbors = countFreeNeighbors(curr, level);
                        if (bestInComponent == null || neighbors > maxNeighbors) {
                            bestInComponent = curr;
                            maxNeighbors = neighbors;
                        }
                    }
                    
                    for (Direction dir : Direction.values()) {
                        Position next = curr.move(dir);
                        if (!level.isWall(next) && !visited.contains(next)) {
                            visited.add(next);
                            q.add(next);
                        }
                    }
                }
                
                // Keep the root from the largest component
                if (componentSize > maxComponentSize && bestInComponent != null) {
                    maxComponentSize = componentSize;
                    bestRoot = bestInComponent;
                }
            }
        }
        return bestRoot;
    }

    /**
     * BFS reachability check considering walls, boxes, and a hypothetically blocked position.
     * Per MAPF standard: dynamic obstacles (boxes) affect reachability analysis.
     */
    private static boolean isReachable(Position start, Position target, Level level, 
                                        Position blocked, Set<Position> boxPositions) {
        if (start.equals(target)) return true;
        
        Queue<Position> q = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        q.add(start);
        visited.add(start);
        visited.add(blocked); // Treat hypothetically filled goal as wall
        
        while (!q.isEmpty()) {
            Position current = q.poll();
            if (current.equals(target)) return true;
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                // Check: not wall, not visited, not a box (unless it's the target)
                if (!level.isWall(next) && !visited.contains(next) &&
                    (!boxPositions.contains(next) || next.equals(target))) {
                    visited.add(next);
                    q.add(next);
                }
            }
        }
        return false;
    }
    
    // ========== Cycle Detection ==========
    
    /**
     * Detects if there's a circular dependency among goals.
     */
    private static boolean detectCycle(Map<Position, Set<Position>> dependsOn, List<Position> goals) {
        Set<Position> visited = new HashSet<>();
        Set<Position> inStack = new HashSet<>();
        
        for (Position goal : goals) {
            if (hasCycleDFS(goal, dependsOn, visited, inStack)) {
                return true;
            }
        }
        return false;
    }
    
    private static boolean hasCycleDFS(Position node, Map<Position, Set<Position>> dependsOn,
                                       Set<Position> visited, Set<Position> inStack) {
        if (inStack.contains(node)) return true;
        if (visited.contains(node)) return false;
        
        visited.add(node);
        inStack.add(node);
        
        for (Position dep : dependsOn.getOrDefault(node, Collections.emptySet())) {
            if (hasCycleDFS(dep, dependsOn, visited, inStack)) {
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
    private static List<Position> topologicalSort(Map<Position, Set<Position>> dependsOn, List<Position> goals) {
        List<Position> result = new ArrayList<>();
        Set<Position> visited = new HashSet<>();
        
        for (Position goal : goals) {
            topologicalDFS(goal, dependsOn, visited, result);
        }
        
        return result;
    }
    
    private static void topologicalDFS(Position node, Map<Position, Set<Position>> dependsOn,
                                       Set<Position> visited, List<Position> result) {
        if (visited.contains(node)) return;
        visited.add(node);
        
        // Visit all dependencies first (they must be completed before this goal)
        for (Position dep : dependsOn.getOrDefault(node, Collections.emptySet())) {
            topologicalDFS(dep, dependsOn, visited, result);
        }
        
        result.add(node);
    }
    
    /**
     * Computes maximum depth in dependency graph.
     */
    private static int computeMaxDependencyDepth(Map<Position, Set<Position>> dependsOn, List<Position> goals) {
        Map<Position, Integer> depths = new HashMap<>();
        int maxDepth = 0;
        
        for (Position goal : goals) {
            int depth = computeDepth(goal, dependsOn, depths, new HashSet<>());
            maxDepth = Math.max(maxDepth, depth);
        }
        
        return maxDepth;
    }
    
    private static int computeDepth(Position node, Map<Position, Set<Position>> dependsOn,
                                    Map<Position, Integer> cache, Set<Position> visiting) {
        if (cache.containsKey(node)) return cache.get(node);
        if (visiting.contains(node)) return 0; // Cycle detected, break it
        
        visiting.add(node);
        int maxDepDepth = 0;
        for (Position dep : dependsOn.getOrDefault(node, Collections.emptySet())) {
            maxDepDepth = Math.max(maxDepDepth, computeDepth(dep, dependsOn, cache, visiting) + 1);
        }
        visiting.remove(node);
        
        cache.put(node, maxDepDepth);
        return maxDepDepth;
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
                                        Map<Position, Set<Position>> dependsOn, int maxDepth,
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
        int depCount = dependsOn.values().stream().mapToInt(Set::size).sum();
        if (depCount > 0) {
            sb.append("\nGoal dependencies (").append(depCount).append(" total):\n");
            for (Map.Entry<Position, Set<Position>> entry : dependsOn.entrySet()) {
                if (!entry.getValue().isEmpty()) {
                    sb.append("  ").append(entry.getKey()).append(" depends on: ");
                    sb.append(entry.getValue()).append("\n");
                }
            }
        }
        
        sb.append("\nRecommended strategy: ").append(recommended).append("\n");
        sb.append("=====================================\n");
        
        return sb.toString();
    }
}
