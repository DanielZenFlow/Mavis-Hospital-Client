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
        
        // 4. Goal dependency analysis - includes Box-on-Goal dependencies
        Map<Position, Set<Position>> goalBlockedBy = computeGoalDependencies(activeGoals, level, state);
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
     * Computes goal dependencies using MAPF standard "Push Reachability" analysis.
     * 
     * Dependency rule: Goal A depends on Goal B if:
     *   - Currently we CAN push a box to A
     *   - But if B is filled first, we can NO LONGER push a box to A
     * 
     * This correctly handles nested/corner goals without hardcoded thresholds.
     */
    private static Map<Position, Set<Position>> computeGoalDependencies(
            List<Position> goals, Level level, State state) {
        Map<Position, Set<Position>> blockedBy = new HashMap<>();
        
        // Build goal type map for quick lookup
        Map<Position, Character> goalTypes = new HashMap<>();
        for (Position goal : goals) {
            char type = level.getBoxGoal(goal);
            if (type != '\0') {
                goalTypes.put(goal, type);
            }
        }
        
        // Separate box goals from agent goals
        List<Position> boxGoals = new ArrayList<>();
        Set<Position> boxGoalSet = new HashSet<>();
        for (Position goal : goals) {
            if (goalTypes.containsKey(goal)) {
                boxGoals.add(goal);
                boxGoalSet.add(goal);
            }
        }
        
        for (Position goalA : goals) {
            blockedBy.put(goalA, new HashSet<>());
            char goalAType = goalTypes.getOrDefault(goalA, '\0');
            
            // Check 1: Box-on-Goal dependency (a box of wrong type sits on this goal)
            // Only triggers when there IS a box at the goal, and it's the WRONG type
            char boxAtGoal = state.getBoxAt(goalA);
            if (boxAtGoal != '\0' && boxAtGoal != goalAType) {
                // A wrong-type box is sitting on this goal - find where that box type should go
                for (Position goalX : boxGoals) {  // Only box goals can be blockers
                    char goalXType = goalTypes.getOrDefault(goalX, '\0');
                    if (goalXType == boxAtGoal) {
                        blockedBy.get(goalA).add(goalX);
                    }
                }
            }
            
            // Check 2: Push Position Overlap - detect which goals block other goals' push access
            // If goalB occupies a position needed to push to goalA, then B depends on A
            if (boxGoalSet.contains(goalA)) {
                // Find all valid push positions for goalA (where agent must stand to push)
                Set<Position> pushPositions = getPushPositions(goalA, level);
                
                // If any push position is also a box goal, that goal depends on goalA
                for (Position pushPos : pushPositions) {
                    if (boxGoalSet.contains(pushPos) && !pushPos.equals(goalA)) {
                        // pushPos is a goal that would block pushing to goalA
                        // So pushPos (goalB) depends on goalA - goalA must be filled first
                        blockedBy.computeIfAbsent(pushPos, k -> new HashSet<>()).add(goalA);
                    }
                }
            }
            
            // Check 3: Agent goal blocked by box goals on path (only box goals can block agent goals)
            if (!boxGoalSet.contains(goalA)) {
                int directDist = bfsDistanceFromEdge(goalA, level, Collections.emptySet());
                for (Position goalX : boxGoals) {  // Only check box goals as blockers
                    if (goalX.equals(goalA)) continue;
                    int avoidDist = bfsDistanceFromEdge(goalA, level, Collections.singleton(goalX));
                    if (avoidDist > directDist) {
                        blockedBy.get(goalA).add(goalX);
                    }
                }
            }
        }
        
        return blockedBy;
    }
    
    /**
     * Get all valid push positions for a goal (positions where agent stands AFTER pushing box to goal).
     * Push mechanics: agent at (goal+2*dir) pushes box at (goal+dir) into goal.
     * After push: agent ends at (goal+dir), box ends at goal.
     */
    private static Set<Position> getPushPositions(Position goal, Level level) {
        Set<Position> pushPositions = new HashSet<>();
        for (Direction dir : Direction.values()) {
            Position boxFromPos = goal.move(dir);         // Box starts here, agent ends here after push
            Position agentFromPos = boxFromPos.move(dir); // Agent starts here before push
            
            // Both positions must be non-wall for this push direction to be valid
            if (!level.isWall(boxFromPos) && !level.isWall(agentFromPos)) {
                pushPositions.add(boxFromPos);
            }
        }
        return pushPositions;
    }
    
    /**
     * MAPF standard: Check if a box can be pushed to the goal position.
     * A push requires: agent at one adjacent cell, box comes from opposite direction.
     * Returns true if at least one push direction is available.
     */
    private static boolean canPushToGoal(Position goal, Level level, Set<Position> filledGoals) {
        for (Direction dir : Direction.values()) {
            Position agentPos = goal.move(dir);  // Agent stands here to push
            Position boxFromPos = goal.move(dir.opposite());  // Box comes from here
            
            // Agent position must be valid and not filled
            if (level.isWall(agentPos) || filledGoals.contains(agentPos)) {
                continue;
            }
            
            // Box source position must be valid (not wall)
            if (level.isWall(boxFromPos)) {
                continue;
            }
            
            // Check if agent can reach agentPos (avoiding filled goals as obstacles)
            if (canReachPosition(agentPos, level, filledGoals)) {
                return true;  // At least one push direction works
            }
        }
        return false;
    }
    
    /**
     * Check if a position is reachable from any open area, avoiding filled goals.
     */
    private static boolean canReachPosition(Position target, Level level, Set<Position> obstacles) {
        if (obstacles.contains(target)) return false;
        
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        // Start BFS from target, try to reach any "open" cell (>= 3 free neighbors)
        queue.add(target);
        visited.add(target);
        
        while (!queue.isEmpty()) {
            Position curr = queue.poll();
            
            // Check if this is an open area
            int freeNeighbors = 0;
            for (Direction d : Direction.values()) {
                Position n = curr.move(d);
                if (!level.isWall(n) && !obstacles.contains(n)) {
                    freeNeighbors++;
                }
            }
            if (freeNeighbors >= 3) {
                return true;  // Reached open area
            }
            
            // Continue BFS
            for (Direction d : Direction.values()) {
                Position next = curr.move(d);
                if (level.isWall(next) || obstacles.contains(next) || visited.contains(next)) {
                    continue;
                }
                visited.add(next);
                queue.add(next);
            }
        }
        
        return false;
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
            int depth = computeDepth(goal, blockedBy, depths, new HashSet<>());
            maxDepth = Math.max(maxDepth, depth);
        }
        
        return maxDepth;
    }
    
    private static int computeDepth(Position node, Map<Position, Set<Position>> blockedBy,
                                    Map<Position, Integer> cache, Set<Position> visiting) {
        if (cache.containsKey(node)) return cache.get(node);
        if (visiting.contains(node)) return 0; // Cycle detected, break it
        
        visiting.add(node);
        int maxBlockerDepth = 0;
        for (Position blocker : blockedBy.getOrDefault(node, Collections.emptySet())) {
            maxBlockerDepth = Math.max(maxBlockerDepth, computeDepth(blocker, blockedBy, cache, visiting) + 1);
        }
        visiting.remove(node);
        
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
