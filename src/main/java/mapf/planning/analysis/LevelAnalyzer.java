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
        
        // MAPF FIX: Bottleneck detection (path intersection points)
        public final Set<Position> bottleneckPositions;  // positions that block multiple paths
        public final Map<Position, Integer> bottleneckScores;  // position -> number of paths blocked
        
        // MAPF FIX: Coupling degree (for strategy selection)
        public final double couplingDegree;  // 0.0 = independent, 1.0 = fully coupled
        
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
                            Set<Position> bottlenecks, Map<Position, Integer> bottleneckScores,
                            double couplingDegree,
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
            this.bottleneckPositions = bottlenecks;
            this.bottleneckScores = bottleneckScores;
            this.couplingDegree = couplingDegree;
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
        // CRITICAL FIX: Pass immovable boxes so they are treated as walls, not movable obstacles
        Map<Position, Set<Position>> goalDependsOn = computeGoalDependencies(activeGoals, level, state, taskFilter.immovableBoxes);
        boolean hasCycle = detectCycle(goalDependsOn, activeGoals);
        
        // 5. MAPF FIX: Bottleneck detection (path intersection analysis)
        Map<Position, Integer> bottleneckScores = detectBottlenecks(activeGoals, level, state);
        Set<Position> bottlenecks = new HashSet<>();
        for (Map.Entry<Position, Integer> e : bottleneckScores.entrySet()) {
            if (e.getValue() >= 2) {  // Position blocks 2+ paths
                bottlenecks.add(e.getKey());
            }
        }
        
        // 6. MAPF FIX: Compute coupling degree for strategy selection
        double couplingDegree = computeCouplingDegree(goalDependsOn, bottleneckScores, 
                                                      numAgents, numGoals, activeGoals);
        
        // 7. Adjust execution order: prioritize goals AT bottleneck positions
        // NEW GENERIC FIX: Use BFS distance ("fill from back") for topological sort tie-breaking
        Position root = findOpenSpaceRoot(level, activeGoals, taskFilter.immovableBoxes);
        Map<Position, Integer> distances = (root != null) ? 
            computeDistancesFromRoot(level, root, taskFilter.immovableBoxes) : new HashMap<>();
            
        // MAPF FIX: Removed adjustOrderForBottlenecks. 
        // Logic was flawed (executing bottleneck goals first blocks the path).
        // Reachability analysis (computeGoalDependencies) already handles blocking correctly so
        // we should rely purely on topological sort + distance tie-breaking.
        List<Position> executionOrder = hasCycle ? activeGoals : 
            topologicalSort(goalDependsOn, activeGoals, distances);
        int maxDepth = computeMaxDependencyDepth(goalDependsOn, activeGoals);
        
        // 8. Strategy recommendation (now uses coupling degree)
        StrategyType recommended = recommendStrategy(numAgents, maxDepth, hasCycle, 
                                                     couplingDegree, bottlenecks.size());
        
        // 9. Generate report
        String report = generateReport(numAgents, numBoxes, numGoals, freeSpaces, taskFilter,
                                       goalDependsOn, maxDepth, hasCycle, 
                                       corridorCells, junctionCells, recommended);
        
        return new LevelFeatures(numAgents, numBoxes, numGoals, freeSpaces, taskFilter,
                                goalDependsOn, executionOrder, maxDepth, hasCycle,
                                bottlenecks, bottleneckScores, couplingDegree,
                                corridorCells, junctionCells, recommended, report);
    }
    
    // ========== Goal Dependency Analysis ==========
    
    /**
     * Reachability-Based Goal Dependency Analysis (非硬编码方法)
     * 
     * 核心思想（来自Gemini/SIW理论）：
     * 不使用BFS层级硬编码，而是通过"假设测试"动态发现依赖关系。
     * 
     * 算法：
     * 对于每对目标 (Gi, Gj)：
     *   假设在 Gi 位置放置障碍物（模拟 Gi 已被box占据）
     *   检测从开放空间是否还能到达 Gj
     *   如果不可达 → Gj 必须在 Gi 之前完成 → Gi 依赖于 Gj
     * 
     * 这种方法的优点：
     * - 不依赖几何坐标或层级
     * - 基于因果逻辑（Causality）
     * - 适用于任何地图结构（死胡同、桥梁、机关门等）
     */
    private static Map<Position, Set<Position>> computeGoalDependencies(
            List<Position> goals, Level level, State state, Set<Position> immovableBoxes) {
        Map<Position, Set<Position>> dependsOn = new HashMap<>();
        
        // Initialize dependency map
        for (Position g : goals) {
            dependsOn.put(g, new HashSet<>());
        }
        
        if (goals.isEmpty()) return dependsOn;
        
        // Find root position representing "Main Open Space" (agent accessible area)
        // CRITICAL FIX: Treat immovable boxes as walls when finding open space root
        Position root = findOpenSpaceRoot(level, goals, immovableBoxes);
        if (root == null) {
            System.err.println("[LevelAnalyzer] Warning: No open space root found");
            return dependsOn;
        }
        
        System.err.println("[LevelAnalyzer] Reachability-based dependency analysis");
        System.err.println("[LevelAnalyzer] Open space root: " + root);
        System.err.println("[LevelAnalyzer] Testing " + goals.size() + " goals...");
        System.err.println("[LevelAnalyzer] Immovable boxes (treated as walls): " + immovableBoxes.size());
        
        // Get MOVABLE box positions only (exclude immovable boxes - they are permanent walls)
        Set<Position> movableBoxPositions = new HashSet<>();
        for (Position boxPos : state.getBoxes().keySet()) {
            if (!immovableBoxes.contains(boxPos)) {
                movableBoxPositions.add(boxPos);
            }
        }
        
        // Build dependency graph using hypothetical reachability testing
        // For each pair (Gi, Gj): if blocking Gi makes Gj unreachable, then Gi depends on Gj
        // 
        // KEY INSIGHT: We need to check if a BOX can be PUSHED into Gj, not just agent reachability.
        // A box can be pushed into Gj if agent can reach a position adjacent to Gj 
        // (to push from) AND there's a path for the box.
        
        int dependencyCount = 0;
        for (Position goalI : goals) {
            for (Position goalJ : goals) {
                if (goalI.equals(goalJ)) continue;
                
                // Hypothetical test: if goalI is filled, can we still push a box into goalJ?
                // This requires checking if there's at least one push-entry direction available
                // CRITICAL FIX: Use movableBoxPositions (excludes immovable boxes) and pass immovableBoxes
                boolean canPushToGoalJ = canPushBoxToGoalWithBlock(goalJ, goalI, level, movableBoxPositions, immovableBoxes, root);
                
                if (!canPushToGoalJ) {
                    // Cannot push to goalJ when goalI is blocked
                    // Therefore: goalJ must be completed BEFORE goalI
                    // In dependency terms: goalI depends on goalJ
                    dependsOn.get(goalI).add(goalJ);
                    dependencyCount++;
                    
                    char charI = level.getBoxGoal(goalI.row, goalI.col);
                    char charJ = level.getBoxGoal(goalJ.row, goalJ.col);
                    System.err.println("[DEBUG] " + goalI + "(" + charI + ") depends on " + 
                                       goalJ + "(" + charJ + ") - blocking " + goalI + " makes " + goalJ + " unreachable for push");
                }
            }
        }
        System.err.println("[LevelAnalyzer] Found " + dependencyCount + " dependencies");
        
        // Debug: Log dependency graph
        System.err.println("[LevelAnalyzer] Dependency graph (A depends on B means B executes first):");
        for (Position goal : goals) {
            Set<Position> deps = dependsOn.get(goal);
            if (deps != null && !deps.isEmpty()) {
                char goalChar = level.getBoxGoal(goal.row, goal.col);
                StringBuilder sb = new StringBuilder();
                sb.append("[LevelAnalyzer] Dep: ").append(goal).append("(").append(goalChar).append(") depends on: ");
                for (Position dep : deps) {
                    char depChar = level.getBoxGoal(dep.row, dep.col);
                    sb.append(dep).append("(").append(depChar).append(") ");
                }
                System.err.println(sb.toString());
            }
        }
        
        return dependsOn;
    }
    
    /**
     * Checks if a box can be pushed into goalPos when blockedPos is occupied.
     * 
     * For push operation to work, agent needs to:
     * 1. Be adjacent to the box (from the opposite side of push direction)
     * 2. Push the box into the goal
     * 
     * So we check: is there at least one direction from which agent can approach
     * the goal's adjacent cell (push position) while the block is in place?
     * 
     * CRITICAL: immovableBoxes are treated as permanent walls.
     */
    private static boolean canPushBoxToGoalWithBlock(Position goalPos, Position blockedPos,
            Level level, Set<Position> movableBoxPositions, Set<Position> immovableBoxes, Position openSpaceRoot) {
        
        // For each possible push direction into the goal
        for (Direction pushDir : Direction.values()) {
            // Agent would be on the opposite side of the goal
            Position agentPushPosition = goalPos.move(pushDir.opposite());
            
            // The box would come from where the agent is, pushed into goal
            // Agent needs to reach agentPushPosition to push
            
            if (level.isWall(agentPushPosition)) continue;
            if (immovableBoxes.contains(agentPushPosition)) continue;  // Immovable box = wall
            if (agentPushPosition.equals(blockedPos)) continue;  // Blocked
            
            // Check if agent can reach the push position with the block in place
            boolean canReach = isReachableWithHypotheticalBlock(
                openSpaceRoot, agentPushPosition, level, blockedPos, movableBoxPositions, immovableBoxes);
            
            if (canReach) {
                return true;  // At least one push direction works
            }
        }
        
        return false;  // No valid push direction available
    }
    
    /**
     * Checks if target is reachable from start with a hypothetical block at blockedPos.
     * This simulates "if blockedPos is filled with a box, can we still reach target?"
     * 
     * CRITICAL: immovableBoxes are treated as permanent walls (not passable).
     */
    private static boolean isReachableWithHypotheticalBlock(Position start, Position target, 
            Level level, Position blockedPos, Set<Position> movableBoxPositions, Set<Position> immovableBoxes) {
        if (start.equals(target)) return true;
        if (target.equals(blockedPos)) return false;  // Target itself is blocked
        if (immovableBoxes.contains(target)) return false;  // Target is an immovable box (wall)
        
        Queue<Position> q = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        q.add(start);
        visited.add(start);
        visited.add(blockedPos);  // Treat hypothetically blocked position as impassable
        visited.addAll(immovableBoxes);  // Treat all immovable boxes as walls
        
        while (!q.isEmpty()) {
            Position current = q.poll();
            if (current.equals(target)) return true;
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                // Note: movableBoxPositions are NOT blocking agent movement for reachability
                // because the agent can push/pull them out of the way.
                // We only block on: walls, hypothetical block, and immovable boxes (already in visited)
                if (!level.isWall(next) && !visited.contains(next)) {
                    visited.add(next);
                    q.add(next);
                }
            }
        }
        return false;
    }
    
    /**
     * Finds a root position in the main open space (largest connected component).
     * CRITICAL: immovableBoxes are treated as permanent walls.
     */
    private static Position findOpenSpaceRoot(Level level, List<Position> goals, Set<Position> immovableBoxes) {
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
                if (immovableBoxes.contains(p)) continue;  // Treat immovable boxes as walls
                
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
                        if (!level.isWall(next) && !visited.contains(next) && !immovableBoxes.contains(next)) {
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
     * 
     * Uses two-level sorting:
     * 1. Topological Importance (Dependency-based)
     * 2. BFS Distance from Root (Distance-based): Furthest goals first
     */
    private static List<Position> topologicalSort(Map<Position, Set<Position>> dependsOn, List<Position> goals, Map<Position, Integer> distances) {
        List<Position> result = new ArrayList<>();
        Set<Position> visited = new HashSet<>();
        
        // Build reverse dependency map: who depends on each goal
        Map<Position, Set<Position>> dependedBy = new HashMap<>();
        for (Position goal : goals) {
            dependedBy.put(goal, new HashSet<>());
        }
        for (Map.Entry<Position, Set<Position>> entry : dependsOn.entrySet()) {
            for (Position dep : entry.getValue()) {
                if (dependedBy.containsKey(dep)) {
                    dependedBy.get(dep).add(entry.getKey());
                }
            }
        }
        
        // Calculate "importance": how many goals (transitively) depend on this goal
        Map<Position, Integer> importance = new HashMap<>();
        for (Position goal : goals) {
            computeImportance(goal, dependedBy, importance, new HashSet<>());
        }
        
        // Sort goals by importance (most important = most depended upon = first)
        // Tie-breaker: Distance from root (Descending) -> Fill deepest rooms first
        List<Position> sortedGoals = new ArrayList<>(goals);
        sortedGoals.sort((a, b) -> {
            int impA = importance.getOrDefault(a, 0);
            int impB = importance.getOrDefault(b, 0);
            if (impA != impB) {
                return Integer.compare(impB, impA);  // Higher importance first
            }
            // Tie-breaker: Furthest distance first
            int distA = distances.getOrDefault(a, 0);
            int distB = distances.getOrDefault(b, 0);
            return Integer.compare(distB, distA);
        });
        
        // Now do topological DFS - dependencies visited before dependents
        for (Position goal : sortedGoals) {
            topologicalDFS(goal, dependsOn, visited, result);
        }
        
        // DEBUG: Log topological sort result
        System.err.println("[LevelAnalyzer] Topological sort result:");
        for (int i = 0; i < result.size(); i++) {
            Position p = result.get(i);
            int dist = distances.getOrDefault(p, -1);
            System.err.println("[LevelAnalyzer] Topo: " + (i+1) + ". " + p + " (Dist: " + dist + ")");
        }
        
        return result;
    }

    /**
     * Computes BFS distances from the root to all reachable cells.
     */
    private static Map<Position, Integer> computeDistancesFromRoot(Level level, Position root, Set<Position> immovableBoxes) {
        Map<Position, Integer> distances = new HashMap<>();
        if (root == null) return distances;
        
        Queue<Position> q = new LinkedList<>();
        q.add(root);
        distances.put(root, 0);
        
        while (!q.isEmpty()) {
            Position curr = q.poll();
            int dist = distances.get(curr);
            
            for (Direction dir : Direction.values()) {
                Position next = curr.move(dir);
                if (!level.isWall(next) && !immovableBoxes.contains(next) && !distances.containsKey(next)) {
                    distances.put(next, dist + 1);
                    q.add(next);
                }
            }
        }
        return distances;
    }
    
    /**
     * Compute how many goals (transitively) depend on this goal.
     * Higher value = this goal is more "important" and should be filled earlier.
     */
    private static int computeImportance(Position node, Map<Position, Set<Position>> dependedBy,
                                         Map<Position, Integer> cache, Set<Position> visiting) {
        if (cache.containsKey(node)) return cache.get(node);
        if (visiting.contains(node)) return 0;
        
        visiting.add(node);
        
        int count = 0;
        Set<Position> dependents = dependedBy.getOrDefault(node, Collections.emptySet());
        count += dependents.size();  // Direct dependents
        
        for (Position dep : dependents) {
            count += computeImportance(dep, dependedBy, cache, visiting);  // Transitive
        }
        
        visiting.remove(node);
        cache.put(node, count);
        return count;
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
    
    // ========== MAPF FIX: Bottleneck Detection ==========
    
    /**
     * Detects bottleneck positions by analyzing paths from boxes to goals.
     * A bottleneck is a position that appears on multiple box-to-goal paths.
     * 
     * @return Map of position -> number of paths passing through it
     */
    private static Map<Position, Integer> detectBottlenecks(List<Position> goals, Level level, State state) {
        Map<Position, Integer> passCount = new HashMap<>();
        
        // For each goal, find the nearest matching box and trace the path
        for (Position goal : goals) {
            char goalType = level.getBoxGoal(goal);
            if (goalType == '\0') continue;
            
            // Find nearest box of this type
            Position nearestBox = null;
            int minDist = Integer.MAX_VALUE;
            for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
                if (entry.getValue() == goalType) {
                    int dist = manhattanDistance(entry.getKey(), goal);
                    if (dist < minDist) {
                        minDist = dist;
                        nearestBox = entry.getKey();
                    }
                }
            }
            
            if (nearestBox == null) continue;
            
            // BFS to find path from box to goal
            List<Position> path = findPath(nearestBox, goal, level, state);
            if (path != null) {
                // Count how many paths pass through each position
                for (Position p : path) {
                    passCount.merge(p, 1, Integer::sum);
                }
            }
        }
        
        return passCount;
    }
    
    /**
     * BFS pathfinding from start to target, avoiding walls.
     */
    private static List<Position> findPath(Position start, Position target, Level level, State state) {
        if (start.equals(target)) return Collections.singletonList(start);
        
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> parent = new HashMap<>();
        queue.add(start);
        parent.put(start, null);
        
        while (!queue.isEmpty()) {
            Position curr = queue.poll();
            
            if (curr.equals(target)) {
                // Reconstruct path
                List<Position> path = new ArrayList<>();
                Position p = curr;
                while (p != null) {
                    path.add(0, p);
                    p = parent.get(p);
                }
                return path;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = curr.move(dir);
                if (!level.isWall(next) && !parent.containsKey(next)) {
                    parent.put(next, curr);
                    queue.add(next);
                }
            }
        }
        return null;  // No path found
    }
    
    private static int manhattanDistance(Position a, Position b) {
        return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
    }
    
    /**
     * Adjusts goal execution order: goals AT high-traffic bottleneck positions
     * should be completed earlier to clear the bottleneck.
     */
    private static List<Position> adjustOrderForBottlenecks(List<Position> baseOrder, 
                                                            Map<Position, Integer> bottleneckScores) {
        if (bottleneckScores.isEmpty()) return baseOrder;
        
        List<Position> adjusted = new ArrayList<>(baseOrder);
        
        // Sort: higher bottleneck score = earlier execution (to clear the position)
        adjusted.sort((a, b) -> {
            int scoreA = bottleneckScores.getOrDefault(a, 0);
            int scoreB = bottleneckScores.getOrDefault(b, 0);
            // Higher score first (descending)
            if (scoreA != scoreB) return Integer.compare(scoreB, scoreA);
            // Otherwise keep original order
            return Integer.compare(baseOrder.indexOf(a), baseOrder.indexOf(b));
        });
        
        return adjusted;
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
    
    // ========== MAPF FIX: Coupling Degree Calculation ==========
    
    /**
     * Computes coupling degree between agents/goals.
     * 
     * Coupling degree is a value between 0.0 and 1.0:
     * - 0.0: Fully independent (agents don't interact)
     * - 1.0: Fully coupled (every goal depends on others)
     * 
     * Factors considered:
     * 1. Dependency ratio: how many goals have dependencies
     * 2. Bottleneck ratio: how many bottleneck positions exist relative to goals
     * 3. Agent density in bottlenecks: multiple agents needing same bottleneck
     */
    private static double computeCouplingDegree(Map<Position, Set<Position>> goalDependsOn,
                                                Map<Position, Integer> bottleneckScores,
                                                int numAgents, int numGoals,
                                                List<Position> activeGoals) {
        if (numGoals == 0 || numAgents <= 1) return 0.0;
        
        // Factor 1: Dependency ratio (goals with dependencies / total goals)
        int goalsWithDeps = 0;
        int totalDeps = 0;
        for (Set<Position> deps : goalDependsOn.values()) {
            if (!deps.isEmpty()) {
                goalsWithDeps++;
                totalDeps += deps.size();
            }
        }
        double dependencyRatio = (double) goalsWithDeps / numGoals;
        
        // Factor 2: Average dependencies per goal
        double avgDeps = (double) totalDeps / Math.max(1, numGoals);
        double depIntensity = Math.min(1.0, avgDeps / 3.0);  // Normalize: 3+ deps = 1.0
        
        // Factor 3: Bottleneck intensity (how congested are bottlenecks)
        int highTrafficBottlenecks = 0;
        for (int score : bottleneckScores.values()) {
            if (score >= 3) highTrafficBottlenecks++;  // Position used by 3+ paths
        }
        double bottleneckRatio = (double) highTrafficBottlenecks / Math.max(1, numGoals);
        
        // Weighted combination
        double coupling = 0.4 * dependencyRatio + 0.3 * depIntensity + 0.3 * bottleneckRatio;
        return Math.min(1.0, coupling);
    }
    
    // ========== Strategy Recommendation ==========
    
    /**
     * MAPF FIX: Strategy recommendation now uses coupling degree.
     * 
     * - Low coupling (< 0.2): Independent planning (PP) works well
     * - Medium coupling (0.2-0.5): CBS recommended for optimal conflict resolution
     * - High coupling (> 0.5): Joint search or strict ordering required
     */
    private static StrategyType recommendStrategy(int numAgents, int maxDepth, 
                                                   boolean hasCycle, double couplingDegree,
                                                   int numBottlenecks) {
        if (numAgents == 1) {
            return StrategyType.SINGLE_AGENT;
        }
        
        if (hasCycle) {
            return StrategyType.CYCLE_BREAKER;
        }
        
        // MAPF FIX: Use coupling degree for strategy selection
        if (couplingDegree > 0.5 || maxDepth >= 3) {
            // High coupling: need strict ordering
            return StrategyType.STRICT_ORDER;
        }
        
        if (couplingDegree > 0.2 && numAgents <= 4) {
            // Medium coupling with few agents: joint search handles conflicts well
            return StrategyType.JOINT_SEARCH;
        }
        
        if (numAgents <= 3 && numBottlenecks == 0) {
            // Few agents, no bottlenecks: joint search is optimal
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
