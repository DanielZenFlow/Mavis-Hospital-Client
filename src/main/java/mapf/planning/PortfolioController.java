package mapf.planning;

import mapf.domain.*;
import mapf.planning.analysis.LevelAnalyzer;
import mapf.planning.analysis.LevelAnalyzer.LevelFeatures;
import mapf.planning.analysis.LevelAnalyzer.StrategyType;
import mapf.planning.cbs.CBSStrategy;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.planning.strategy.JointAStarStrategy;
import mapf.planning.strategy.PriorityPlanningStrategy;
import mapf.planning.strategy.PriorityPlanningStrategy.OrderingMode;
import mapf.planning.strategy.SingleAgentStrategy;

import java.util.*;

/**
 * Portfolio-based search controller that selects and executes strategies
 * based on level analysis. Implements fallback mechanism for robustness.
 * 
 * Per ARCHITECTURE.md: "Start with Option A (independent planning), 
 * upgrade to Option B or hybrid approach if competition levels require it."
 */
public class PortfolioController implements SearchStrategy {
    
    private final SearchConfig config;
    private long timeoutMs;
    private LevelFeatures features;
    
    // Cached heuristic - expensive to compute, reuse across strategies
    private Heuristic cachedHeuristic;
    
    // Cached SubgoalManager - shares ImmovableBoxDetector distance cache across PP retries
    private mapf.planning.strategy.SubgoalManager cachedSubgoalManager;
    
    // Track attempts for debugging
    private final List<AttemptRecord> attempts = new ArrayList<>();
    
    public PortfolioController(SearchConfig config) {
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
    }
    
    @Override
    public String getName() {
        return "Portfolio Controller";
    }
    
    @Override
    public void setTimeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    public void setMaxStates(int maxStates) {
        // Delegated to individual strategies
    }
    
    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        long remainingTime = timeoutMs;
        attempts.clear();
        
        // Step 1: Pre-analyze level
        features = LevelAnalyzer.analyze(level, initialState);
        if (SearchConfig.isNormal()) {
            System.err.println(features.analysisReport);
        }
        
        // Step 1b: Independence detection — solve independent groups separately
        // This is a shortcut: if all groups solve, return immediately.
        // If any group fails (partial), fall through to normal portfolio as fallback.
        List<Action[]> bestPartialPlan = null;
        
        if (initialState.getNumAgents() > 1) {
            List<List<Integer>> independentGroups = detectIndependentGroups(initialState, level);
            if (independentGroups.size() > 1) {
                if (SearchConfig.isMinimal()) {
                    System.err.println("[Portfolio] Detected " + independentGroups.size() + 
                        " independent agent groups: " + independentGroups);
                }
                List<Action[]> mergedPlan = solveIndependentGroups(independentGroups, initialState, level, startTime);
                if (mergedPlan != null && !mergedPlan.isEmpty()) {
                    State finalState = replayPlan(mergedPlan, initialState, level);
                    if (finalState != null && finalState.isGoalState(level)) {
                        if (SearchConfig.isMinimal()) {
                            System.err.println("[Portfolio] Independence detection: SOLVED (" + mergedPlan.size() + " steps)");
                        }
                        return mergedPlan;
                    }
                    // Partial — save as fallback, continue with normal portfolio
                    bestPartialPlan = mergedPlan;
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] Independence detection: partial (" + mergedPlan.size() + 
                            " steps) — falling back to normal portfolio");
                    }
                }
            }
        }
        
        // Step 2: Build strategy sequence based on analysis
        List<StrategyConfig> strategies = buildStrategySequence(features, initialState);
        
        if (SearchConfig.isMinimal()) {
            System.err.println("[Portfolio] Strategy sequence: " + 
                strategies.stream().map(s -> s.type.name() + 
                    (s.orderingMode != null ? "(" + s.orderingMode + 
                        (s.orderingMode == OrderingMode.RANDOM ? "#" + s.randomSeed : "") + ")" : "")).toList());
        }
        
        // Step 3: Try strategies in sequence, keeping the best result
        
        for (StrategyConfig strategyConfig : strategies) {
            if (remainingTime <= 0) {
                System.err.println("[Portfolio] Timeout - no more time for attempts");
                break;
            }
            
            // Allocate time for this attempt
            long attemptTimeout = computeAttemptTimeout(strategyConfig, remainingTime, strategies.size());
            
            if (SearchConfig.isMinimal()) {
                String modeStr = "";
                if (strategyConfig.orderingMode != null) {
                    modeStr = "(" + strategyConfig.orderingMode;
                    if (strategyConfig.orderingMode == OrderingMode.RANDOM) modeStr += "#" + strategyConfig.randomSeed;
                    modeStr += ")";
                }
                System.err.println("[Portfolio] Trying " + strategyConfig.type + modeStr +
                    " (timeout=" + attemptTimeout + "ms, budget=" + 
                    String.format("%.0f%%", strategyConfig.timeBudgetFraction * 100) + ")");
            }
            
            // Create and configure strategy
            SearchStrategy strategy = createStrategy(strategyConfig, level);
            strategy.setTimeout(attemptTimeout);
            
            // Execute
            long attemptStart = System.currentTimeMillis();
            List<Action[]> result = null;
            try {
                result = strategy.search(initialState, level);
            } catch (Exception e) {
                System.err.println("[Portfolio] Strategy " + strategyConfig.type + " threw exception: " + e.getMessage());
                e.printStackTrace(System.err);
            }
            long attemptDuration = System.currentTimeMillis() - attemptStart;
            
            // Record attempt
            attempts.add(new AttemptRecord(strategyConfig.type, attemptDuration, 
                                          result != null && !result.isEmpty()));
            
            if (result != null && !result.isEmpty()) {
                // Verify this is a full solution (goal state reached)
                // by checking the plan length vs. a trivially detected partial plan.
                // Full solutions come from PP's isGoalState() check path.
                // Partial plans come from PP's fallback path and are logged as [PARTIAL].
                // Keep best partial as fallback, but only return immediately for full solutions.
                State finalState = replayPlan(result, initialState, level);
                if (finalState != null && finalState.isGoalState(level)) {
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] SUCCESS with " + strategyConfig.type + 
                            " (" + result.size() + " actions, " + attemptDuration + "ms)");
                    }
                    return result;
                }
                // Partial plan — save if best so far, continue trying
                if (bestPartialPlan == null || result.size() > bestPartialPlan.size()) {
                    bestPartialPlan = result;
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] Partial plan from " + strategyConfig.type + 
                            " (" + result.size() + " steps, saved as best so far)");
                    }
                }
            }
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] " + strategyConfig.type + " " +
                    (result != null && !result.isEmpty() ? "partial" : "failed") + " after " + attemptDuration + "ms");
            }
            
            // Update remaining time
            remainingTime = timeoutMs - (System.currentTimeMillis() - startTime);
        }
        
        if (bestPartialPlan != null) {
            System.err.println("[Portfolio] No full solution found. Returning best partial plan (" 
                + bestPartialPlan.size() + " steps)");
            printAttemptSummary();
            return bestPartialPlan;
        }
        
        System.err.println("[Portfolio] All strategies failed");
        printAttemptSummary();
        return null;
    }
    
    /**
     * Replay a plan from initial state to get the final state.
     * Used to verify if a returned plan actually solves the level.
     */
    private State replayPlan(List<Action[]> plan, State initialState, Level level) {
        State state = initialState;
        for (Action[] jointAction : plan) {
            try {
                state = state.applyJointAction(jointAction, level);
            } catch (Exception e) {
                return state; // Return whatever state we reached
            }
        }
        return state;
    }
    
    /**
     * Builds strategy sequence based on level features.
     * 
     * Pull-Sokoban portfolio design principles:
     * - CBS/JointAStar are NOT used as top-level strategies (CBS models MAPF point-to-point,
     *   not Sokoban push/pull; JointAStar is O(5^n) unusable above 3 agents).
     *   CBS is retained as PP-internal cycle fallback (tryCBSFallback).
     * - REVERSE_TOPOLOGICAL is removed (never solved any competition level in testing).
     * - Multi-seed RANDOM is the primary cycle-breaking mechanism: different seeds
     *   produce different shuffle orderings, multiplying the chance of finding a
     *   subgoal order that avoids TRAP/REGRESS deadlocks.
     */
    private List<StrategyConfig> buildStrategySequence(LevelFeatures f, State state) {
        List<StrategyConfig> strategies = new ArrayList<>();
        
        if (f.recommendedStrategy == StrategyType.SINGLE_AGENT) {
            // Single agent: A* with increasing weights
            strategies.add(new StrategyConfig(StrategyType.SINGLE_AGENT, 1.0, null, 0, 0.40));
            strategies.add(new StrategyConfig(StrategyType.SINGLE_AGENT, 5.0, null, 0, 0.60));
        } else if (f.hasCircularDependency) {
            // Cyclic dependencies: multi-seed RANDOM is primary strategy.
            // Each seed produces a different subgoal shuffle, breaking different cycles.
            // TOPOLOGICAL retained as backup — still produces good partial plans even with cycles.
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 42, 0.25));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 137, 0.25));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.25));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.25));
        } else {
            // No cycles: topological order is well-founded, use it first.
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.40));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.30));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 42, 0.30));
        }
        
        return strategies;
    }
    
    /**
     * Computes timeout for a single attempt based on pre-assigned time budget fraction.
     * Fractions are relative to total timeout, but capped by remaining time.
     */
    private long computeAttemptTimeout(StrategyConfig config, long remainingTime, int totalStrategies) {
        long budgetMs = (long) (timeoutMs * config.timeBudgetFraction);
        return Math.min(budgetMs, remainingTime);
    }
    
    /**
     * Creates actual strategy instance based on config.
     */
    private SearchStrategy createStrategy(StrategyConfig config, Level level) {
        Heuristic heuristic = createHeuristic(level);
        SearchConfig strategyConfig = new SearchConfig(
            config.weight == Double.POSITIVE_INFINITY ? this.config.getTimeoutMs() : this.config.getTimeoutMs(),
            this.config.getMaxStates(),
            config.weight
        );
        
        switch (config.type) {
            case SINGLE_AGENT:
                SingleAgentStrategy singleAgent = new SingleAgentStrategy(heuristic, strategyConfig);
                singleAgent.setWeight(config.weight);
                return singleAgent;
                
            case CBS:
                CBSStrategy cbs = new CBSStrategy(heuristic, strategyConfig);
                return cbs;
                
            case JOINT_SEARCH:
                JointAStarStrategy jointAStar = new JointAStarStrategy(heuristic, strategyConfig);
                jointAStar.setWeight(config.weight);
                return jointAStar;
                
            case STRICT_ORDER:
            case CYCLE_BREAKER:
            case GREEDY_WITH_RETRY:
            default:
                // All use PriorityPlanningStrategy with different ordering modes
                // Share SubgoalManager across retries to reuse BFS distance cache
                if (cachedSubgoalManager == null) {
                    cachedSubgoalManager = new mapf.planning.strategy.SubgoalManager(heuristic);
                }
                PriorityPlanningStrategy priorityPlanning = new PriorityPlanningStrategy(heuristic, strategyConfig, cachedSubgoalManager);
                // Pass execution order from analysis
                if (features != null && features.executionOrder != null) {
                    priorityPlanning.setGoalExecutionOrder(features.executionOrder);
                }
                // Pass goal dependencies (for hard/soft frozen distinction)
                if (features != null && features.goalDependsOn != null) {
                    priorityPlanning.setGoalDependencies(features.goalDependsOn);
                }
                // Pass immovable boxes (treated as walls)
                if (features != null && features.taskFilter != null) {
                    priorityPlanning.setImmovableBoxes(features.taskFilter.immovableBoxes);
                }
                // Set ordering mode for this retry attempt
                if (config.orderingMode != null) {
                    priorityPlanning.setOrderingMode(config.orderingMode);
                }
                // Set random seed for RANDOM ordering (multi-seed diversification)
                if (config.orderingMode == OrderingMode.RANDOM && config.randomSeed != 0) {
                    priorityPlanning.setRandomSeed(config.randomSeed);
                }
                return priorityPlanning;
        }
    }
    
    private Heuristic createHeuristic(Level level) {
        // Cache heuristic - BFS precomputation is expensive
        if (cachedHeuristic == null) {
            try {
                cachedHeuristic = new TrueDistanceHeuristic(level);
            } catch (Exception e) {
                cachedHeuristic = new ManhattanHeuristic();
            }
        }
        return cachedHeuristic;
    }
    
    private void printAttemptSummary() {
        if (!SearchConfig.isMinimal()) return;
        
        System.err.println("\n=== Portfolio Attempt Summary ===");
        for (AttemptRecord record : attempts) {
            System.err.println(String.format("  %s: %dms, %s",
                record.strategy, record.durationMs, record.success ? "SUCCESS" : "FAILED"));
        }
    }
    
    /**
     * Returns the analysis result (for debugging/testing).
     */
    public LevelFeatures getFeatures() {
        return features;
    }
    
    // ========== Independence Detection ==========
    
    /**
     * Detects independent agent groups using BFS reachability.
     * Two agents are in the same group if their reachable areas overlap
     * (i.e., they can physically reach each other's positions).
     * Immovable boxes (pre-satisfied boxes that shouldn't be moved) are treated as walls
     * because pushing them would unsatisfy goals. This matches PPS's componentization.
     */
    private List<List<Integer>> detectIndependentGroups(State state, Level level) {
        int numAgents = state.getNumAgents();
        Set<Position> immovable = features != null && features.taskFilter != null 
            ? features.taskFilter.immovableBoxes : Collections.emptySet();
        
        // BFS reachability per agent
        Map<Integer, Set<Position>> reachable = new HashMap<>();
        for (int a = 0; a < numAgents; a++) {
            Set<Position> area = new HashSet<>();
            Queue<Position> queue = new LinkedList<>();
            Position start = state.getAgentPosition(a);
            queue.add(start);
            area.add(start);
            while (!queue.isEmpty()) {
                Position current = queue.poll();
                for (Direction dir : Direction.values()) {
                    Position next = current.move(dir);
                    if (!level.isWall(next) && !immovable.contains(next) && !area.contains(next)) {
                        area.add(next);
                        queue.add(next);
                    }
                }
            }
            reachable.put(a, area);
        }
        
        // Union-Find: agents whose reachable sets overlap
        int[] parent = new int[numAgents];
        for (int i = 0; i < numAgents; i++) parent[i] = i;
        for (int a = 0; a < numAgents; a++) {
            for (int b = a + 1; b < numAgents; b++) {
                if (reachable.get(a).contains(state.getAgentPosition(b))) {
                    ufUnion(parent, a, b);
                }
            }
        }
        
        // Group by root
        Map<Integer, List<Integer>> groups = new LinkedHashMap<>();
        for (int a = 0; a < numAgents; a++) {
            groups.computeIfAbsent(ufFind(parent, a), k -> new ArrayList<>()).add(a);
        }
        return new ArrayList<>(groups.values());
    }
    
    private int ufFind(int[] parent, int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    
    private void ufUnion(int[] parent, int a, int b) {
        parent[ufFind(parent, a)] = ufFind(parent, b);
    }
    
    /**
     * Solves independent groups separately and merges results.
     * Each group gets a projected State+Level with remapped agent IDs (0..k-1),
     * an independent strategy sequence, and its own timeout budget.
     */
    private List<Action[]> solveIndependentGroups(List<List<Integer>> groups,
            State initialState, Level level, long startTime) {
        int numAgents = level.getNumAgents();
        Set<Position> immovable = features != null && features.taskFilter != null 
            ? features.taskFilter.immovableBoxes : Collections.emptySet();
        List<GroupResult> results = new ArrayList<>();
        
        for (List<Integer> group : groups) {
            long remaining = timeoutMs - (System.currentTimeMillis() - startTime);
            if (remaining <= 0) break;
            
            // Compute reachable area for this group
            Set<Position> reachableArea = computeReachableArea(group, initialState, level, immovable);
            
            // Project State and Level to this group
            int[] originalIds = group.stream().sorted().mapToInt(Integer::intValue).toArray();
            Map<Integer, Integer> newIdMap = new HashMap<>();
            for (int i = 0; i < originalIds.length; i++) newIdMap.put(originalIds[i], i);
            
            State projState = projectState(originalIds, initialState, reachableArea);
            Level projLevel = projectLevel(originalIds, newIdMap, level, reachableArea);
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] Solving group " + group + " (" + group.size() + 
                    " agents, timeout=" + remaining + "ms)");
            }
            
            // Solve this group independently
            List<Action[]> groupPlan = solveGroup(projState, projLevel, remaining);
            
            // Check if group subproblem was fully solved
            boolean groupSolved = false;
            if (groupPlan != null && !groupPlan.isEmpty()) {
                State finalState = replayPlan(groupPlan, projState, projLevel);
                groupSolved = finalState != null && finalState.isGoalState(projLevel);
            }
            results.add(new GroupResult(originalIds, groupPlan));
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] Group " + group + ": " + 
                    (groupPlan != null ? groupPlan.size() + " steps" + (groupSolved ? " SOLVED" : " PARTIAL") : "FAILED"));
            }
        }
        
        return mergeGroupPlans(results, numAgents);
    }
    
    /**
     * BFS reachability from all agents in the group.
     * Movable boxes are passable, immovable boxes and walls are not.
     */
    private Set<Position> computeReachableArea(List<Integer> agentIds, State state, 
            Level level, Set<Position> immovable) {
        Set<Position> area = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        for (int agentId : agentIds) {
            Position start = state.getAgentPosition(agentId);
            if (area.add(start)) queue.add(start);
        }
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (level.isWall(next)) continue;
                if (immovable.contains(next)) {
                    area.add(next); // Include boundary immovable box but don't expand through
                    continue;
                }
                if (area.add(next)) {
                    queue.add(next);
                }
            }
        }
        return area;
    }
    
    /**
     * Projects a State to contain only the group's agents (remapped 0..k-1)
     * and boxes within the reachable area.
     */
    private State projectState(int[] originalIds, State state, Set<Position> reachableArea) {
        Position[] newAgentPositions = new Position[originalIds.length];
        for (int i = 0; i < originalIds.length; i++) {
            newAgentPositions[i] = state.getAgentPosition(originalIds[i]);
        }
        Map<Position, Character> newBoxes = new HashMap<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (reachableArea.contains(entry.getKey())) {
                newBoxes.put(entry.getKey(), entry.getValue());
            }
        }
        return new State(newAgentPositions, newBoxes);
    }
    
    /**
     * Projects a Level for an independent group:
     * - Walls: unchanged (immovable boxes are included as boxes in the projected state;
     *   the group's TaskFilter will naturally detect them as immovable)
     * - Box goals: only those in the reachable area
     * - Agent goals: remapped to new IDs, only active agents
     * - Agent colors: remapped to new IDs
     * - Box colors: unchanged (unused types are harmless)
     */
    private Level projectLevel(int[] originalIds, Map<Integer, Integer> newIdMap,
            Level level, Set<Position> reachableArea) {
        int rows = level.getRows();
        int cols = level.getCols();
        
        boolean[][] walls = new boolean[rows][cols];
        char[][] boxGoals = new char[rows][cols];
        int[][] agentGoals = new int[rows][cols];
        
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                walls[r][c] = level.isWall(r, c);
                agentGoals[r][c] = -1; // default: no agent goal
                
                // Keep box goals only in reachable area
                char bg = level.getBoxGoal(r, c);
                if (bg != '\0' && reachableArea.contains(Position.of(r, c))) {
                    boxGoals[r][c] = bg;
                }
                
                // Remap agent goals to new IDs
                int ag = level.getAgentGoal(r, c);
                if (ag >= 0 && newIdMap.containsKey(ag)) {
                    agentGoals[r][c] = newIdMap.get(ag);
                }
            }
        }
        
        Map<Integer, Color> agentColors = new HashMap<>();
        for (int i = 0; i < originalIds.length; i++) {
            agentColors.put(i, level.getAgentColor(originalIds[i]));
        }
        
        return new Level(level.getName() + "_g" + originalIds[0], rows, cols,
                         walls, boxGoals, agentGoals, level.getBoxColors(), agentColors);
    }
    
    /**
     * Runs the full strategy sequence for a projected group subproblem.
     */
    private List<Action[]> solveGroup(State projState, Level projLevel, long timeout) {
        LevelFeatures groupFeatures = LevelAnalyzer.analyze(projLevel, projState);
        List<StrategyConfig> strategies = buildStrategySequence(groupFeatures, projState);
        
        Heuristic heuristic;
        try {
            heuristic = new TrueDistanceHeuristic(projLevel);
        } catch (Exception e) {
            heuristic = new ManhattanHeuristic();
        }
        
        if (SearchConfig.isMinimal()) {
            System.err.println("  Strategy sequence: " + 
                strategies.stream().map(s -> s.type.name() + 
                    (s.orderingMode != null ? "(" + s.orderingMode + 
                        (s.orderingMode == OrderingMode.RANDOM ? "#" + s.randomSeed : "") + ")" : "")).toList());
        }
        
        long groupStart = System.currentTimeMillis();
        long remaining = timeout;
        List<Action[]> bestPartial = null;
        
        mapf.planning.strategy.SubgoalManager groupSubgoalManager = 
            new mapf.planning.strategy.SubgoalManager(heuristic);
        
        for (StrategyConfig sc : strategies) {
            if (remaining <= 0) break;
            long attemptTimeout = Math.min((long)(timeout * sc.timeBudgetFraction), remaining);
            
            SearchConfig strategyConfig = new SearchConfig(
                timeout, this.config.getMaxStates(), sc.weight);
            
            SearchStrategy strategy;
            if (sc.type == StrategyType.SINGLE_AGENT) {
                SingleAgentStrategy sa = new SingleAgentStrategy(heuristic, strategyConfig);
                sa.setWeight(sc.weight);
                strategy = sa;
            } else {
                PriorityPlanningStrategy pp = new PriorityPlanningStrategy(
                    heuristic, strategyConfig, groupSubgoalManager);
                if (groupFeatures.executionOrder != null) pp.setGoalExecutionOrder(groupFeatures.executionOrder);
                if (groupFeatures.goalDependsOn != null) pp.setGoalDependencies(groupFeatures.goalDependsOn);
                if (groupFeatures.taskFilter != null) pp.setImmovableBoxes(groupFeatures.taskFilter.immovableBoxes);
                if (sc.orderingMode != null) pp.setOrderingMode(sc.orderingMode);
                if (sc.orderingMode == OrderingMode.RANDOM && sc.randomSeed != 0) pp.setRandomSeed(sc.randomSeed);
                strategy = pp;
            }
            strategy.setTimeout(attemptTimeout);
            
            List<Action[]> result = null;
            try {
                result = strategy.search(projState, projLevel);
            } catch (Exception e) {
                System.err.println("[Portfolio] Group strategy " + sc.type + " threw: " + e.getMessage());
            }
            
            if (result != null && !result.isEmpty()) {
                State finalState = replayPlan(result, projState, projLevel);
                if (finalState != null && finalState.isGoalState(projLevel)) {
                    return result;
                }
                if (bestPartial == null || result.size() > bestPartial.size()) {
                    bestPartial = result;
                }
            }
            
            remaining = timeout - (System.currentTimeMillis() - groupStart);
        }
        
        return bestPartial;
    }
    
    /**
     * Merges per-group plans into a single joint action plan.
     * Each group's actions are remapped from projected agent IDs (0..k-1) back to
     * original agent IDs. Non-active agents get NoOp at each timestep.
     */
    private List<Action[]> mergeGroupPlans(List<GroupResult> results, int numAgents) {
        int maxLen = 0;
        for (GroupResult gr : results) {
            if (gr.plan != null) maxLen = Math.max(maxLen, gr.plan.size());
        }
        if (maxLen == 0) return null;
        
        List<Action[]> merged = new ArrayList<>(maxLen);
        for (int t = 0; t < maxLen; t++) {
            Action[] joint = new Action[numAgents];
            Arrays.fill(joint, Action.NOOP);
            for (GroupResult gr : results) {
                if (gr.plan != null && t < gr.plan.size()) {
                    Action[] groupAction = gr.plan.get(t);
                    for (int newId = 0; newId < gr.originalIds.length; newId++) {
                        int origId = gr.originalIds[newId];
                        if (newId < groupAction.length && groupAction[newId] != null) {
                            joint[origId] = groupAction[newId];
                        }
                    }
                }
            }
            merged.add(joint);
        }
        return merged;
    }
    
    private static class GroupResult {
        final int[] originalIds;
        final List<Action[]> plan;
        GroupResult(int[] originalIds, List<Action[]> plan) {
            this.originalIds = originalIds;
            this.plan = plan;
        }
    }
    
    // ========== Helper Classes ==========
    
    private static class StrategyConfig {
        final StrategyType type;
        final double weight;
        final OrderingMode orderingMode;
        final int randomSeed;
        final double timeBudgetFraction;
        
        StrategyConfig(StrategyType type, double weight, OrderingMode orderingMode, int randomSeed, double timeBudgetFraction) {
            this.type = type;
            this.weight = weight;
            this.orderingMode = orderingMode;
            this.randomSeed = randomSeed;
            this.timeBudgetFraction = timeBudgetFraction;
        }
    }
    
    private static class AttemptRecord {
        final StrategyType strategy;
        final long durationMs;
        final boolean success;
        
        AttemptRecord(StrategyType strategy, long durationMs, boolean success) {
            this.strategy = strategy;
            this.durationMs = durationMs;
            this.success = success;
        }
    }
}
