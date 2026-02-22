package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
import mapf.planning.analysis.CrossColorBarrierAnalyzer;
import mapf.planning.analysis.DependencyAnalyzer;
import mapf.planning.analysis.LevelAnalyzer;
import mapf.planning.cbs.CBSStrategy;
import mapf.planning.coordination.DeadlockResolver;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.spacetime.ReservationTable;
import java.util.*;

/**
 * Priority-based planning strategy with subgoal decomposition.
 * Per ARCHITECTURE.md: "Independent Planning + Conflict Resolution"
 */
public class PriorityPlanningStrategy implements SearchStrategy {

    private final Heuristic heuristic;
    private final SearchConfig config;
    private long timeoutMs;
    private int maxStates;
    private final Random random = new Random(SearchConfig.RANDOM_SEED);

    // Core helper classes (SRP: each handles one responsibility)
    private final SubgoalManager subgoalManager;
    private final ConflictResolver conflictResolver;
    private final BoxSearchPlanner boxSearchPlanner;
    private final GreedyPlanner greedyPlanner;
    private final PlanMerger planMerger;
    private final PathAnalyzer pathAnalyzer;
    private final AgentCoordinator agentCoordinator;
    private final DeadlockBreaker deadlockBreaker;
    private final DeadlockResolver deadlockResolver = new DeadlockResolver();

    /** Tracks displaced boxes to avoid infinite loops. */
    private Set<String> displacementHistory = new HashSet<>();
    
    /** Counts displacement attempts; forces CBS after MAX_DISPLACEMENT_ATTEMPTS. */
    private int displacementAttempts = 0;
    private static final int MAX_DISPLACEMENT_ATTEMPTS = 3;
    
    /** 
     * Goal-level cycle detection: tracks how many times each goal position has been
     * completed. If a goal is completed > MAX_GOAL_COMPLETIONS times, PP is cycling
     * (push to goal → pull off → push back → ...). This supplements displacementHistory
     * which only tracks box-level displacement events and misses goal-level cycles.
     */
    private Map<Position, Integer> goalCompletionCount = new HashMap<>();
    private static final int MAX_GOAL_COMPLETIONS = 3;
    
    /** Tracks agent goals that have been completed at least once, to detect phantom progress. */
    private Set<Position> completedAgentGoals = new HashSet<>();;
    
    /**
     * Tracks completed goals that were intentionally displaced by tryRecovery.
     * These positions remain in completedBoxGoals (to satisfy dependency checks)
     * but are excluded from the frozen set (so BSP can path through them).
     * Cleared when another subgoal makes genuine progress.
     */
    private Set<Position> displacedGoals = new HashSet<>();
    
    /**
     * Borrow-and-return: goals displaced by the most recent tryPathClearing call.
     * After clearing+BSP succeeds, these are re-planned back to their goals.
     * Max MAX_CLEARING_DISPLACEMENTS allowed per clearing operation.
     */
    private List<Position> lastClearingDisplacedGoals = new ArrayList<>();
    private static final int MAX_CLEARING_DISPLACEMENTS = 4;
    
    /** Pre-computed goal execution order from LevelAnalyzer (optional). */
    private List<Position> precomputedGoalOrder = null;
    
    /** Goal dependency graph from LevelAnalyzer: goal → set of goals it depends on. */
    private Map<Position, Set<Position>> goalDependsOn = Collections.emptyMap();
    
    /** Immovable boxes (treated as walls in pathfinding). */
    private Set<Position> immovableBoxes = Collections.emptySet();

    /**
     * Cache of barrier clearing orders that were found non-extractable.
     * Prevents the infinite loop where dynamic barrier re-detection keeps
     * finding the same non-extractable barrier every stuck iteration.
     * Reset when genuine progress is made (completedBoxGoals changes).
     */
    private Set<List<Position>> skippedBarrierClearingOrders = new HashSet<>();

    /**
     * Counter for dynamic barrier re-detection rounds (in the stuck recovery loop).
     * Limited to prevent wasting hundreds of steps on incomplete barrier clearing
     * that doesn't produce additional goal solves.
     */
    private int dynamicBarrierRounds = 0;
    private static final int MAX_DYNAMIC_BARRIER_ROUNDS = 5;
    
    /**
     * Ordering mode for subgoal execution. Different modes explore different
     * orderings, which is the most effective "retry knob" for PP.
     */
    public enum OrderingMode {
        /** Use LevelAnalyzer's topological sort (dependency-first). Default. */
        TOPOLOGICAL,
        /** Reverse of topological sort. Breaks deadlocks when normal order fails. */
        REVERSE_TOPOLOGICAL,
        /** Sort by box-to-goal distance, nearest first. Good for low-dependency levels. */
        DISTANCE_GREEDY,
        /** Random shuffle. Last-resort diversification. */
        RANDOM
    }
    
    /** Current ordering mode for subgoal sorting. */
    private OrderingMode orderingMode = OrderingMode.TOPOLOGICAL;
    
    /** Completed box goals - treated as permanent obstacles per MAPF standard. */
    private Set<Position> completedBoxGoals = new HashSet<>();
    
    /** Space-time reservation table for collision avoidance. */
    private ReservationTable reservationTable = new ReservationTable();
    
    /** Current global time step for space-time planning. */
    private int globalTimeStep = 0;
    
    /** Flag: was the last progress from re-solving a previously-completed agent goal? */
    private boolean lastProgressWasPhantom = false;
    
    /** Maps agent ID to connected component ID. Agents in different components can execute in parallel. */
    private Map<Integer, Integer> agentComponentId = Collections.emptyMap();
    
    /** Tracks subgoals being executed via stored plans for independent agents. */
    private Map<Integer, Subgoal> storedPlanSubgoals = new HashMap<>();

    // Logging helpers
    private void logMinimal(String msg) {
        if (SearchConfig.isMinimal()) System.err.println(msg);
    }
    private void logNormal(String msg) {
        if (SearchConfig.isNormal()) System.err.println(msg);
    }
    private void logVerbose(String msg) {
        if (SearchConfig.isVerbose()) System.err.println(msg);
    }

    public PriorityPlanningStrategy(Heuristic heuristic, SearchConfig config) {
        this(heuristic, config, new SubgoalManager(heuristic));
    }

    /**
     * Constructor with shared SubgoalManager for cross-strategy cache reuse.
     * Follows Dependency Inversion: Portfolio injects shared instance.
     */
    public PriorityPlanningStrategy(Heuristic heuristic, SearchConfig config, SubgoalManager sharedSubgoalManager) {
        this.heuristic = heuristic;
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
        
        this.subgoalManager = sharedSubgoalManager;
        this.conflictResolver = new ConflictResolver();
        this.boxSearchPlanner = new BoxSearchPlanner(heuristic);
        this.greedyPlanner = new GreedyPlanner();
        this.planMerger = new PlanMerger();
        this.pathAnalyzer = new PathAnalyzer();
        this.agentCoordinator = new AgentCoordinator();
        this.deadlockBreaker = new DeadlockBreaker();
    }

    @Override
    public String getName() { return "Priority Planning"; }

    @Override
    public void setTimeout(long timeoutMs) { this.timeoutMs = timeoutMs; }

    @Override
    public void setMaxStates(int maxStates) { this.maxStates = maxStates; }
    
    /** Sets pre-computed goal execution order from LevelAnalyzer. */
    public void setGoalExecutionOrder(List<Position> order) {
        this.precomputedGoalOrder = order;
        if (order != null && SearchConfig.isNormal()) {
            System.err.println("[PP] Using pre-computed goal order: " + order.size() + " goals");
        }
    }
    
    /** Sets goal dependency graph from LevelAnalyzer. Used to distinguish hard/soft frozen goals. */
    public void setGoalDependencies(Map<Position, Set<Position>> deps) {
        this.goalDependsOn = deps != null ? deps : Collections.emptyMap();
    }
    
    /** Sets immovable boxes (treated as walls in pathfinding). */
    public void setImmovableBoxes(Set<Position> immovable) {
        this.immovableBoxes = immovable != null ? immovable : Collections.emptySet();
    }
    
    /** Sets the ordering mode for subgoal execution. */
    public void setOrderingMode(OrderingMode mode) {
        this.orderingMode = mode != null ? mode : OrderingMode.TOPOLOGICAL;
    }

    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        int numAgents = initialState.getNumAgents();

        // Ensure robust dependency analysis is used (SRP fix form step 2)
        // If external controller provided no order, compute it ourselves using the standard analyzer
        if (precomputedGoalOrder == null) {
             LevelAnalyzer.LevelFeatures features = LevelAnalyzer.analyze(level, initialState);
             setGoalExecutionOrder(features.executionOrder);
             setGoalDependencies(features.goalDependsOn);
             if (features.taskFilter != null && (immovableBoxes == null || immovableBoxes.isEmpty())) {
                 setImmovableBoxes(features.taskFilter.immovableBoxes);
             }
        }

        // Precompute BFS distance maps for all goals (O(G×N) once, then O(1) per query)
        subgoalManager.initDistanceCache(initialState, level);

        // Precompute articulation points for parking avoidance (O(V+E) once)
        Set<Position> ap = ArticulationPointFinder.findArticulationPoints(level, immovableBoxes);
        pathAnalyzer.setArticulationPoints(ap);
        if (SearchConfig.isNormal()) {
            System.err.println("[PP] Articulation points: " + ap.size());
        }

        // Compute independent agent groups for parallel execution
        agentComponentId = computeAgentComponents(initialState, level);

        // Reset for new search
        displacementHistory.clear();
        displacementAttempts = 0;
        completedBoxGoals.clear();
        completedAgentGoals.clear();
        goalCompletionCount.clear();
        displacedGoals.clear();
        reservationTable.clear();
        planMerger.clearAllPlans();
        storedPlanSubgoals.clear();
        globalTimeStep = 0;
        lastComputedState = null;
        lastComputedPlanSize = 0;
        dynamicBarrierRounds = 0;

        return planWithSubgoals(initialState, level, startTime);
    }

    /** Cached subgoal order - MAPF PP requires fixed priority (computed once, reused). */
    private List<Subgoal> cachedSubgoalOrder = null;

    /**
     * Core PP algorithm: iteratively solve subgoals in priority order.
     * Per ARCHITECTURE.md: "moves one box at a time, naturally handles dependencies"
     * MAPF FIX: Priority is fixed at start, not recomputed each iteration.
     */
    private List<Action[]> planWithSubgoals(State initialState, Level level, long startTime) {
        List<Action[]> fullPlan = new ArrayList<>();
        State currentState = initialState;
        int numAgents = initialState.getNumAgents();
        int stuckCount = 0;
        
        // MAPF FIX: Compute subgoal order ONCE at start, then reuse
        cachedSubgoalOrder = null;

        // Cross-color barrier clearing: detect if any agent is blocked from its goals
        // by boxes of a different color, and pre-clear a path before the main loop.
        currentState = detectAndExecuteClearingPhase(
                currentState, initialState, level, fullPlan, numAgents, startTime);

        while (!currentState.isGoalState(level)) {
            // PRODUCT.md constraints: 3 minutes, 20,000 actions
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                logVerbose(getName() + ": Timeout");
                break;
            }
            if (fullPlan.size() >= SearchConfig.MAX_ACTIONS) {
                logVerbose(getName() + ": Action limit");
                break;
            }
            if (stuckCount > SearchConfig.MAX_STUCK_ITERATIONS) {
                logVerbose(getName() + ": Stuck");
                break;
            }

            // Try CBS fallback on cyclic dependency detection
            if (stuckCount == SearchConfig.DEPENDENCY_CHECK_THRESHOLD && SearchConfig.USE_CBS_ON_CYCLE) {
                List<Action[]> cbsResult = tryCBSFallback(currentState, level, initialState, fullPlan, startTime, numAgents);
                if (cbsResult != null) return cbsResult;
            }

            // MAPF FIX: Use cached order, only filter out completed goals
            List<Subgoal> unsatisfied = getOrComputeSubgoalOrder(currentState, level);
            if (unsatisfied.isEmpty()) break;

            // Try to execute highest priority subgoal
            boolean madeProgress = tryExecuteSubgoals(unsatisfied, fullPlan, currentState, level, numAgents, initialState);
            
            if (madeProgress) {
                // Update state from plan
                currentState = recomputeState(initialState, fullPlan, level, numAgents);
                // Only reset stuckCount for genuine progress (not re-solving same agent goal)
                if (!lastProgressWasPhantom) {
                    stuckCount = 0;
                    // MAPF FIX: Clear displacement history on genuine progress
                    displacementHistory.clear();
                    displacementAttempts = 0;
                    // Clear barrier cache — state changed, barriers may now be extractable
                    skippedBarrierClearingOrders.clear();
                    dynamicBarrierRounds = 0;
                } else {
                    stuckCount++;
                }
            } else {
                stuckCount++;
                
                // Try recovery mechanisms when stuck
                if (stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING) {
                    boolean recovered = tryRecovery(unsatisfied, fullPlan, currentState, level, numAgents, initialState);
                    if (recovered) {
                        currentState = recomputeState(initialState, fullPlan, level, numAgents);
                        stuckCount = 0;
                    } else {
                        // Dynamic barrier re-detection: state changes may have created
                        // new cross-color barriers. Run barrier analysis again.
                        // Limited to prevent wasting steps on incomplete clearing.
                        if (dynamicBarrierRounds < MAX_DYNAMIC_BARRIER_ROUNDS) {
                            State afterBarrier = detectAndExecuteClearingPhase(
                                    currentState, initialState, level, fullPlan, numAgents, startTime);
                            dynamicBarrierRounds++;
                            if (afterBarrier != currentState) {
                                currentState = afterBarrier;
                                stuckCount = 0;
                                logVerbose("[PP] Dynamic barrier re-detection cleared new barriers (round "
                                        + dynamicBarrierRounds + "/" + MAX_DYNAMIC_BARRIER_ROUNDS + ")");
                            }
                        }
                    }
                }
            }
        }

        if (currentState.isGoalState(level)) {
            logMinimal(getName() + ": [OK] Goal state reached!");
            // MAPF FIX: Validate and optimize the plan before returning
            fullPlan = validateAndOptimizePlan(fullPlan, initialState, level, numAgents);
            return fullPlan.isEmpty() ? null : fullPlan;
        }
        // Goal not fully reached — return partial plan for use as fallback
        if (!fullPlan.isEmpty()) {
            logMinimal(getName() + ": [PARTIAL] Partial plan (" + fullPlan.size() + " steps) — goal not reached");
            fullPlan = validateAndOptimizePlan(fullPlan, initialState, level, numAgents);
            return fullPlan.isEmpty() ? null : fullPlan;
        }
        logMinimal(getName() + ": [FAIL] Could not reach goal state, no partial plan available");
        return null;
    }

    // ==================== Cross-Color Barrier Clearing ====================

    /**
     * Pre-pass: detects cross-color barriers and clears them before the main PP loop.
     * 
     * In pull-supporting Sokoban, a single agent can move any same-color box through
     * arbitrarily narrow corridors using pull chains. When an agent is blocked by
     * OTHER-color boxes, we identify the minimum set of boxes to clear, assign
     * a same-color agent to clear each one via planBoxDisplacement, and execute.
     * 
     * After clearing, the displaced box-goals become unsatisfied. The normal PP loop
     * will generate return subgoals automatically (cyan agents push A-boxes back).
     * 
     * @return updated state after clearing (or original state if no barriers)
     */
    private State detectAndExecuteClearingPhase(State currentState, State initialState,
                                                  Level level, List<Action[]> fullPlan,
                                                  int numAgents, long startTime) {
        // Only analyze barriers if there are multiple colors of boxes
        if (level.getNumAgents() < 2) return currentState;

        // Build a minimal subgoal list for barrier analysis
        List<Subgoal> checkSubgoals = subgoalManager.getUnsatisfiedSubgoals(currentState, level, completedBoxGoals);
        if (checkSubgoals.isEmpty()) return currentState;

        List<CrossColorBarrierAnalyzer.Barrier> barriers =
                CrossColorBarrierAnalyzer.analyzeBarriers(currentState, level, checkSubgoals);

        if (barriers.isEmpty()) return currentState;

        logMinimal("[PP] Cross-color barriers detected: " + barriers.size());

        // Sort barriers by clearing order size: solve small barriers first. 
        // This opens up parking space (e.g., clearing Z opens left-right connection)
        // before tackling large barriers that need many parking spots.
        barriers.sort((a, b) -> Integer.compare(a.clearingOrder.size(), b.clearingOrder.size()));

        for (CrossColorBarrierAnalyzer.Barrier barrier : barriers) {
            if (System.currentTimeMillis() - startTime > timeoutMs / 2) {
                logNormal("[PP] Clearing phase timeout — skipping remaining barriers");
                break;
            }

            // Cache check: skip barriers that were already found non-extractable
            if (skippedBarrierClearingOrders.contains(barrier.clearingOrder)) {
                logVerbose("[PP] Barrier already cached as non-extractable — skipping");
                continue;
            }

            logMinimal("[PP] Clearing barrier for agent " + barrier.blockedAgentId
                    + ": " + barrier.clearingOrder.size() + " " + barrier.blockingBoxType
                    + "-boxes to clear");

            // Find clearing agent (agent of the blocking color)
            int clearingAgent = CrossColorBarrierAnalyzer.findClearingAgent(
                    barrier.blockingColor, barrier.clearingOrder.get(0), currentState, level);
            if (clearingAgent < 0) {
                logNormal("[PP] No agent of color " + barrier.blockingColor + " found — skipping");
                continue;
            }

            // Find parking positions for the cleared boxes
            Set<Position> agentReachable = bfsReachableForClearing(
                    currentState.getAgentPosition(clearingAgent), currentState, level);

            // PRE-CHECK: Verify the first barrier box is physically extractable.
            Position firstBox = barrier.clearingOrder.get(0);
            if (!CrossColorBarrierAnalyzer.isBoxExtractable(firstBox, agentReachable, currentState, level)) {
                logNormal("[PP] First barrier box " + firstBox + " is not extractable "
                        + "(dead-end corridor, no bypass) — skipping barrier");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }

            Set<Position> baseAvoidPositions = new HashSet<>(barrier.clearingOrder);
            // Avoid cells near barrier entry — the pull-chain approach path.
            // Radius-1 around ALL barrier boxes + radius-2 around the gap entry point.
            for (Position barrierPos : barrier.clearingOrder) {
                for (Direction dir : Direction.values()) {
                    Position adj = barrierPos.move(dir);
                    if (!level.isWall(adj)) {
                        baseAvoidPositions.add(adj);
                    }
                }
            }
            // Extended exclusion around gap entry (first box = closest to agent)
            Position gapEntry = barrier.clearingOrder.get(0);
            for (int dr = -2; dr <= 2; dr++) {
                for (int dc = -2; dc <= 2; dc++) {
                    if (Math.abs(dr) + Math.abs(dc) <= 2) {
                        Position near = Position.of(gapEntry.row + dr, gapEntry.col + dc);
                        if (!level.isWall(near)) {
                            baseAvoidPositions.add(near);
                        }
                    }
                }
            }

            // APPROACH CORRIDOR: compute BFS shortest path from agent to the first
            // barrier box. All cells on this path are critical for reaching the barrier;
            // parking on them would block subsequent box extractions.
            List<Position> approachPath = computeApproachPath(
                    currentState.getAgentPosition(clearingAgent), firstBox, currentState, level);
            baseAvoidPositions.addAll(approachPath);

            // GAP EXIT ROW PROTECTION: the row connecting the maze to the gap is critical.
            // BSP may chain-push parked boxes along this row, blocking the gap approach.
            // Identify the gap exit row from the approach path end (near the barrier).
            if (approachPath.size() >= 2) {
                // Last cell = gap (adjacent to barrier), second-to-last = gap exit
                Position gapCell = approachPath.get(approachPath.size() - 1);
                Position preGapCell = approachPath.get(approachPath.size() - 2);
                if (preGapCell.row != gapCell.row) {
                    // Gap exit is on a different row — protect that entire row segment
                    int gapExitRow = preGapCell.row;
                    for (int c = preGapCell.col; c < level.getCols(); c++) {
                        Position pos = Position.of(gapExitRow, c);
                        if (level.isWall(pos)) break;
                        baseAvoidPositions.add(pos);
                    }
                    for (int c = preGapCell.col - 1; c >= 0; c--) {
                        Position pos = Position.of(gapExitRow, c);
                        if (level.isWall(pos)) break;
                        baseAvoidPositions.add(pos);
                    }
                    logNormal("[PP] [CLEAR-BARRIER] Gap exit row " + gapExitRow + " protected from parking");
                }
            }

            // Build the full set of barrier positions to unfreeze.
            // BSP needs permission to disturb these positions since the boxes on them
            // are the barrier itself (normally frozen because they're on satisfied goals).
            Set<Position> unfreezePositions = new HashSet<>(barrier.clearingOrder);

            // PRE-CHECK: verify sufficient parking for ALL barrier boxes.
            // Incomplete clearing wastes steps without enabling the blocked agent's goals.
            // Also skip barriers where the clear count exceeds practical maze capacity:
            // parking many boxes in a limited maze blocks internal paths.
            if (barrier.clearingOrder.size() > 5) {
                logNormal("[PP] [CLEAR-BARRIER] Barrier too large ("
                        + barrier.clearingOrder.size() + " boxes) — skipping");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }
            List<Position> allParking = CrossColorBarrierAnalyzer.findParkingPositions(
                    barrier.clearingOrder.size(), agentReachable, currentState, level,
                    baseAvoidPositions, firstBox);
            if (allParking.size() < barrier.clearingOrder.size()) {
                logNormal("[PP] [CLEAR-BARRIER] Insufficient parking (" + allParking.size()
                        + "/" + barrier.clearingOrder.size() + ") — skipping barrier");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }

            // Track parking positions used so far — avoid them for subsequent boxes
            Set<Position> usedParkings = new HashSet<>();

            int cleared = 0;
            boolean earlyExit = false;
            for (int i = 0; i < barrier.clearingOrder.size(); i++) {
                Position boxPos = barrier.clearingOrder.get(i);

                // Verify box is still at the expected position
                Character boxAtPos = currentState.getBoxes().get(boxPos);
                if (boxAtPos == null || boxAtPos != barrier.blockingBoxType) {
                    logVerbose("[PP] [CLEAR-BARRIER] Box at " + boxPos + " changed — skipping");
                    continue;
                }

                // Check extractability for each box (reachability changes as boxes are cleared)
                Set<Position> updatedReachable = bfsReachableForClearing(
                        currentState.getAgentPosition(clearingAgent), currentState, level);
                if (cleared == 0) {
                    // Full extractability pre-check for the first box
                    if (!CrossColorBarrierAnalyzer.isBoxExtractable(boxPos, updatedReachable, currentState, level)) {
                        logNormal("[PP] [CLEAR-BARRIER] Box " + boxPos + " not extractable — stopping");
                        break;
                    }
                } else {
                    // For subsequent boxes: skip the expensive extractability pre-check.
                    // The first box's clearance confirms the gap is workable; parked boxes
                    // may block the pull-chain bypass BFS but BSP can route around them.
                    // Only verify the agent can physically reach an adjacent cell.
                    boolean canReach = false;
                    for (Direction dir : Direction.values()) {
                        if (updatedReachable.contains(boxPos.move(dir))) {
                            canReach = true;
                            break;
                        }
                    }
                    if (!canReach) {
                        logNormal("[PP] [CLEAR-BARRIER] Box " + boxPos + " not reachable by agent — stopping");
                        break;
                    }
                }

                // Compute parking INCREMENTALLY: avoid previously used parkings
                // so they don't block the pull-chain for subsequent boxes.
                Set<Position> avoidPositions = new HashSet<>(baseAvoidPositions);
                avoidPositions.addAll(usedParkings);
                List<Position> parkingCandidates = CrossColorBarrierAnalyzer.findParkingPositions(
                        1, updatedReachable, currentState, level, avoidPositions, boxPos);

                if (parkingCandidates.isEmpty()) {
                    logNormal("[PP] [CLEAR-BARRIER] No parking for " + boxPos + " — stopping");
                    break;
                }
                Position parkPos = parkingCandidates.get(0);

                logNormal("[PP] [CLEAR-BARRIER] Moving " + barrier.blockingBoxType + " from "
                        + boxPos + " to " + parkPos + " (agent " + clearingAgent + ")");

                int planSizeBefore = fullPlan.size();

                // Use higher BSP budget — pull-chain paths through narrow gaps need more exploration
                // Progressive budget: deeper boxes may need longer displacement paths
                int clearingBudget = SearchConfig.MIN_BSP_BUDGET * (6 + cleared * 2);
                List<Action> displacePath = boxSearchPlanner.planBoxDisplacementWithUnfreeze(
                        clearingAgent, boxPos, parkPos, barrier.blockingBoxType,
                        currentState, level, unfreezePositions, clearingBudget);

                if (displacePath == null || displacePath.isEmpty()) {
                    logNormal("[PP] [CLEAR-BARRIER] BSP failed for " + boxPos + " -> " + parkPos);
                    // First box failure → deeper boxes are certainly harder. Abort this barrier.
                    if (cleared == 0) {
                        logNormal("[PP] [CLEAR-BARRIER] First box failed — aborting barrier");
                        skippedBarrierClearingOrders.add(barrier.clearingOrder);
                        earlyExit = true;
                        break;
                    }
                    continue;
                }

                // Execute the displacement path
                for (Action action : displacePath) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            clearingAgent, action, currentState, level, numAgents,
                            false, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(
                            jointAction, currentState, level, clearingAgent);
                    fullPlan.add(jointAction);
                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    globalTimeStep++;
                }

                // POST-DISPLACEMENT VERIFICATION: check if box actually left its position
                // AND reached parking target. Joint action conflict resolution may have 
                // converted some BSP actions to NoOp, leaving the box at an unexpected position.
                Character boxStillThere = currentState.getBoxes().get(boxPos);
                if (boxStillThere != null && boxStillThere == barrier.blockingBoxType) {
                    logNormal("[PP] [CLEAR-BARRIER] Box still at " + boxPos
                            + " after BSP execution (conflict resolution interference) — rolling back");
                    // Rollback: remove the executed actions
                    while (fullPlan.size() > planSizeBefore) {
                        fullPlan.remove(fullPlan.size() - 1);
                        globalTimeStep--;
                    }
                    currentState = recomputeState(initialState, fullPlan, level, numAgents);
                    continue; // Don't increment cleared, don't track parking
                }
                
                // Verify box reached parking target (not stuck at intermediate position)
                Character boxAtPark = currentState.getBoxes().get(parkPos);
                if (boxAtPark == null || boxAtPark != barrier.blockingBoxType) {
                    // Box left source but didn't reach target — it's stuck at an intermediate
                    // position potentially blocking the approach corridor. Rollback.
                    logNormal("[PP] [CLEAR-BARRIER] Box left " + boxPos + " but didn't reach "
                            + parkPos + " — rolling back to avoid blocking corridor");
                    while (fullPlan.size() > planSizeBefore) {
                        fullPlan.remove(fullPlan.size() - 1);
                        globalTimeStep--;
                    }
                    currentState = recomputeState(initialState, fullPlan, level, numAgents);
                    // Try next parking candidate
                    continue;
                }

                // Track the displaced goal position
                char goalType = level.getBoxGoal(boxPos);
                if (goalType != '\0') {
                    displacedGoals.add(boxPos);
                }

                usedParkings.add(parkPos);
                cleared++;
                logNormal("[PP] [CLEAR-BARRIER] Cleared " + barrier.blockingBoxType + " from "
                        + boxPos + " in " + displacePath.size() + " steps (total cleared: " + cleared + ")");
            }

            if (cleared > 0) {
                logMinimal("[PP] Barrier clearing: " + cleared + "/" + barrier.clearingOrder.size()
                        + " boxes cleared in " + fullPlan.size() + " steps");
            }
        }

        return currentState;
    }

    /**
     * BFS reachability for clearing agent (treats all boxes as obstacles).
     */
    private Set<Position> bfsReachableForClearing(Position start, State state, Level level) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (state.hasBoxAt(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    /**
     * BFS shortest path from start to any cell adjacent to targetBox (treating 
     * boxes as walls). Returns all cells on the path (start → adjacent-to-target),
     * or empty list if unreachable. Used to compute the approach corridor that
     * must be kept clear for subsequent barrier box clearing.
     */
    private List<Position> computeApproachPath(Position start, Position targetBox,
                                                State state, Level level) {
        Map<Position, Position> parent = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        parent.put(start, null);
        queue.add(start);

        Position reached = null;
        while (!queue.isEmpty() && reached == null) {
            Position current = queue.poll();
            // Check if current is adjacent to the target box
            for (Direction dir : Direction.values()) {
                if (current.move(dir).equals(targetBox)) {
                    reached = current;
                    break;
                }
            }
            if (reached != null) break;

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (parent.containsKey(next)) continue;
                if (level.isWall(next)) continue;
                if (state.hasBoxAt(next)) continue;
                parent.put(next, current);
                queue.add(next);
            }
        }

        if (reached == null) return Collections.emptyList();

        // Reconstruct path
        List<Position> path = new ArrayList<>();
        for (Position p = reached; p != null; p = parent.get(p)) {
            path.add(p);
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * MAPF FIX: Validates plan correctness and removes redundant actions.
     * 
     * 1. Collision check: Ensure no two agents occupy same cell at same time
     * 2. Redundant NoOp removal: Remove trailing NoOps when agents are at goals
     */
    private List<Action[]> validateAndOptimizePlan(List<Action[]> plan, State initialState, 
                                                   Level level, int numAgents) {
        if (plan == null || plan.isEmpty()) return plan;
        
        // Step 1: Validate (detect collisions)
        State state = initialState;
        for (int step = 0; step < plan.size(); step++) {
            Action[] actions = plan.get(step);
            
            // Check for vertex conflicts (two agents at same position)
            Set<Position> nextPositions = new HashSet<>();
            for (int a = 0; a < numAgents; a++) {
                Position nextPos = computeNextPosition(state, a, actions[a], level);
                if (nextPositions.contains(nextPos)) {
                    logVerbose("[PP] WARNING: Collision detected at step " + step + " at " + nextPos);
                }
                nextPositions.add(nextPos);
            }
            
            // Apply actions
            state = applyJointAction(actions, state, level, numAgents);
        }
        
        // Step 2: Optimize - remove trailing NoOps
        int lastMeaningfulStep = plan.size() - 1;
        while (lastMeaningfulStep >= 0) {
            Action[] actions = plan.get(lastMeaningfulStep);
            boolean allNoOp = true;
            for (Action a : actions) {
                if (a.type != Action.ActionType.NOOP) {
                    allNoOp = false;
                    break;
                }
            }
            if (!allNoOp) break;
            lastMeaningfulStep--;
        }
        
        if (lastMeaningfulStep < plan.size() - 1) {
            int removed = plan.size() - lastMeaningfulStep - 1;
            logNormal("[PP] Optimized: removed " + removed + " trailing NoOp steps");
            return plan.subList(0, lastMeaningfulStep + 1);
        }
        
        return plan;
    }
    
    /**
     * Computes the next position of an agent after applying an action.
     */
    private Position computeNextPosition(State state, int agentId, Action action, Level level) {
        Position current = state.getAgentPosition(agentId);
        if (action == null || action.type == Action.ActionType.NOOP) {
            return current;
        }
        if (action.type == Action.ActionType.MOVE) {
            return current.move(action.agentDir);
        }
        // Push/Pull: agent moves
        return current.move(action.agentDir);
    }
    
    /**
     * MAPF FIX: Get cached subgoal order or compute once.
     * Priority is fixed at start - only filter out completed goals.
     * UPDATED: Allows refreshing if list is empty but goal state not reached (Phase 1 -> Phase 2).
     */
    private List<Subgoal> getOrComputeSubgoalOrder(State currentState, Level level) {
        if (cachedSubgoalOrder == null) {
            computeAndCacheSubgoals(currentState, level);
        }
        
        // Filter out completed goals from cached order
        List<Subgoal> remaining = filterUnsatisfiedSubgoals(cachedSubgoalOrder, currentState, level);
        
        // Fail-safe: If ran out of goals but not at goal state, refresh!
        // This handles the transition from Box Goals (Phase 1) to Agent Goals (Phase 2)
        // Also: re-enable barrier-displaced goals so CYAN can return A-boxes.
        if (remaining.isEmpty() && !currentState.isGoalState(level)) {
            logVerbose("[PP] Phase switch: refreshing subgoal list");
            if (!displacedGoals.isEmpty()) {
                logVerbose("[PP] Re-enabling " + displacedGoals.size() + " barrier-displaced goals for return phase");
                displacedGoals.clear();
            }
            computeAndCacheSubgoals(currentState, level);
            remaining = filterUnsatisfiedSubgoals(cachedSubgoalOrder, currentState, level);
        }
        
        return remaining;
    }
    
    private void computeAndCacheSubgoals(State state, Level level) {
        // Hungarian (SELECTION: which box fills which goal) and dependency analysis
        // (ORDER: what order to fill goals) are ORTHOGONAL concerns — both always active.
        // Hungarian's cost matrix is pure box-to-goal distance with NO execution order
        // assumptions. planSubgoal() has a box-retry fallback: if Hungarian's globally-
        // optimal pick fails BSP (serial execution mismatch), it automatically falls
        // through to greedy Layer 2 candidates.
        subgoalManager.computeHungarianAssignment(state, level, completedBoxGoals);
        
        cachedSubgoalOrder = subgoalManager.getUnsatisfiedSubgoals(state, level, completedBoxGoals);
        sortSubgoals(cachedSubgoalOrder, state, level);
        
        // Physical blocking sort: reorder subgoals where filling goal A would place a box
        // on goal B's box-to-goal path. Goal B should be filled BEFORE goal A.
        applyPhysicalBlockingSort(cachedSubgoalOrder, state, level);
        
        logGoalOrder(cachedSubgoalOrder);
    }
    
    private List<Subgoal> filterUnsatisfiedSubgoals(List<Subgoal> source, State currentState, Level level) {
        List<Subgoal> strictRemaining = new ArrayList<>();
        List<Subgoal> allUnsatisfied = new ArrayList<>();
        
        for (Subgoal sg : source) {
            // Basic satisfaction check
            if (completedBoxGoals.contains(sg.goalPos)) continue;
            // Barrier-displaced goals: boxes intentionally cleared to open a path
            // for a blocked agent. Defer re-filling until the blocked agent's goals
            // are achieved (Phase switch will clear displacedGoals).
            if (!sg.isAgentGoal && displacedGoals.contains(sg.goalPos)) continue;

            boolean satisfied;
            if (sg.isAgentGoal) {
                Position agentPos = currentState.getAgentPosition(sg.agentId);
                satisfied = agentPos.equals(sg.goalPos);
            } else {
                Character boxAtGoal = currentState.getBoxes().get(sg.goalPos);
                satisfied = (boxAtGoal != null && boxAtGoal == sg.boxType);
            }
            if (satisfied) continue;

            allUnsatisfied.add(sg);
            
            // Strict dependency check
            if (areDependenciesMet(sg.goalPos, level)) {
                strictRemaining.add(sg);
            }
        }
        
        // Normal path: some goals have all dependencies met
        if (!strictRemaining.isEmpty()) {
            return strictRemaining;
        }
        
        // Cycle fallback: ALL goals have unmet dependencies (circular dependency deadlock).
        // In push-pull domain, no filled goal is truly permanent — boxes can be pulled away.
        // Graceful degradation: return ALL unsatisfied goals, sorted by fewest unmet deps first.
        if (!allUnsatisfied.isEmpty()) {
            allUnsatisfied.sort((a, b) -> {
                int unmetA = countUnmetDependencies(a.goalPos, level);
                int unmetB = countUnmetDependencies(b.goalPos, level);
                return Integer.compare(unmetA, unmetB);
            });
        }
        
        return allUnsatisfied;
    }
    
    /**
     * Counts how many dependencies for a goal are NOT yet satisfied.
     * Used for cycle-fallback ordering: prefer goals with fewer blockers.
     */
    private int countUnmetDependencies(Position goal, Level level) {
        Set<Position> deps = goalDependsOn.get(goal);
        if (deps == null || deps.isEmpty()) return 0;
        int count = 0;
        for (Position dep : deps) {
            boolean isBoxGoal = level.getBoxGoal(dep) != '\0';
            if (isBoxGoal) {
                if (!completedBoxGoals.contains(dep)) count++;
            } else {
                if (!completedAgentGoals.contains(dep)) count++;
            }
        }
        return count;
    }

    /**
     * Checks if all dependencies for a goal are satisfied.
     * A dependency is satisfied if the prerequisite goal is completed.
     */
    private boolean areDependenciesMet(Position goal, Level level) {
        Set<Position> deps = goalDependsOn.get(goal);
        if (deps == null || deps.isEmpty()) return true;

        for (Position dep : deps) {
            boolean isBoxGoal = level.getBoxGoal(dep) != '\0';
            if (isBoxGoal) {
                if (!completedBoxGoals.contains(dep)) return false;
            } else {
                // Agent dependency: less common, but check if satisfied
                // Note: completedAgentGoals tracks *ever* completed, but agent might move.
                // However, for dependency logic, we assume "Completed" means "Done".
                if (!completedAgentGoals.contains(dep)) return false;
            }
        }
        return true;
    }
    
    /** Sort subgoals based on the current OrderingMode. */
    private void sortSubgoals(List<Subgoal> subgoals, State state, Level level) {
        switch (orderingMode) {
            case REVERSE_TOPOLOGICAL:
                if (precomputedGoalOrder != null && !precomputedGoalOrder.isEmpty()) {
                    Map<Position, Integer> orderMap = new HashMap<>();
                    for (int i = 0; i < precomputedGoalOrder.size(); i++) {
                        orderMap.put(precomputedGoalOrder.get(i), i);
                    }
                    // Reverse: higher index first
                    subgoals.sort((a, b) -> {
                        int orderA = orderMap.getOrDefault(a.goalPos, -1);
                        int orderB = orderMap.getOrDefault(b.goalPos, -1);
                        return Integer.compare(orderB, orderA);
                    });
                    logVerbose("[PP] Sorted subgoals in REVERSE topological order");
                } else {
                    sortByDifficulty(subgoals, state, level);
                }
                break;
                
            case DISTANCE_GREEDY:
                // Sort by estimated difficulty: easiest (nearest) first
                sortByDifficulty(subgoals, state, level);
                logVerbose("[PP] Sorted subgoals by DISTANCE_GREEDY (nearest first)");
                break;
                
            case RANDOM:
                Collections.shuffle(subgoals, random);
                logVerbose("[PP] Sorted subgoals in RANDOM order");
                break;
                
            case TOPOLOGICAL:
            default:
                if (precomputedGoalOrder != null && !precomputedGoalOrder.isEmpty()) {
                    Map<Position, Integer> orderMap = new HashMap<>();
                    for (int i = 0; i < precomputedGoalOrder.size(); i++) {
                        orderMap.put(precomputedGoalOrder.get(i), i);
                    }
                    
                    subgoals.sort((a, b) -> {
                        int orderA = orderMap.getOrDefault(a.goalPos, Integer.MAX_VALUE);
                        int orderB = orderMap.getOrDefault(b.goalPos, Integer.MAX_VALUE);
                        return Integer.compare(orderA, orderB);
                    });
                } else {
                    sortByDifficulty(subgoals, state, level);
                }
                break;
        }
    }
    
    /** Sort subgoals by estimated difficulty (easiest first). */
    private void sortByDifficulty(List<Subgoal> subgoals, State state, Level level) {
        final State s = state;
        final Level lv = level;
        subgoals.sort((a, b) -> {
            int diffA = subgoalManager.estimateSubgoalDifficulty(a, s, lv, completedBoxGoals);
            int diffB = subgoalManager.estimateSubgoalDifficulty(b, s, lv, completedBoxGoals);
            return Integer.compare(diffA, diffB);
        });
    }
    
    /**
     * Computes a dynamic BSP search budget based on estimated box-to-goal distance.
     * Short distances get a smaller budget (fail/succeed fast), long distances get
     * a larger budget (more room to explore complex maneuvers).
     * 
     * Formula: clamp(MIN_BSP_BUDGET + distance * BSP_BUDGET_PER_DISTANCE, MIN, MAX)
     * e.g. dist=1→8K, dist=5→12K, dist=8→21K (≈current 20K), dist=15→38K, dist=20→40K
     */
    private int computeDynamicBspBudget(Position boxPos, Position goalPos) {
        int distance = boxPos.manhattanDistance(goalPos);
        int budget = SearchConfig.MIN_BSP_BUDGET + distance * SearchConfig.BSP_BUDGET_PER_DISTANCE;
        return Math.max(SearchConfig.MIN_BSP_BUDGET, Math.min(SearchConfig.MAX_BSP_BUDGET, budget));
    }
    
    /**
     * Physical blocking sort: detects cross-color physical blocking between subgoals
     * and reorders to avoid deadlocks.
     * 
     * For each pair (A, B) of box subgoals handled by DIFFERENT color agents:
     *   If A's goalPos lies on B's box→goal path, then B should be done BEFORE A
     *   (because filling A would block B's path).
     * 
     * Uses bubble-sort style promotion: if B is blocked by A and B comes after A,
     * swap them. Only applies to cross-color pairs (same-color handled by dependency graph).
     * Limited to O(n²) pairwise checks with max n² swaps.
     */
    private void applyPhysicalBlockingSort(List<Subgoal> subgoals, State state, Level level) {
        int n = subgoals.size();
        if (n < 2) return;
        
        // Build box→goal paths for each subgoal (lazily cached)
        Map<Integer, Set<Position>> pathCells = new HashMap<>();
        
        boolean changed = true;
        int passes = 0;
        int maxPasses = n; // limit iterations
        
        while (changed && passes < maxPasses) {
            changed = false;
            passes++;
            
            for (int i = 0; i < n - 1; i++) {
                Subgoal a = subgoals.get(i);
                if (a.isAgentGoal) continue;
                
                Color colorA = level.getAgentColor(a.agentId);
                
                for (int j = i + 1; j < n; j++) {
                    Subgoal b = subgoals.get(j);
                    if (b.isAgentGoal) continue;
                    
                    Color colorB = level.getAgentColor(b.agentId);
                    
                    // Only check cross-color pairs
                    if (colorA != null && colorA.equals(colorB)) continue;
                    
                    // Does A's goalPos block B's box→goal path?
                    Set<Position> bPath = pathCells.computeIfAbsent(j, idx -> {
                        Position boxPos = subgoalManager.findBestBoxForGoal(b, state, level, subgoals, completedBoxGoals);
                        if (boxPos == null) return Collections.emptySet();
                        List<Position> path = pathAnalyzer.findPathIgnoringDynamicObstacles(boxPos, b.goalPos, level);
                        return (path != null) ? new HashSet<>(path) : Collections.emptySet();
                    });
                    
                    if (bPath.contains(a.goalPos)) {
                        // A's goal blocks B's path → B should be done before A
                        // Promote B to position i (move A down)
                        subgoals.remove(j);
                        subgoals.add(i, b);
                        pathCells.clear(); // invalidate path cache after reorder
                        changed = true;
                        logVerbose("[PP] [PHYS-SORT] Promoted " + b.boxType + "->" + b.goalPos 
                                + " before " + a.boxType + "->" + a.goalPos + " (cross-color blocking)");
                        break; // restart inner loop from new i
                    }
                }
                if (changed) break; // restart outer loop
            }
        }
    }
    
    /** Log the goal execution order. Internal planning detail, verbose only. */
    private void logGoalOrder(List<Subgoal> subgoals) {
        if (!SearchConfig.isVerbose()) return;
        System.err.println("[PP] Task Plan (" + subgoals.size() + " subgoals):");
        for (int i = 0; i < subgoals.size(); i++) {
            Subgoal sg = subgoals.get(i);
            String taskType = sg.isAgentGoal ? "AgentGoal" : ("Box " + sg.boxType);
            System.err.println("  " + (i+1) + ". " + taskType + " -> " + sg.goalPos + " [Agent " + sg.agentId + "]");
        }
    }

    /** Try to execute subgoals in priority order. Returns true if progress made. */
    private boolean tryExecuteSubgoals(List<Subgoal> subgoals, List<Action[]> fullPlan, 
            State currentState, Level level, int numAgents, State initialState) {
        
        int skippedByDeps = 0;
        for (Subgoal subgoal : subgoals) {
            // MAPF FIX: Strictly enforce goal dependencies.
            // Even if a lower-priority goal is locally executable, we must not execute it
            // if it depends on a higher-priority goal that is not yet complete.
            if (goalDependsOn.containsKey(subgoal.goalPos)) {
                boolean depMet = true;
                for (Position dep : goalDependsOn.get(subgoal.goalPos)) {
                    boolean isBoxG = level.getBoxGoal(dep.row, dep.col) != '\0';
                    if (isBoxG && !completedBoxGoals.contains(dep)) {
                        depMet = false;
                        break;
                    } else if (!isBoxG && !completedAgentGoals.contains(dep)) {
                        depMet = false;
                        break;
                    }
                }
                if (!depMet) {
                    logVerbose("[PP] Skipped " + (subgoal.isAgentGoal ? "Agent" : "Box " + subgoal.boxType)
                            + " -> " + subgoal.goalPos + " (unmet deps)");
                    skippedByDeps++; continue;
                }
            }

            // Goal-level cycle detection: skip goals that have been completed too many times.
            // This breaks push-pull cycles where a box is pushed to goal, pulled off by
            // another agent's recovery, then pushed back again indefinitely.
            if (!subgoal.isAgentGoal) {
                int completions = goalCompletionCount.getOrDefault(subgoal.goalPos, 0);
                if (completions >= MAX_GOAL_COMPLETIONS) {
                    logVerbose("[PP] [CYCLE-SKIP] Goal " + subgoal.goalPos + " (box " + subgoal.boxType
                            + ") already completed " + completions + " times — skipping to break cycle");
                    continue;
                }
            }

            // Task-Aware: Pass the full list of subgoals for global allocation checking
            List<Action> path = planSubgoal(subgoal, currentState, level, subgoals);
            
            if (path != null && !path.isEmpty()) {
                
                // Snapshot plan size for potential rollback (agent-trap detection)
                int planSizeBefore = fullPlan.size();
                
                // Plan parallel subgoals for agents in independent components
                planIndependentAgents(subgoal, subgoals, currentState, level);
                
                // Record agent path in reservation table for space-time collision avoidance
                List<Position> agentPath = extractAgentPath(subgoal.agentId, currentState, path, level);
                boolean permanentEnd = subgoal.isAgentGoal;
                reservationTable.reservePath(subgoal.agentId, agentPath, globalTimeStep, permanentEnd);
                
                // Execute the path
                State tempState = currentState;
                for (Action action : path) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            subgoal.agentId, action, tempState, level, numAgents, subgoal.isAgentGoal, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, subgoal.agentId);
                    fullPlan.add(jointAction);
                    tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    globalTimeStep++;
                    planMerger.updatePlanIndexes(jointAction, numAgents, subgoal.agentId);
                }
                
                // POST-PLAN REGRESSION GUARD: Check if executing this path disturbed
                // any previously completed box goals. If so, rollback — the cure is worse
                // than the disease. This is the primary defense against push-then-pull cycles.
                // Only applies when we had completed goals to protect (Round 2b/3 may have
                // relaxed frozen protections, allowing BSP to disturb them).
                if (!completedBoxGoals.isEmpty()) {
                    List<Position> regressedGoals = detectRegressedGoals(tempState, level);
                    // Exclude goals that were intentionally displaced by tryRecovery.
                    // Those positions are expected to be empty — not a regression.
                    regressedGoals.removeAll(displacedGoals);
                    if (!regressedGoals.isEmpty()) {
                        logNormal(getName() + ": [REGRESS] Path for " 
                                + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType)
                                + " -> " + subgoal.goalPos + " disturbed " + regressedGoals.size() 
                                + " completed goal(s): " + regressedGoals + " — rollback");
                        // Rollback: remove the executed actions from fullPlan
                        while (fullPlan.size() > planSizeBefore) {
                            fullPlan.remove(fullPlan.size() - 1);
                            globalTimeStep--;
                        }
                        // Invalidate stored plans — state rolled back
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                        continue; // try next subgoal in priority order
                    }
                }
                
                // Check if any parallel (stored plan) agents completed their goals
                checkStoredPlanCompletions(tempState, level);
                
                // Verify goal was reached
                boolean reached = verifyGoalReached(subgoal, tempState, level);
                if (reached) {
                    // Pukoban agent-trap detection: after filling a box goal, check if the 
                    // agent is trapped in a dead-end disconnected from remaining tasks.
                    // This is a POST-PLAN validation (not static analysis) because it depends
                    // on the actual agent position and world state after execution.
                    if (!subgoal.isAgentGoal && wouldTrapAgent(subgoal, tempState, level, subgoals, currentState)) {
                        // Agent would be trapped — skip this subgoal, let PP try alternatives
                        logNormal(getName() + ": [TRAP] Skipping " + subgoal.boxType + " -> " + subgoal.goalPos
                                + " — agent " + subgoal.agentId + " would be trapped after filling");
                        // Rollback: remove the executed actions from fullPlan
                        // Rollback: remove the executed actions from fullPlan
                        while (fullPlan.size() > planSizeBefore) {
                            fullPlan.remove(fullPlan.size() - 1);
                            globalTimeStep--;
                        }
                        // Invalidate stored plans — state rolled back
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                        continue; // try next subgoal in priority order
                    }
                    
                    // Mark box goal as completed
                    if (!subgoal.isAgentGoal) {
                        completedBoxGoals.add(subgoal.goalPos);
                        // NOTE: displacedGoals NOT cleared here. Barrier-displaced goals
                        // stay deferred until the phase switch (all non-displaced goals
                        // satisfied) to prevent PP from undoing barrier clearing.
                        
                        // Goal-level cycle detection: track completion count
                        int count = goalCompletionCount.getOrDefault(subgoal.goalPos, 0) + 1;
                        goalCompletionCount.put(subgoal.goalPos, count);
                        if (count > MAX_GOAL_COMPLETIONS) {
                            logVerbose("[PP] CYCLE DETECTED: goal " + subgoal.goalPos 
                                    + " completed " + count + " times (box " + subgoal.boxType + ")");
                        }
                        
                        // Invalidate Hungarian cache — world state changed, assignment may be stale
                        subgoalManager.invalidateHungarianCache();
                        // Force subgoal list refresh: completing a box goal may make new agent 
                        // goals eligible (agents whose same-color box tasks are now all done).
                        // Without this, filterUnsatisfiedSubgoals can only REMOVE completed goals 
                        // from the cache — it cannot ADD newly-eligible agent goals that were 
                        // excluded during initial cache computation.
                        cachedSubgoalOrder = null;
                    }
                    
                    // Check if any previously completed goals were disturbed (soft-unlock)
                    revalidateCompletedGoals(tempState, level);
                    
                    // Proactive Yielding: park agent at safe position after completing subgoal.
                    // This prevents agents from blocking corridors/chokepoints for later subgoals.
                    // CRITICAL FIX: Do NOT park an agent that just completed its own agent goal.
                    // Parking moves it off the goal position, causing it to be re-added to
                    // unsatisfied subgoals next iteration → infinite loop (park ↔ re-solve).
                    if (!subgoal.isAgentGoal) {
                        // Pass remaining subgoals for task-aware critical path checking
                        List<Subgoal> remainingSubgoals = new ArrayList<>();
                        for (Subgoal sg : subgoals) {
                            if (sg != subgoal) remainingSubgoals.add(sg);
                        }
                        tempState = parkAgentAfterSubgoal(subgoal.agentId, tempState, level, fullPlan, numAgents, remainingSubgoals);
                    }
                    
                    // Track phantom progress: re-solving an agent goal that was already done
                    if (subgoal.isAgentGoal) {
                        lastProgressWasPhantom = completedAgentGoals.contains(subgoal.goalPos);
                        completedAgentGoals.add(subgoal.goalPos);
                    } else {
                        lastProgressWasPhantom = false;
                    }
                    
                    // PHANTOM = re-solving completed agent goal (debug info, verbose only)
                    if (lastProgressWasPhantom) {
                        logVerbose(getName() + ": [OK] " + 
                            (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) +
                            " -> " + subgoal.goalPos + " (" + path.size() + " steps) [PHANTOM]");
                    } else {
                        logMinimal(getName() + ": [OK] " + 
                            (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) +
                            " -> " + subgoal.goalPos + " (" + path.size() + " steps)");
                    }
                    return true;
                }
            } else {
                // PATH CLEARING: When BSP fails for a box subgoal, other boxes may
                // physically block the path. Try to identify and move them.
                // Example: In MADS, box C at (10,3) blocks box A's path to (10,1).
                // Agent 1 (red) can't push C (pink), so BSP fails. We need agent 2
                // (pink) to move C out of the way first.
                if (!subgoal.isAgentGoal) {
                    State clearedState = tryPathClearing(subgoal, currentState, level, 
                            fullPlan, numAgents, subgoals);
                    if (clearedState != null) {
                        // Clearing moved some blocking boxes. Retry planSubgoal.
                        path = planSubgoal(subgoal, clearedState, level, subgoals);
                        if (path != null && !path.isEmpty()) {
                            logNormal("[PP] [CLEAR] Path found after clearing: " + path.size() + " steps");
                            currentState = clearedState;
                            // Fall through to the execution block below
                        }
                    }
                }
                
                // Execute path if clearing succeeded
                if (path != null && !path.isEmpty()) {
                    int planSizeBefore = fullPlan.size();
                    List<Position> agentPath = extractAgentPath(subgoal.agentId, currentState, path, level);
                    reservationTable.reservePath(subgoal.agentId, agentPath, globalTimeStep, subgoal.isAgentGoal);
                    
                    State tempState = currentState;
                    for (Action action : path) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                                subgoal.agentId, action, tempState, level, numAgents, subgoal.isAgentGoal, completedBoxGoals);
                        jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, subgoal.agentId);
                        fullPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                        globalTimeStep++;
                        planMerger.updatePlanIndexes(jointAction, numAgents, subgoal.agentId);
                    }
                    
                    // Same post-plan checks as normal path
                    if (!completedBoxGoals.isEmpty()) {
                        List<Position> regressedGoals = detectRegressedGoals(tempState, level);
                        regressedGoals.removeAll(displacedGoals);
                        if (!regressedGoals.isEmpty()) {
                            logNormal(getName() + ": [REGRESS] Cleared path disturbed " 
                                    + regressedGoals.size() + " goal(s) — rollback");
                            while (fullPlan.size() > planSizeBefore) {
                                fullPlan.remove(fullPlan.size() - 1);
                                globalTimeStep--;
                            }
                            continue;
                        }
                    }
                    
                    boolean reached = verifyGoalReached(subgoal, tempState, level);
                    if (reached) {
                        if (!subgoal.isAgentGoal) {
                            completedBoxGoals.add(subgoal.goalPos);
                            // NOTE: displacedGoals NOT cleared here — see phase switch.
                            int count = goalCompletionCount.getOrDefault(subgoal.goalPos, 0) + 1;
                            goalCompletionCount.put(subgoal.goalPos, count);
                            subgoalManager.invalidateHungarianCache();
                            cachedSubgoalOrder = null;
                        }
                        revalidateCompletedGoals(tempState, level);
                        
                        // Borrow-and-return: re-plan displaced goals back
                        if (!lastClearingDisplacedGoals.isEmpty()) {
                            tempState = returnDisplacedGoals(tempState, level, fullPlan, numAgents);
                        }
                        
                        if (!subgoal.isAgentGoal) {
                            List<Subgoal> rem = new ArrayList<>();
                            for (Subgoal sg : subgoals) { if (sg != subgoal) rem.add(sg); }
                            parkAgentAfterSubgoal(subgoal.agentId, tempState, level, fullPlan, numAgents, rem);
                        }
                        lastProgressWasPhantom = subgoal.isAgentGoal && completedAgentGoals.contains(subgoal.goalPos);
                        if (subgoal.isAgentGoal) completedAgentGoals.add(subgoal.goalPos);
                        else lastProgressWasPhantom = false;
                        logMinimal(getName() + ": [OK] " + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) 
                                + " -> " + subgoal.goalPos + " (" + path.size() + " steps) [CLEARED]");
                        return true;
                    }
                }
            }
        }
        logVerbose("[PP] tryExecuteSubgoals: no progress (skippedByDeps=" + skippedByDeps 
                + ", total=" + subgoals.size() + ")");;
        return false;
    }
    
    /**
     * Path Clearing: when BSP fails because OTHER boxes physically block the target
     * box's path to its goal, this method identifies those blockers and uses the
     * appropriate same-color agent to push them out of the way.
     * 
     * Algorithm:
     * 1. Compute the static BFS path from box to goal (ignoring all dynamic obstacles)
     * 2. Find OTHER boxes that sit on this path (or on required agent push positions)
     * 3. For each blocker: find same-color agent, find safe clearing position via BFS,
     *    use BSP to plan displacement, execute
     * 4. Return updated state if any clearing was done, null otherwise
     * 
     * This handles the classic multi-agent box-blocking pattern: e.g., in MADS,
     * box C (pink) at (10,3) blocks red agent's path for box A to goal (10,1).
     */
    private State tryPathClearing(Subgoal subgoal, State state, Level level,
            List<Action[]> fullPlan, int numAgents, List<Subgoal> allSubgoals) {
        
        Position boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
        if (boxPos == null) return null;
        
        // Step 1: Find the static path from box to goal (ignoring all movable objects)
        List<Position> idealPath = pathAnalyzer.findPathIgnoringDynamicObstacles(boxPos, subgoal.goalPos, level);
        if (idealPath == null || idealPath.isEmpty()) return null;
        
        // Build the "danger zone": all positions on the path + agent push positions
        Set<Position> dangerZone = new HashSet<>(idealPath);
        for (int i = 0; i < idealPath.size() - 1; i++) {
            Position from = idealPath.get(i);
            Position to = idealPath.get(i + 1);
            int dr = to.row - from.row;
            int dc = to.col - from.col;
            // Agent push position: opposite side of push direction
            Position agentPush = new Position(from.row - dr, from.col - dc);
            if (!level.isWall(agentPush)) {
                dangerZone.add(agentPush);
            }
        }
        
        // Step 2: Find boxes that are in the danger zone (excluding the target box itself)
        List<Position> blockerPositions = new ArrayList<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position bPos = entry.getKey();
            if (bPos.equals(boxPos)) continue;
            if (dangerZone.contains(bPos)) {
                blockerPositions.add(bPos);
            }
        }
        
        if (blockerPositions.isEmpty()) return null;
        
        logVerbose("[PP] [CLEAR] Found " + blockerPositions.size() + " box(es) blocking path for "
                + subgoal.boxType + " -> " + subgoal.goalPos + ": " + blockerPositions);
        
        // Step 3: For each blocker, find its agent and a safe clearing position
        State currentState = state;
        boolean anyCleared = false;
        lastClearingDisplacedGoals.clear();
        int displacementCount = 0;
        
        for (Position blockerPos : blockerPositions) {
            Character blockerType = currentState.getBoxes().get(blockerPos);
            if (blockerType == null) continue; // already moved
            
            Color blockerColor = level.getBoxColor(blockerType);
            if (blockerColor == null) continue;
            
            // Find agent that can push this box
            int clearingAgent = -1;
            for (int a = 0; a < level.getNumAgents(); a++) {
                if (blockerColor.equals(level.getAgentColor(a))) {
                    clearingAgent = a;
                    break;
                }
            }
            if (clearingAgent < 0) continue;
            
            // Find safe clearing position: BFS from blocker to find nearest cell NOT in dangerZone
            Position clearTarget = findClearingPosition(blockerPos, dangerZone, currentState, level);
            if (clearTarget == null) {
                logVerbose("[PP] [CLEAR] No safe clearing position for " + blockerType + " at " + blockerPos);
                continue;
            }
            
            logVerbose("[PP] [CLEAR] Moving " + blockerType + " from " + blockerPos + " to " + clearTarget
                    + " (agent " + clearingAgent + ")");
            
            // Plan displacement
            List<Action> displacePath = boxSearchPlanner.planBoxDisplacement(
                    clearingAgent, blockerPos, clearTarget, blockerType, currentState, level);
            
            if (displacePath != null && !displacePath.isEmpty()) {
                // Check if this displacement would exceed the cap
                // Count goals that would be displaced by moving this blocker
                boolean blockerOnCompletedGoal = completedBoxGoals.contains(blockerPos);
                if (blockerOnCompletedGoal) {
                    displacementCount++;
                    if (displacementCount > MAX_CLEARING_DISPLACEMENTS) {
                        logVerbose("[PP] [CLEAR] Displacement cap reached (" + MAX_CLEARING_DISPLACEMENTS + "), skipping remaining");
                        break;
                    }
                }
                
                // Execute the clearing path
                for (Action action : displacePath) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            clearingAgent, action, currentState, level, numAgents, false, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, clearingAgent);
                    fullPlan.add(jointAction);
                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    globalTimeStep++;
                }
                anyCleared = true;
                
                // Track displaced goals for borrow-and-return
                if (blockerOnCompletedGoal) {
                    lastClearingDisplacedGoals.add(blockerPos);
                    displacedGoals.add(blockerPos);
                    logVerbose("[PP] [CLEAR] Displaced completed goal at " + blockerPos);
                }
                
                logVerbose("[PP] [CLEAR] Cleared " + blockerType + " in " + displacePath.size() + " steps");
            } else {
                logVerbose("[PP] [CLEAR] BSP can't plan displacement for " + blockerType + " at " + blockerPos);
            }
        }
        
        // Phase B: Agent→box approach path clearing
        // Only runs if box→goal clearing didn't make progress AND the agent actually
        // can't reach the box (BFS through real obstacles fails).
        if (!anyCleared) {
            Position agentPos = currentState.getAgentPosition(subgoal.agentId);
            Color agentColor = level.getAgentColor(subgoal.agentId);
            
            // Check if agent can actually reach the box
            List<Action> agentPath = pathAnalyzer.planAgentPath(
                    subgoal.agentId, boxPos, currentState, level, numAgents);
            
            if (agentPath == null) {
                // Agent can't reach box — find different-color boxes blocking the path
                List<Position> agentToBoxPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        agentPos, boxPos, level);
                
                if (agentToBoxPath != null) {
                    for (Position pathCell : agentToBoxPath) {
                        Character boxAtCell = currentState.getBoxes().get(pathCell);
                        if (boxAtCell != null && level.getBoxColor(boxAtCell) != agentColor) {
                            // Find same-color agent for this blocker
                            Color bColor = level.getBoxColor(boxAtCell);
                            int clearAgent = -1;
                            for (int a = 0; a < level.getNumAgents(); a++) {
                                if (bColor.equals(level.getAgentColor(a))) {
                                    clearAgent = a;
                                    break;
                                }
                            }
                            if (clearAgent < 0) continue;
                            
                            // Clear it away from both dangerZone and agent→box path
                            Set<Position> excludeZone = new HashSet<>(dangerZone);
                            excludeZone.addAll(agentToBoxPath);
                            Position clearTarget = findClearingPosition(pathCell, excludeZone, currentState, level);
                            if (clearTarget == null) {
                                clearTarget = findClearingPosition(pathCell, dangerZone, currentState, level);
                            }
                            if (clearTarget == null) continue;
                            
                            logVerbose("[PP] [CLEAR-A2B] Moving " + boxAtCell + " from " + pathCell 
                                    + " to " + clearTarget + " (agent " + clearAgent + ")");
                            
                            List<Action> displacePath = boxSearchPlanner.planBoxDisplacement(
                                    clearAgent, pathCell, clearTarget, boxAtCell, currentState, level);
                            
                            if (displacePath != null && !displacePath.isEmpty()) {
                                // Check displacement cap
                                boolean onGoal = completedBoxGoals.contains(pathCell);
                                if (onGoal) {
                                    displacementCount++;
                                    if (displacementCount > MAX_CLEARING_DISPLACEMENTS) {
                                        logVerbose("[PP] [CLEAR-A2B] Displacement cap reached");
                                        break;
                                    }
                                }
                                
                                for (Action action : displacePath) {
                                    Action[] jointAction = planMerger.createJointActionWithMerging(
                                            clearAgent, action, currentState, level, numAgents, false, completedBoxGoals);
                                    jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, clearAgent);
                                    fullPlan.add(jointAction);
                                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                                    globalTimeStep++;
                                }
                                anyCleared = true;
                                
                                if (onGoal) {
                                    lastClearingDisplacedGoals.add(pathCell);
                                    displacedGoals.add(pathCell);
                                    logVerbose("[PP] [CLEAR-A2B] Displaced completed goal at " + pathCell);
                                }
                                
                                logVerbose("[PP] [CLEAR-A2B] Cleared " + boxAtCell + " in " + displacePath.size() + " steps");
                            }
                        }
                    }
                }
            }
        }
        
        return anyCleared ? currentState : null;
    }
    
    /**
     * BFS to find the nearest safe clearing position for a blocker box.
     * The position must be:
     * - Not in the danger zone (path + agent push positions)
     * - Not a wall
     * - Not occupied by another box
     * - Not an agent position
     * - Preferably NOT on a completed goal (borrow-and-return: minimize displacements)
     */
    private Position findClearingPosition(Position blockerPos, Set<Position> dangerZone,
            State state, Level level) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        visited.add(blockerPos);
        queue.add(blockerPos);
        
        Position firstGoalFallback = null; // first valid pos that's on a completed goal
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next) || level.isWall(next)) continue;
                visited.add(next);
                
                // Valid clearing position: not in danger zone, not occupied
                if (!dangerZone.contains(next) 
                        && !state.getBoxes().containsKey(next)
                        && !isAgentAt(next, state, level)) {
                    // Prefer positions NOT on completed goals
                    if (!completedBoxGoals.contains(next)) {
                        return next;
                    }
                    // Remember first goal-position as fallback
                    if (firstGoalFallback == null) {
                        firstGoalFallback = next;
                    }
                }
                queue.add(next);
            }
        }
        return firstGoalFallback; // may be null if nothing found at all
    }
    
    /** Check if any agent is at the given position. */
    private boolean isAgentAt(Position pos, State state, Level level) {
        for (int a = 0; a < state.getNumAgents(); a++) {
            if (state.getAgentPosition(a).equals(pos)) return true;
        }
        return false;
    }
    
    /**
     * Borrow-and-return: after path clearing + BSP succeeds, try to push
     * displaced boxes back to their original goal positions.
     * Best-effort: if any return fails, the goal will be re-planned normally
     * in a later iteration (revalidateCompletedGoals removed it from completedBoxGoals).
     */
    private State returnDisplacedGoals(State state, Level level, List<Action[]> fullPlan, int numAgents) {
        State currentState = state;
        int returned = 0;
        
        for (Position goalPos : lastClearingDisplacedGoals) {
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') continue;
            
            // Check if the goal is actually still unsatisfied
            Character boxAtGoal = currentState.getBoxes().get(goalPos);
            if (boxAtGoal != null && boxAtGoal == goalType) continue; // already back
            
            // Find the nearest box of this type that can be pushed back
            Color goalColor = level.getBoxColor(goalType);
            if (goalColor == null) continue;
            
            int returnAgent = -1;
            for (int a = 0; a < level.getNumAgents(); a++) {
                if (goalColor.equals(level.getAgentColor(a))) {
                    returnAgent = a;
                    break;
                }
            }
            if (returnAgent < 0) continue;
            
            // Find the displaced box — nearest box of the right type not on its goal
            Position bestBoxPos = null;
            int bestDist = Integer.MAX_VALUE;
            for (Map.Entry<Position, Character> entry : currentState.getBoxes().entrySet()) {
                if (entry.getValue() != goalType) continue;
                Position bPos = entry.getKey();
                // Skip boxes that are already on a satisfied goal
                char bGoal = level.getBoxGoal(bPos.row, bPos.col);
                if (bGoal == entry.getValue()) continue;
                int dist = goalPos.manhattanDistance(bPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestBoxPos = bPos;
                }
            }
            if (bestBoxPos == null) continue;
            
            // Plan BSP to push box back to goal
            Set<Position> frozenForReturn = new HashSet<>(completedBoxGoals);
            frozenForReturn.removeAll(displacedGoals);
            frozenForReturn.remove(goalPos); // target must not be frozen
            List<Action> returnPath = boxSearchPlanner.searchForSubgoal(
                    returnAgent, bestBoxPos, goalPos, goalType, currentState, level, frozenForReturn);
            
            if (returnPath != null && !returnPath.isEmpty()) {
                for (Action action : returnPath) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            returnAgent, action, currentState, level, numAgents, false, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, returnAgent);
                    fullPlan.add(jointAction);
                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    globalTimeStep++;
                }
                
                // Verify the box actually reached the goal
                Character atGoal = currentState.getBoxes().get(goalPos);
                if (atGoal != null && atGoal == goalType) {
                    completedBoxGoals.add(goalPos);
                    displacedGoals.remove(goalPos);
                    returned++;
                    logVerbose("[PP] [RETURN] Returned " + goalType + " to " + goalPos 
                            + " (" + returnPath.size() + " steps)");
                }
            }
        }
        
        if (returned > 0) {
            logNormal("[PP] [RETURN] Returned " + returned + "/" 
                    + lastClearingDisplacedGoals.size() + " displaced goals");
            revalidateCompletedGoals(currentState, level);
        }
        lastClearingDisplacedGoals.clear();
        return currentState;
    }

    /**
     * Agent-trap detection: checks if committing to this subgoal would trap the agent.
     * 
     * IMPORTANT: In a pull-supporting domain, the agent can always pull same-color
     * boxes out of the way to create paths. This means boxes of the agent's color
     * are NOT permanent obstacles. Only treat walls and different-color/immovable
     * boxes as obstacles. Also, the agent can push/pull boxes it's adjacent to,
     * so we include cells reachable after moving a same-color box.
     * 
     * Pre-execution comparison: If the agent was ALREADY unable to reach remaining
     * tasks before executing this subgoal (e.g., blocked by cross-color boxes),
     * then filling the goal doesn't make things worse — don't flag as trapped.
     * This prevents false positives in dense levels like ZOOM where A-boxes
     * permanently separate map regions.
     */
    private boolean wouldTrapAgent(Subgoal completedSubgoal, State stateAfterExecution, 
            Level level, List<Subgoal> allSubgoals, State stateBeforeExecution) {
        int agentId = completedSubgoal.agentId;
        Position agentPosAfter = stateAfterExecution.getAgentPosition(agentId);
        Color agentColor = level.getAgentColor(agentId);
        
        // Collect remaining same-color box subgoals (excluding the one just completed)
        List<Subgoal> remainingSameColor = new ArrayList<>();
        for (Subgoal sg : allSubgoals) {
            if (sg == completedSubgoal) continue;
            if (sg.isAgentGoal) continue;
            if (completedBoxGoals.contains(sg.goalPos)) continue;
            // Same color check: this agent's tasks only
            Color sgColor = level.getBoxColor(sg.boxType);
            if (agentColor != null && agentColor.equals(sgColor)) {
                remainingSameColor.add(sg);
            }
        }
        
        // No remaining same-color tasks → agent has nothing left to do → not trapped
        if (remainingSameColor.isEmpty()) return false;
        
        // Pull-aware BFS: same-color boxes are NOT permanent obstacles.
        // Agent can pull them out of the way, so they don't block movement.
        Set<Position> reachableAfter = agentReachabilityBFS(agentPosAfter, stateAfterExecution, level, agentColor);
        
        // Check if agent can reach ANY candidate box for its remaining tasks (post-execution)
        boolean canReachAfter = false;
        for (Subgoal sg : remainingSameColor) {
            if (canReachAfter) break;
            for (Map.Entry<Position, Character> entry : stateAfterExecution.getBoxes().entrySet()) {
                if (entry.getValue() == sg.boxType) {
                    Position boxPos = entry.getKey();
                    for (Direction dir : Direction.values()) {
                        Position neighbor = boxPos.move(dir);
                        if (reachableAfter.contains(neighbor)) {
                            canReachAfter = true;
                            break;
                        }
                    }
                    if (canReachAfter) break;
                }
            }
        }
        
        if (canReachAfter) return false; // Can reach remaining tasks → not trapped
        
        // Agent can't reach remaining tasks AFTER execution.
        // But was the agent ALREADY unable to reach them BEFORE execution?
        // If yes → filling this goal doesn't make things worse → allow it.
        Position agentPosBefore = stateBeforeExecution.getAgentPosition(agentId);
        Set<Position> reachableBefore = agentReachabilityBFS(agentPosBefore, stateBeforeExecution, level, agentColor);
        
        boolean couldReachBefore = false;
        for (Subgoal sg : remainingSameColor) {
            if (couldReachBefore) break;
            for (Map.Entry<Position, Character> entry : stateBeforeExecution.getBoxes().entrySet()) {
                if (entry.getValue() == sg.boxType) {
                    Position boxPos = entry.getKey();
                    for (Direction dir : Direction.values()) {
                        Position neighbor = boxPos.move(dir);
                        if (reachableBefore.contains(neighbor)) {
                            couldReachBefore = true;
                            break;
                        }
                    }
                    if (couldReachBefore) break;
                }
            }
        }
        
        if (!couldReachBefore && remainingSameColor.size() >= 2) {
            // Agent was already cut off from multiple remaining tasks — filling this goal
            // doesn't trap it any more than it already was. Allow the goal.
            // Threshold ≥2: with only 1 remaining task, the trap rollback can accidentally
            // improve parallel scheduling (observed in ClosedAI: K→(6,10) agent 9).
            logVerbose("[PP] [TRAP-BYPASS] Agent " + agentId + " already couldn't reach remaining " 
                    + remainingSameColor.size() + " tasks before " + completedSubgoal.boxType 
                    + " -> " + completedSubgoal.goalPos + " — allowing");
            return false;
        }
        
        // Agent COULD reach tasks before but CAN'T after → truly trapped
        return true;
    }
    
    /**
     * BFS from a position to find all cells reachable by the agent.
     * Pull-aware: same-color boxes can be pulled out of the way, so they are
     * NOT treated as permanent obstacles. Only walls, immovable boxes, and
     * different-color boxes block the agent.
     */
    private Set<Position> agentReachabilityBFS(Position start, State state, Level level, Color agentColor) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        
        visited.add(start);
        queue.add(start);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                
                // Check if there's a box at this position
                Character boxAtNext = state.getBoxes().get(next);
                if (boxAtNext != null) {
                    // Same-color box: agent can pull it out of the way → passable
                    Color boxColor = level.getBoxColor(boxAtNext);
                    if (boxColor != null && boxColor.equals(agentColor)) {
                        // Box is same color — agent can interact (push/pull) to clear path
                        visited.add(next);
                        queue.add(next);
                        continue;
                    }
                    // Different-color or immovable box → obstacle
                    continue;
                }
                
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }
    
    // ====================== Connected Component Parallel Execution ======================
    
    /**
     * Computes connected components for agents. Agents in different components
     * are separated by walls/immovable boxes and cannot interact — their plans
     * can execute in parallel, reducing total plan steps from O(sum) to O(max).
     */
    private Map<Integer, Integer> computeAgentComponents(State state, Level level) {
        int numAgents = state.getNumAgents();
        if (numAgents <= 1) {
            Map<Integer, Integer> single = new HashMap<>();
            single.put(0, 0);
            return single;
        }
        
        // BFS reachability per agent (movable boxes are passable — agent can push/pull them)
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
                    if (!level.isWall(next) && !immovableBoxes.contains(next) && !area.contains(next)) {
                        area.add(next);
                        queue.add(next);
                    }
                }
            }
            reachable.put(a, area);
        }
        
        // Union-Find: agents whose reachable sets overlap are in the same component
        int[] parent = new int[numAgents];
        for (int i = 0; i < numAgents; i++) parent[i] = i;
        
        for (int a = 0; a < numAgents; a++) {
            for (int b = a + 1; b < numAgents; b++) {
                // Fast check: is agent b's position in agent a's reachable set?
                if (reachable.get(a).contains(state.getAgentPosition(b))) {
                    unionFind(parent, a, b);
                }
            }
        }
        
        Map<Integer, Integer> result = new HashMap<>();
        for (int a = 0; a < numAgents; a++) {
            result.put(a, findRoot(parent, a));
        }
        
        // Log independent groups
        Map<Integer, List<Integer>> groups = new HashMap<>();
        for (var entry : result.entrySet()) {
            groups.computeIfAbsent(entry.getValue(), k -> new ArrayList<>()).add(entry.getKey());
        }
        if (groups.size() > 1) {
            logNormal("[PP] Detected " + groups.size() + " independent agent groups: " + groups.values());
        }
        
        return result;
    }
    
    private int findRoot(int[] parent, int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]]; // path compression
            x = parent[x];
        }
        return x;
    }
    
    private void unionFind(int[] parent, int a, int b) {
        int pa = findRoot(parent, a);
        int pb = findRoot(parent, b);
        parent[pa] = pb;
    }
    
    /**
     * Plans subgoals for agents in independent connected components.
     * When the primary agent's subgoal is being executed, agents in other
     * components can execute their own subgoals in parallel via stored plans.
     * The existing addOtherAgentMoves() in PlanMerger merges them at each step.
     */
    private void planIndependentAgents(Subgoal primarySubgoal, List<Subgoal> allSubgoals,
            State state, Level level) {
        if (agentComponentId.size() <= 1) return;
        
        int primaryComponent = agentComponentId.getOrDefault(primarySubgoal.agentId, -1);
        Set<Integer> plannedComponents = new HashSet<>();
        plannedComponents.add(primaryComponent);
        
        for (Subgoal sg : allSubgoals) {
            if (sg == primarySubgoal) continue;
            int sgComponent = agentComponentId.getOrDefault(sg.agentId, -1);
            if (sgComponent == primaryComponent || plannedComponents.contains(sgComponent)) continue;
            
            // Skip if agent already has a stored plan being executed
            if (planMerger.hasPlanRemaining(sg.agentId)) {
                plannedComponents.add(sgComponent);
                continue;
            }
            
            // Check dependencies
            if (!areDependenciesMet(sg.goalPos, level)) continue;
            
            // Skip goals completed too many times (cycle detection)
            if (!sg.isAgentGoal) {
                int completions = goalCompletionCount.getOrDefault(sg.goalPos, 0);
                if (completions >= MAX_GOAL_COMPLETIONS) continue;
            }
            
            // Plan this subgoal
            List<Action> path = planSubgoal(sg, state, level, allSubgoals);
            if (path != null && !path.isEmpty()) {
                planMerger.storePlan(sg.agentId, path, globalTimeStep);
                storedPlanSubgoals.put(sg.agentId, sg);
                plannedComponents.add(sgComponent);
                logNormal("[PP] [PARALLEL] Stored plan for Agent " + sg.agentId 
                        + " (" + (sg.isAgentGoal ? "AgentGoal" : "Box " + sg.boxType) + " -> " + sg.goalPos
                        + ", " + path.size() + " steps) in component " + sgComponent);
            }
        }
    }
    
    /**
     * Checks if any agents executing via stored plans (parallel execution)
     * have completed their goals. Updates completion tracking accordingly.
     */
    private void checkStoredPlanCompletions(State state, Level level) {
        if (storedPlanSubgoals.isEmpty()) return;
        
        Iterator<Map.Entry<Integer, Subgoal>> it = storedPlanSubgoals.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Subgoal> entry = it.next();
            int agentId = entry.getKey();
            Subgoal sg = entry.getValue();
            
            // Check if plan is exhausted
            if (!planMerger.hasPlanRemaining(agentId)) {
                boolean reached = verifyGoalReached(sg, state, level);
                if (reached) {
                    if (!sg.isAgentGoal) {
                        completedBoxGoals.add(sg.goalPos);
                        int count = goalCompletionCount.getOrDefault(sg.goalPos, 0) + 1;
                        goalCompletionCount.put(sg.goalPos, count);
                        subgoalManager.invalidateHungarianCache();
                        cachedSubgoalOrder = null;
                    }
                    if (sg.isAgentGoal) {
                        completedAgentGoals.add(sg.goalPos);
                    }
                    logMinimal(getName() + ": [OK] " 
                            + (sg.isAgentGoal ? "Agent " + agentId : "Box " + sg.boxType)
                            + " -> " + sg.goalPos + " [PARALLEL]");
                }
                planMerger.invalidatePlan(agentId);
                it.remove();
            }
        }
    }

    /** Extract agent position path from action sequence. */
    private List<Position> extractAgentPath(int agentId, State startState, List<Action> actions, Level level) {
        List<Position> path = new ArrayList<>();
        State state = startState;
        path.add(state.getAgentPosition(agentId));
        
        for (Action action : actions) {
            if (action.type != Action.ActionType.NOOP && state.isApplicable(action, agentId, level)) {
                state = state.apply(action, agentId);
            }
            path.add(state.getAgentPosition(agentId));
        }
        return path;
    }
    
    /**
     * Park agent at a safe position after completing a subgoal.
     * 
     * Pukoban standard yielding: after an agent finishes its task, it must move out of
     * corridors and chokepoints so that subsequent agents can use those passages.
     * Uses PathAnalyzer.findParkingPosition which avoids:
     * - Corridors (positions with only 2 passable neighbors)
     * - Box goal positions
     * - Agent goal positions  
     * - Satisfied goal positions (frozen boxes)
     * - Other agent positions
     */
    private State parkAgentAfterSubgoal(int agentId, State state, Level level, 
            List<Action[]> fullPlan, int numAgents, List<Subgoal> pendingSubgoals) {
        Position agentPos = state.getAgentPosition(agentId);
        
        // Check if agent is already at a safe position:
        // Not on a goal, not in a corridor (2 passable neighbors = corridor)
        // TASK-AWARE FIX: Also check if agent is on the critical path of any pending subgoal
        boolean onBoxGoal = level.getBoxGoal(agentPos) != '\0';
        boolean onAgentGoal = level.getAgentGoal(agentPos.row, agentPos.col) >= 0;
        boolean inCorridor = pathAnalyzer.isInCorridor(agentPos, level);
        boolean onCriticalPath = isOnPendingSubgoalCriticalPath(agentPos, agentId, pendingSubgoals, state, level);
        
        if (!onBoxGoal && !onAgentGoal && !inCorridor && !onCriticalPath) {
            return state; // Already in a safe parking spot
        }
        
        logVerbose("[PP] Yielding agent " + agentId + " from " + agentPos
                + " (onBoxGoal=" + onBoxGoal + ", onAgentGoal=" + onAgentGoal 
                + ", inCorridor=" + inCorridor + ", onCriticalPath=" + onCriticalPath + ")");
        
        // Compute positions to avoid: all frozen/completed goal positions
        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);
        satisfiedGoals.addAll(completedBoxGoals);
        
        // Critical positions: other agents' current positions + pending subgoal paths
        Set<Position> criticalPositions = new HashSet<>();
        for (int i = 0; i < numAgents; i++) {
            if (i == agentId) continue;
            criticalPositions.add(state.getAgentPosition(i));
        }
        
        // Also add critical positions from pending subgoals so parking doesn't block them
        for (Subgoal sg : pendingSubgoals) {
            if (sg.isAgentGoal) {
                criticalPositions.add(sg.goalPos);
            } else {
                criticalPositions.add(sg.goalPos);
                // Task-Aware: Use globally valid box allocation for critical path checking
                Position boxPos = subgoalManager.findBestBoxForGoal(sg, state, level, pendingSubgoals, completedBoxGoals);
                if (boxPos != null) {
                    List<Position> boxPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                            boxPos, sg.goalPos, level);
                    if (boxPath != null) {
                        criticalPositions.addAll(boxPath);
                        // Add adjacent cells (agent push positions)
                        for (Position p : boxPath) {
                            for (Direction dir : Direction.values()) {
                                Position adj = p.move(dir);
                                if (!level.isWall(adj)) criticalPositions.add(adj);
                            }
                        }
                    }
                }
            }
        }
        
        // Find parking position using PathAnalyzer's existing logic
        Position parkingPos = pathAnalyzer.findParkingPosition(
                agentId, state, level, numAgents, criticalPositions, satisfiedGoals);
        
        if (parkingPos == null || parkingPos.equals(agentPos)) {
            return state; // No better parking position found
        }
        
        // Plan path to parking position (with push/pull fallback through same-color boxes)
        List<Action> parkPath = pathAnalyzer.planAgentPathWithFallback(
                agentId, parkingPos, state, level, numAgents, boxSearchPlanner);
        if (parkPath == null || parkPath.isEmpty()) {
            return state;
        }
        
        // Limit parking path length to avoid wasting too many steps
        int maxParkSteps = 10;
        if (parkPath.size() > maxParkSteps) {
            parkPath = parkPath.subList(0, maxParkSteps);
        }
        
        // Execute the parking path
        State tempState = state;
        for (Action moveAction : parkPath) {
            if (!tempState.isApplicable(moveAction, agentId, level)) break;
            Action[] jointAction = new Action[numAgents];
            for (int i = 0; i < numAgents; i++) {
                jointAction[i] = (i == agentId) ? moveAction : Action.noOp();
            }
            fullPlan.add(jointAction);
            tempState = tempState.apply(moveAction, agentId);
            globalTimeStep++;
        }
        
        logVerbose("[PP] Parked agent " + agentId + " at " + tempState.getAgentPosition(agentId)
                + " (was " + agentPos + ")");
        return tempState;
    }
    
    /**
     * Task-aware critical path check: determines if an agent's current position
     * lies on the critical path of any pending (not yet completed) subgoal.
     * 
     * This is the key improvement over position-based-only yielding: even if an
     * agent is in an open area (not on a goal, not in a corridor), it may still
     * block a future box-to-goal or agent-to-box path. By checking against all
     * remaining subgoals, we proactively move the agent before BSP even attempts
     * to plan around it.
     * 
     * @param agentPos       Current position of the agent
     * @param agentId        ID of the agent being checked
     * @param pendingSubgoals List of subgoals not yet completed
     * @param state          Current world state
     * @param level          Level definition
     * @return true if the agent is on the critical path of at least one pending subgoal
     */
    private boolean isOnPendingSubgoalCriticalPath(Position agentPos, int agentId,
            List<Subgoal> pendingSubgoals, State state, Level level) {
        if (pendingSubgoals == null || pendingSubgoals.isEmpty()) {
            return false;
        }
        
        for (Subgoal sg : pendingSubgoals) {
            if (sg.isAgentGoal) {
                // Agent goal: check if position is on agent's path to its goal
                Position otherAgentPos = state.getAgentPosition(sg.agentId);
                List<Position> path = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        otherAgentPos, sg.goalPos, level);
                if (path != null && path.contains(agentPos)) {
                    return true;
                }
            } else {
                // Box goal: check both agent-to-box path AND box-to-goal path
                Position boxPos = subgoalManager.findBestBoxForGoal(sg, state, level, completedBoxGoals);
                if (boxPos == null) continue;
                
                // Check box-to-goal path (and adjacent cells, since agent pushes from beside)
                List<Position> boxPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        boxPos, sg.goalPos, level);
                if (boxPath != null) {
                    for (Position p : boxPath) {
                        if (p.equals(agentPos)) {
                            return true;
                        }
                        for (Direction dir : Direction.values()) {
                            if (p.move(dir).equals(agentPos)) {
                                return true;
                            }
                        }
                    }
                }
                
                // Check agent-to-box path (the agent assigned to this subgoal must reach the box)
                Position sgAgentPos = state.getAgentPosition(sg.agentId);
                List<Position> agentToBox = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        sgAgentPos, boxPos, level);
                if (agentToBox != null && agentToBox.contains(agentPos)) {
                    return true;
                }
            }
        }
        return false;
    }

    /** Get direction from one position to adjacent position. */
    private Direction getDirection(Position from, Position to) {
        int dRow = to.row - from.row;
        int dCol = to.col - from.col;
        for (Direction dir : Direction.values()) {
            if (dir.dRow == dRow && dir.dCol == dCol) return dir;
        }
        return null;
    }
    
    /**
     * Plan path for a single subgoal.
     * 
     * Frozen = wall semantics: completedBoxGoals are treated as impassable walls.
     * Caller-managed retry sequence with progressive relaxation:
     *   Round 1: All completedBoxGoals frozen (wall) — ST-A* then 2D A*
     *   Round 2: Self-blockers unlocked (targeted) — ST-A* then 2D A*
     *   Round 3: No frozen at all (desperate last resort) — 2D A*
     * 
     * For agent goals: same wall semantics, 2-round retry.
     */
    private List<Action> planSubgoal(Subgoal subgoal, State state, Level level, List<Subgoal> allSubgoals) {
        // Frozen = all completed box goals treated as walls,
        // EXCEPT goals that were intentionally displaced by tryRecovery.
        // Displaced goals remain in completedBoxGoals for dependency satisfaction
        // but must not block BSP pathfinding (the box is physically gone).
        Set<Position> frozen = new HashSet<>(completedBoxGoals);
        frozen.removeAll(displacedGoals);
        
        if (subgoal.isAgentGoal) {
            // Round 1: Agent goal with frozen protection (don't push completed goals)
            List<Action> path = boxSearchPlanner.searchForAgentGoal(
                    subgoal.agentId, subgoal.goalPos, state, level, frozen);
            if (path != null) return path;
            
            // Round 2: Without frozen (allow pushing if path is blocked by completed goals)
            if (!frozen.isEmpty()) {
                path = boxSearchPlanner.searchForAgentGoal(
                        subgoal.agentId, subgoal.goalPos, state, level, Collections.emptySet());
            }
            return path;
        } else {
            // Box-retry mechanism: try BSP with the best box candidate. If ALL rounds
            // fail AND Hungarian was used for selection, invalidate the cache and retry
            // with greedy fallback (which may pick a different, BSP-friendlier box).
            //
            // WHY: Hungarian optimizes for SIMULTANEOUS fulfillment (global minimum total
            // distance), but PP executes SERIALLY. A globally-optimal box can be locally
            // terrible for BSP's limited search budget (far away, complex path). The retry
            // gives greedy Layer 2 a chance to pick a closer, easier box.
            boolean usedHungarian = subgoalManager.hasHungarianCache();
            Position boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
            
            for (int attempt = 0; attempt < 2; attempt++) {
                if (boxPos == null) {
                    if (attempt == 0 && usedHungarian) {
                        // Hungarian may have constrained the feasibility check too strictly.
                        // Retry with greedy-only.
                        logVerbose("[PP] findBestBoxForGoal returned null (attempt " + attempt 
                                + "), retrying without Hungarian");
                        subgoalManager.invalidateHungarianCache();
                        boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
                        continue;
                    }
                    logVerbose("[PP] findBestBoxForGoal returned null for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    return null;
                }
                
                logVerbose("[PP] Box " + subgoal.boxType + " at " + boxPos + " -> goal " + subgoal.goalPos
                        + " (agent " + subgoal.agentId + " at " + state.getAgentPosition(subgoal.agentId) 
                        + ", frozen=" + frozen + ", attempt=" + attempt + ")");
                
                // Dynamic BSP budget: scale search budget based on box-to-goal distance
                int dynamicBudget = computeDynamicBspBudget(boxPos, subgoal.goalPos);
                boxSearchPlanner.setMaxStatesOverride(dynamicBudget);
                
                try {
                // Round 1: ST-A* with all frozen as walls
                List<Action> path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                        subgoal.goalPos, subgoal.boxType, state, level, frozen,
                        reservationTable, globalTimeStep);
                
                // Round 1b: 2D A* with all frozen as walls
                if (path == null) {
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, frozen);
                }
                if (path != null) return path;
                logVerbose("[PP] Round 1 (frozen) failed for " + subgoal.boxType + " -> " + subgoal.goalPos);
                
                // Round 2a: Same-color self-blocking recovery — find frozen goals of the
                // same color that block the agent's path to the target box.
                if (!frozen.isEmpty()) {
                    Color agentColor = level.getAgentColor(subgoal.agentId);
                    Set<Position> selfBlockers = findPathBlockers(
                            subgoal.agentId, boxPos, state, level, frozen, agentColor, true);
                    
                    if (!selfBlockers.isEmpty()) {
                        Set<Position> relaxedFrozen = new HashSet<>(frozen);
                        relaxedFrozen.removeAll(selfBlockers);
                        
                        logVerbose("[PP] Targeted unlock (same-color): " + selfBlockers.size() 
                                + " blockers for " + subgoal.boxType + " -> " + subgoal.goalPos);
                        
                        path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen,
                                reservationTable, globalTimeStep);
                        if (path == null) {
                            path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                    subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen);
                        }
                        if (path != null) return path;
                    }
                }
                
                // Round 2b: Cross-color blocking recovery — find ANY frozen goal (regardless
                // of color) that blocks the path. This handles multi-agent scenarios where
                // e.g. a blue box on its goal blocks a red agent's path (MADS-level pattern).
                // More permissive than Round 2a but still targeted (not blanket removal).
                if (!frozen.isEmpty()) {
                    Set<Position> crossColorBlockers = findPathBlockers(
                            subgoal.agentId, boxPos, state, level, frozen, null, false);
                    
                    if (!crossColorBlockers.isEmpty()) {
                        Set<Position> relaxedFrozen = new HashSet<>(frozen);
                        relaxedFrozen.removeAll(crossColorBlockers);
                        
                        logVerbose("[PP] Targeted unlock (cross-color): " + crossColorBlockers.size() 
                                + " blockers for " + subgoal.boxType + " -> " + subgoal.goalPos);
                        
                        path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen,
                                reservationTable, globalTimeStep);
                        if (path == null) {
                            path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                    subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen);
                        }
                        if (path != null) return path;
                    }
                }
                
                // Round 3: No frozen at all (desperate — allows disturbing any completed goal)
                if (!frozen.isEmpty()) {
                    logVerbose("[PP] Round 3 (no frozen) for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, Collections.emptySet());
                    if (path != null) return path;
                }
                
                // Round 4: Weighted A* escalation — use w=3 for faster (suboptimal) search
                // This trades solution quality for search speed when standard A* exhausts budget
                if (boxSearchPlanner.getWeight() < 2.0) {
                    double savedWeight = boxSearchPlanner.getWeight();
                    boxSearchPlanner.setWeight(3.0);
                    logVerbose("[PP] Round 4 (weighted A* w=3) for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, frozen);
                    if (path == null && !frozen.isEmpty()) {
                        path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                subgoal.goalPos, subgoal.boxType, state, level, Collections.emptySet());
                    }
                    boxSearchPlanner.setWeight(savedWeight);
                    if (path != null) return path;
                }
                
                // All BSP rounds failed for this box. If Hungarian was used on the first
                // attempt, the globally-optimal pick may be BSP-hard for serial execution.
                // Invalidate cache and retry — greedy Layer 2 may pick a different box.
                if (attempt == 0 && usedHungarian) {
                    Position failedBox = boxPos;
                    subgoalManager.invalidateHungarianCache();
                    boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
                    // If greedy picks the SAME box, no point retrying BSP rounds
                    if (boxPos != null && boxPos.equals(failedBox)) {
                        logVerbose("[PP] Greedy fallback picked same box " + boxPos + " — no retry");
                        return null;
                    }
                    if (boxPos != null) {
                        logVerbose("[PP] All rounds failed for Hungarian box " + failedBox 
                                + " — retrying with greedy pick " + boxPos);
                    }
                    continue; // retry BSP rounds with new box
                }
                
                logVerbose("[PP] All rounds exhausted for " + subgoal.boxType + " -> " + subgoal.goalPos);
                return null;
                } finally {
                    boxSearchPlanner.clearMaxStatesOverride();
                }
            }
            return null; // should not reach here, but safety
        }
    }
    
    /**
     * Finds frozen goals that block the agent's path to the target box.
     * 
     * Two modes:
     * - sameColorOnly=true: Only considers frozen goals whose box color matches agentColor.
     *   These are "self-blockers" — the agent filled them and now can't pass through.
     * - sameColorOnly=false: Considers ALL frozen goals regardless of color.
     *   Handles cross-color blocking (e.g., blue box on its goal blocks red agent's path).
     *   This targeted approach avoids falling through to Round 3's blanket frozen removal.
     * 
     * Algorithm: BFS from agent position to find reachable cells. If target box is
     * unreachable, identify frozen goals on the boundary between reachable and unreachable
     * regions. Verify that removing candidates actually unblocks the path.
     * 
     * @param agentId       The agent that needs to reach the target box
     * @param targetBox     Position of the box to be moved
     * @param state         Current world state
     * @param level         Level definition
     * @param hardFrozen    Set of positions treated as frozen (wall-like)
     * @param agentColor    Color of the agent (used when sameColorOnly=true; nullable when false)
     * @param sameColorOnly If true, only return blockers matching agentColor
     * @return Set of frozen positions whose unlock would unblock the agent's path
     */
    private Set<Position> findPathBlockers(int agentId, Position targetBox, State state,
            Level level, Set<Position> hardFrozen, Color agentColor, boolean sameColorOnly) {
        Set<Position> blockers = new HashSet<>();
        if (sameColorOnly && agentColor == null) return blockers;
        
        Position agentPos = state.getAgentPosition(agentId);
        
        // BFS from agent position to find reachable cells (agent walk, ignoring box pushability)
        Set<Position> reachable = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        reachable.add(agentPos);
        queue.add(agentPos);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (reachable.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (state.getBoxes().containsKey(next)) continue; // boxes block walking
                reachable.add(next);
                queue.add(next);
            }
        }
        
        // Check if agent can reach any neighbor of the target box (to push/pull)
        boolean canReachBox = false;
        for (Direction dir : Direction.values()) {
            if (reachable.contains(targetBox.move(dir))) {
                canReachBox = true;
                break;
            }
        }
        
        if (canReachBox) return blockers; // agent CAN reach box, no blocking
        
        // Agent can't reach box. Find frozen goals on the boundary of reachable region.
        Set<Position> candidates = new HashSet<>();
        for (Position frozen : hardFrozen) {
            char frozenBoxType = level.getBoxGoal(frozen.row, frozen.col);
            if (frozenBoxType == '\0') continue;
            
            // Color filter: in same-color mode, skip different-color frozen goals
            if (sameColorOnly) {
                Color frozenColor = level.getBoxColor(frozenBoxType);
                if (!agentColor.equals(frozenColor)) continue;
            }
            
            // Check if this frozen goal has a box and is adjacent to the reachable region
            Character boxAtFrozen = state.getBoxes().get(frozen);
            if (boxAtFrozen == null) continue; // no box here, not blocking
            
            boolean isOnBoundary = false;
            for (Direction dir : Direction.values()) {
                if (reachable.contains(frozen.move(dir))) {
                    isOnBoundary = true;
                    break;
                }
            }
            if (isOnBoundary) {
                candidates.add(frozen);
            }
        }
        
        if (candidates.isEmpty()) return blockers;
        
        // VERIFICATION: re-run BFS excluding candidate blockers to confirm
        // that removing them actually makes the target box reachable.
        // This prevents false positives where other obstacles are the real cause.
        Set<Position> verifyReachable = new HashSet<>();
        Queue<Position> verifyQueue = new LinkedList<>();
        verifyReachable.add(agentPos);
        verifyQueue.add(agentPos);
        
        while (!verifyQueue.isEmpty()) {
            Position current = verifyQueue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (verifyReachable.contains(next)) continue;
                if (level.isWall(next)) continue;
                // Allow walking through candidate blocker positions
                if (state.getBoxes().containsKey(next) && !candidates.contains(next)) continue;
                verifyReachable.add(next);
                verifyQueue.add(next);
            }
        }
        
        boolean canReachAfterRelax = false;
        for (Direction dir : Direction.values()) {
            if (verifyReachable.contains(targetBox.move(dir))) {
                canReachAfterRelax = true;
                break;
            }
        }
        
        // Only return blockers if removing them actually unblocks the path
        if (canReachAfterRelax) {
            blockers.addAll(candidates);
        }
        
        return blockers;
    }
    
    /** Verify that a subgoal was actually reached. */
    private boolean verifyGoalReached(Subgoal subgoal, State state, Level level) {
        if (subgoal.isAgentGoal) {
            return state.getAgentPosition(subgoal.agentId).equals(subgoal.goalPos);
        } else {
            return state.getBoxAt(subgoal.goalPos) == subgoal.boxType;
        }
    }
    
    /** Revalidate completed goals - remove any that were disturbed (e.g. by soft-unlock). */
    private void revalidateCompletedGoals(State state, Level level) {
        boolean anyRemoved = false;
        Iterator<Position> it = completedBoxGoals.iterator();
        while (it.hasNext()) {
            Position goalPos = it.next();
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') {
                it.remove();
                anyRemoved = true;
                continue;
            }
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox == null || actualBox != goalType) {
                logVerbose("[PP] Goal at " + goalPos + " disturbed (type " + goalType + "), removing from frozen set");
                it.remove();
                anyRemoved = true;
                // Force subgoal list refresh so this disturbed goal gets re-added
                cachedSubgoalOrder = null;
            }
        }
        // Invalidate Hungarian cache when goals are removed — the optimal assignment
        // changes as completed goals shrink (different boxes become available).
        if (anyRemoved) {
            subgoalManager.invalidateHungarianCache();
        }
    }
    
    /**
     * Detects which completed box goals were regressed (disturbed) in the given state.
     * 
     * This is a READ-ONLY check — it does not modify completedBoxGoals.
     * Used as a pre-commit validation: if a planned path caused regression,
     * the caller should rollback rather than accept the regression.
     * 
     * Difference from revalidateCompletedGoals: that method is a post-hoc cleanup
     * that accepts regressions and removes them from the frozen set. This method
     * is a pre-commit guard that PREVENTS regressions from being accepted.
     * 
     * @param state The state to check (typically after executing a candidate path)
     * @param level Level definition
     * @return List of goal positions that were completed but are now unsatisfied
     */
    private List<Position> detectRegressedGoals(State state, Level level) {
        List<Position> regressed = new ArrayList<>();
        for (Position goalPos : completedBoxGoals) {
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') continue;
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox == null || actualBox != goalType) {
                regressed.add(goalPos);
            }
        }
        return regressed;
    }

    /** Try CBS fallback when stuck with cyclic dependencies. */
    private List<Action[]> tryCBSFallback(State currentState, Level level, State initialState,
            List<Action[]> fullPlan, long startTime, int numAgents) {
        
        DependencyAnalyzer.AnalysisResult analysis = DependencyAnalyzer.analyze(currentState, level);
        
        if (analysis.hasCycle) {
            logVerbose("[PP] Cyclic dependency detected, trying displacement...");
            
            if (displacementAttempts < MAX_DISPLACEMENT_ATTEMPTS) {
                displacementAttempts++;
                List<Action[]> displacementPlan = new ArrayList<>();
                boolean success = deadlockBreaker.attemptCycleBreaking(
                    displacementPlan, currentState, level, numAgents,
                    analysis.cycles.isEmpty() ? new ArrayList<>() : analysis.cycles.get(0),
                    pathAnalyzer, conflictResolver);
                
                if (success && !displacementPlan.isEmpty()) {
                    logVerbose("[PP] Displacement succeeded with " + displacementPlan.size() + " steps");
                    fullPlan.addAll(displacementPlan);
                    return null; // Continue with PP
                }
            }
            
            // Try CBS as last resort
            logVerbose("[PP] Trying CBS fallback...");
            CBSStrategy cbs = new CBSStrategy(heuristic, config);
            cbs.setTimeout(timeoutMs - (System.currentTimeMillis() - startTime));
            List<Action[]> cbsPlan = cbs.search(currentState, level);
            
            if (cbsPlan != null && !cbsPlan.isEmpty()) {
                logMinimal("[PP] CBS solved with " + cbsPlan.size() + " steps");
                fullPlan.addAll(cbsPlan);
                return fullPlan;
            }
        }
        return null;
    }

    /** Try recovery mechanisms when stuck. */
    private boolean tryRecovery(List<Subgoal> subgoals, List<Action[]> fullPlan,
            State currentState, Level level, int numAgents, State initialState) {
        
        if (subgoals.isEmpty()) return false;
        Subgoal blockedGoal = subgoals.get(0);
        
        // Strategy 1: Try clearing blocking agents
        Set<Position> criticalPositions = new HashSet<>();
        criticalPositions.add(blockedGoal.goalPos);
        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(currentState, level);
        
        if (blockedGoal.isAgentGoal) {
            // Agent goal: find critical positions on agent-to-goal path
            criticalPositions.addAll(pathAnalyzer.findCriticalPositionsForAgentGoal(
                currentState, level, blockedGoal.agentId, blockedGoal.goalPos, satisfiedGoals));
        } else {
            // Box goal: find critical positions on BOTH agent-to-box AND box-to-goal paths
            Position boxPos = subgoalManager.findBestBoxForGoal(blockedGoal, currentState, level, completedBoxGoals);
            if (boxPos != null) {
                criticalPositions.addAll(pathAnalyzer.findCriticalPositions(
                    currentState, level, blockedGoal.agentId, blockedGoal.goalPos, boxPos, satisfiedGoals));
            } else {
                criticalPositions.addAll(pathAnalyzer.findCriticalPositionsForAgentGoal(
                    currentState, level, blockedGoal.agentId, blockedGoal.goalPos, satisfiedGoals));
            }
        }
        
        int planSizeBefore = fullPlan.size();
        AgentCoordinator.ClearingResult result = agentCoordinator.tryIdleAgentClearingWithResult(
            fullPlan, currentState, level, numAgents, blockedGoal.agentId, criticalPositions,
            pathAnalyzer, conflictResolver);
        
        if (result.success) {
            logVerbose("[PP] Cleared blocking agent " + result.clearedAgentId);
            return true;
        }
        
        // Strategy 2: Try box displacement
        // MAPF FIX: Pass explicit subgoals to DeadlockResolver so it knows exactly what the targets are
        // (Is it an Agent Goal? Is it a Box Goal? Which box?)
        // Helps avoid identifying the target box itself as an obstacle.
        List<DeadlockResolver.BlockingInfo> blockingInfos = 
            deadlockResolver.analyzeBlocking(currentState, level, subgoals, immovableBoxes);
        
        if (!blockingInfos.isEmpty()) {

            DeadlockResolver.DisplacementPlan displacement = 
                deadlockResolver.createDisplacementPlan(blockingInfos, currentState, level, displacementHistory);
            
            if (displacement != null) {
                List<Action> displacePath;
                boolean isAgentMove = displacement.isAgentDisplacement;
                
                if (isAgentMove) {
                    displacePath = boxSearchPlanner.searchForAgentGoal(
                        displacement.agentId, displacement.tempPosition, currentState, level);
                } else {
                    displacePath = boxSearchPlanner.planBoxDisplacement(
                        displacement.agentId, displacement.boxPosition,
                        displacement.tempPosition, displacement.boxType, currentState, level);
                }
                
                if (displacePath != null && !displacePath.isEmpty()) {
                    String historyKey = isAgentMove ? 
                        ("Agent" + displacement.agentId + "@" + displacement.boxPosition) : 
                        (displacement.boxType + "@" + displacement.boxPosition);
                        
                    displacementHistory.add(historyKey);
                    
                    State tempState = currentState;
                    for (Action action : displacePath) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                            displacement.agentId, action, tempState, level, numAgents, isAgentMove, completedBoxGoals);
                        jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, displacement.agentId);
                        fullPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    }
                    
                    // Displacement is INTENTIONAL disturbance of completed goals.
                    // Track displaced goals so planSubgoal excludes them from frozen
                    // and the regression guard in tryExecuteSubgoals ignores them.
                    if (!completedBoxGoals.isEmpty()) {
                        List<Position> regressed = detectRegressedGoals(tempState, level);
                        if (!regressed.isEmpty()) {
                            displacedGoals.addAll(regressed);
                            logVerbose("[PP] Displacement displaced " + regressed.size()
                                    + " completed goal(s): " + regressed + " (deferred for re-planning)");
                        }
                    }
                    return true;
                }
            }
        }
        
        // Strategy 3: Random reorder and retry
        Collections.shuffle(subgoals, random);
        for (Subgoal sg : subgoals) {
            List<Action> path = planSubgoal(sg, currentState, level, subgoals);
            if (path != null && !path.isEmpty()) {
                State tempState = currentState;
                for (Action action : path) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                        sg.agentId, action, tempState, level, numAgents, sg.isAgentGoal, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, sg.agentId);
                    fullPlan.add(jointAction);
                    tempState = applyJointAction(jointAction, tempState, level, numAgents);
                }
                if (verifyGoalReached(sg, tempState, level)) {
                    if (!sg.isAgentGoal) {
                        completedBoxGoals.add(sg.goalPos);
                        subgoalManager.invalidateHungarianCache();
                    }
                    return true;
                }
            }
        }
        
        return false;
    }

    /** Recompute state from initial + all actions. Cached for incremental performance. */
    private State lastComputedState = null;
    private int lastComputedPlanSize = 0;
    
    private State recomputeState(State initial, List<Action[]> plan, Level level, int numAgents) {
        // Incremental: only replay from where we left off
        if (lastComputedState != null && lastComputedPlanSize <= plan.size()) {
            State state = lastComputedState;
            for (int i = lastComputedPlanSize; i < plan.size(); i++) {
                state = applyJointAction(plan.get(i), state, level, numAgents);
            }
            lastComputedState = state;
            lastComputedPlanSize = plan.size();
            return state;
        }
        
        // Full recompute (first call or if plan was modified)
        State state = initial;
        for (Action[] jointAction : plan) {
            state = applyJointAction(jointAction, state, level, numAgents);
        }
        lastComputedState = state;
        lastComputedPlanSize = plan.size();
        return state;
    }

    /** Apply joint action to state (simultaneous per CLAUDE.md). */
    private State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        return state.applyJointAction(jointAction, level);
    }

    /** Subgoal: move a box or agent to a goal position. */
    public static class Subgoal {
        public final int agentId;
        public final char boxType;
        public final Position goalPos;
        public final boolean isAgentGoal;

        public Subgoal(int agentId, char boxType, Position goalPos, boolean isAgentGoal) {
            this.agentId = agentId;
            this.boxType = boxType;
            this.goalPos = goalPos;
            this.isAgentGoal = isAgentGoal;
        }

        public Subgoal(int agentId, char boxType, Position goalPos) {
            this(agentId, boxType, goalPos, false);
        }
    }
}
