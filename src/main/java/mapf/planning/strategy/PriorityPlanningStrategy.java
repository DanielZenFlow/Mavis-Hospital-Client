package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
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
    
    /** Tracks agent goals that have been completed at least once, to detect phantom progress. */
    private Set<Position> completedAgentGoals = new HashSet<>();
    
    /** Pre-computed goal execution order from LevelAnalyzer (optional). */
    private List<Position> precomputedGoalOrder = null;
    
    /** Goal dependency graph from LevelAnalyzer: goal → set of goals it depends on. */
    private Map<Position, Set<Position>> goalDependsOn = Collections.emptyMap();
    
    /** Immovable boxes (treated as walls in pathfinding). */
    private Set<Position> immovableBoxes = Collections.emptySet();
    
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

        // Reset for new search
        displacementHistory.clear();
        displacementAttempts = 0;
        completedBoxGoals.clear();
        completedAgentGoals.clear();
        reservationTable.clear();
        globalTimeStep = 0;
        lastComputedState = null;
        lastComputedPlanSize = 0;

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
                } else {
                    stuckCount++;
                    logVerbose(getName() + ": Phantom progress (re-solved agent goal), stuckCount=" + stuckCount);
                }
            } else {
                stuckCount++;
                
                // Try recovery mechanisms when stuck
                if (stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING) {
                    boolean recovered = tryRecovery(unsatisfied, fullPlan, currentState, level, numAgents, initialState);
                    if (recovered) {
                        currentState = recomputeState(initialState, fullPlan, level, numAgents);
                        stuckCount = 0;
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
        if (remaining.isEmpty() && !currentState.isGoalState(level)) {
            logVerbose("[PP] Phase switch: refreshing subgoal list");
            computeAndCacheSubgoals(currentState, level);
            remaining = filterUnsatisfiedSubgoals(cachedSubgoalOrder, currentState, level);
        }
        
        return remaining;
    }
    
    private void computeAndCacheSubgoals(State state, Level level) {
        // Hungarian optimal box-to-goal assignment: only when NO serialization dependencies.
        // When goalDependsOn is non-empty, goals must be filled in strict topological order
        // (e.g., dead-end corridors: inner-first). Hungarian's static global assignment
        // conflicts with this dynamic sequential execution — it assumes all goals are
        // simultaneously accessible, which is false in dead-end topologies.
        // In dependency-free levels, Hungarian minimizes total transport distance.
        if (goalDependsOn.isEmpty()) {
            subgoalManager.computeHungarianAssignment(state, level, completedBoxGoals);
        } else {
            subgoalManager.invalidateHungarianCache();
            logNormal("[PP] Goal dependencies detected (" + goalDependsOn.size() 
                    + " deps) — using greedy per-step assignment (Hungarian disabled)");
        }
        
        cachedSubgoalOrder = subgoalManager.getUnsatisfiedSubgoals(state, level, completedBoxGoals);
        sortSubgoals(cachedSubgoalOrder, state, level);
        logGoalOrder(cachedSubgoalOrder);
    }
    
    private List<Subgoal> filterUnsatisfiedSubgoals(List<Subgoal> source, State currentState, Level level) {
        List<Subgoal> strictRemaining = new ArrayList<>();
        List<Subgoal> allUnsatisfied = new ArrayList<>();
        
        for (Subgoal sg : source) {
            // Basic satisfaction check
            if (completedBoxGoals.contains(sg.goalPos)) continue;

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
                    logNormal("[PP] Sorted subgoals in REVERSE topological order");
                } else {
                    sortByDifficulty(subgoals, state, level);
                }
                break;
                
            case DISTANCE_GREEDY:
                // Sort by estimated difficulty: easiest (nearest) first
                sortByDifficulty(subgoals, state, level);
                logNormal("[PP] Sorted subgoals by DISTANCE_GREEDY (nearest first)");
                break;
                
            case RANDOM:
                Collections.shuffle(subgoals, random);
                logNormal("[PP] Sorted subgoals in RANDOM order");
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
    
    /** Log the goal execution order. Always printed at MINIMAL level — the canonical task plan output. */
    private void logGoalOrder(List<Subgoal> subgoals) {
        if (!SearchConfig.isMinimal()) return;
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
                if (!depMet) { skippedByDeps++; continue; }
            }

            // Task-Aware: Pass the full list of subgoals for global allocation checking
            List<Action> path = planSubgoal(subgoal, currentState, level, subgoals);
            
            if (path != null && !path.isEmpty()) {
                
                // Snapshot plan size for potential rollback (agent-trap detection)
                int planSizeBefore = fullPlan.size();
                
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
                }
                
                // Verify goal was reached
                boolean reached = verifyGoalReached(subgoal, tempState, level);
                if (reached) {
                    // Pukoban agent-trap detection: after filling a box goal, check if the 
                    // agent is trapped in a dead-end disconnected from remaining tasks.
                    // This is a POST-PLAN validation (not static analysis) because it depends
                    // on the actual agent position and world state after execution.
                    if (!subgoal.isAgentGoal && wouldTrapAgent(subgoal, tempState, level, subgoals)) {
                        // Agent would be trapped — skip this subgoal, let PP try alternatives
                        logNormal(getName() + ": [TRAP] Skipping " + subgoal.boxType + " -> " + subgoal.goalPos
                                + " — agent " + subgoal.agentId + " would be trapped after filling");
                        // Rollback: remove the executed actions from fullPlan
                        while (fullPlan.size() > planSizeBefore) {
                            fullPlan.remove(fullPlan.size() - 1);
                            globalTimeStep--;
                        }
                        continue; // try next subgoal in priority order
                    }
                    
                    // Mark box goal as completed
                    if (!subgoal.isAgentGoal) {
                        completedBoxGoals.add(subgoal.goalPos);
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
                    
                    logMinimal(getName() + ": [OK] " + 
                        (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) +
                        " -> " + subgoal.goalPos + " (" + path.size() + " steps)" +
                        (lastProgressWasPhantom ? " [PHANTOM]" : ""));
                    return true;
                }
            }
        }
        return false;
    }
    
    /**
     * Pukoban agent-trap detection: checks if committing to this subgoal would trap the agent.
     * 
     * After filling goalPos, the agent ends up at some position. We BFS from that position
     * to check if the agent can still reach any candidate box for its remaining same-color tasks.
     * If not, the agent would be stranded in a dead-end — a classic Pukoban failure mode.
     * 
     * This is the correct architectural location for this check (post-plan, not static analysis)
     * because it operates on the actual state after execution, avoiding false positives from
     * pre-existing room disconnections that static pairwise analysis cannot distinguish.
     */
    private boolean wouldTrapAgent(Subgoal completedSubgoal, State stateAfterExecution, Level level, List<Subgoal> allSubgoals) {
        int agentId = completedSubgoal.agentId;
        Position agentPos = stateAfterExecution.getAgentPosition(agentId);
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
        
        // BFS from agent's final position to find reachable cells
        // Obstacles: walls + all boxes in current state (agent navigates around them)
        Set<Position> reachable = agentReachabilityBFS(agentPos, stateAfterExecution, level);
        
        // Check if agent can reach ANY candidate box for its remaining tasks
        for (Subgoal sg : remainingSameColor) {
            for (Map.Entry<Position, Character> entry : stateAfterExecution.getBoxes().entrySet()) {
                if (entry.getValue() == sg.boxType) {
                    // Agent needs to reach a neighbor of the box (to push/pull it)
                    Position boxPos = entry.getKey();
                    for (Direction dir : Direction.values()) {
                        Position neighbor = boxPos.move(dir);
                        if (reachable.contains(neighbor)) {
                            return false; // Can reach at least one box → not trapped
                        }
                    }
                }
            }
        }
        
        // Agent cannot reach any box for remaining tasks → trapped
        return true;
    }
    
    /**
     * BFS from a position to find all cells reachable by the agent.
     * Agent can walk through empty cells but not walls or boxes.
     */
    private Set<Position> agentReachabilityBFS(Position start, State state, Level level) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        Set<Position> boxPositions = state.getBoxes().keySet();
        
        visited.add(start);
        queue.add(start);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next) && !boxPositions.contains(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }
        return visited;
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
        
        // Plan path to parking position
        List<Action> parkPath = pathAnalyzer.planAgentPath(agentId, parkingPos, state, level, numAgents);
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
        // Frozen = all completed box goals treated as walls
        Set<Position> frozen = new HashSet<>(completedBoxGoals);
        
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
            // Task-Aware Allocation: Pass all remaining subgoals to ensure global feasibility
            Position boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
            if (boxPos == null) {
                return null;
            }
            
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
            
            // Round 2: Self-blocking recovery — find specific frozen goals that block
            // the agent's path to the target box and unlock only those.
            if (!frozen.isEmpty()) {
                Color agentColor = level.getAgentColor(subgoal.agentId);
                Set<Position> selfBlockers = findSameColorBlockers(
                        subgoal.agentId, boxPos, state, level, frozen, agentColor);
                
                if (!selfBlockers.isEmpty()) {
                    Set<Position> relaxedFrozen = new HashSet<>(frozen);
                    relaxedFrozen.removeAll(selfBlockers);
                    
                    logNormal("[PP] Targeted unlock: " + selfBlockers.size() 
                            + " self-blockers for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    
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
                logNormal("[PP] Last resort: no frozen for " + subgoal.boxType + " -> " + subgoal.goalPos);
                path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                        subgoal.goalPos, subgoal.boxType, state, level, Collections.emptySet());
            }
            
            return path;
        }
    }
    
    /**
     * Finds hard-frozen goals of the same color that block the agent's path to the target box.
     * These are "self-blockers": the same-color agent filled them and now needs to pass through.
     * 
     * Algorithm: BFS from agent position, treating walls + boxes (except pathable ones) as obstacles.
     * If the target box is unreachable, check which hard-frozen goals lie on the boundary between
     * reachable and unreachable regions.
     */
    private Set<Position> findSameColorBlockers(int agentId, Position targetBox, State state,
            Level level, Set<Position> hardFrozen, Color agentColor) {
        Set<Position> blockers = new HashSet<>();
        if (agentColor == null) return blockers;
        
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
        
        if (canReachBox) return blockers; // agent CAN reach box, no self-blocking
        
        // Agent can't reach box. Find hard-frozen goals on the boundary of reachable region
        // that are the same color and might be blocking the path.
        Set<Position> candidates = new HashSet<>();
        for (Position frozen : hardFrozen) {
            char frozenBoxType = level.getBoxGoal(frozen.row, frozen.col);
            if (frozenBoxType == '\0') continue;
            Color frozenColor = level.getBoxColor(frozenBoxType);
            if (!agentColor.equals(frozenColor)) continue; // different color, skip
            
            // Check if this frozen goal is adjacent to the reachable region
            // (i.e., the frozen box at this position is on the boundary)
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
        Iterator<Position> it = completedBoxGoals.iterator();
        while (it.hasNext()) {
            Position goalPos = it.next();
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') {
                it.remove();
                continue;
            }
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox == null || actualBox != goalType) {
                logVerbose("[PP] Goal at " + goalPos + " disturbed (type " + goalType + "), removing from frozen set");
                it.remove();
                // Force subgoal list refresh so this disturbed goal gets re-added
                cachedSubgoalOrder = null;
            }
        }
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
