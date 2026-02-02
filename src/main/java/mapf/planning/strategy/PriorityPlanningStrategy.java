package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
import mapf.planning.analysis.DependencyAnalyzer;
import mapf.planning.cbs.CBSStrategy;
import mapf.planning.coordination.AgentYieldingManager;
import mapf.planning.coordination.ConflictDetector;
import mapf.planning.coordination.DeadlockResolver;
import mapf.planning.coordination.SafeZoneCalculator;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;
import java.util.*;

/**
 * Priority-based planning strategy with subgoal decomposition.
 * Uses priority re-ordering, true distance heuristic, and specialized helper classes.
 */
public class PriorityPlanningStrategy implements SearchStrategy {

    private final Heuristic heuristic;
    private final SearchConfig config;
    private long timeoutMs;
    private int maxStates;
    private final Random random = new Random(SearchConfig.RANDOM_SEED);

    // Helper classes
    private final SubgoalManager subgoalManager;
    private final SubgoalSearcher subgoalSearcher;
    private final TopologicalAnalyzer topologicalAnalyzer;
    private final ConflictResolver conflictResolver;
    private final ImmovableBoxDetector immovableDetector;
    
    // New helper classes (Phase 2)
    private final BoxSearchPlanner boxSearchPlanner;
    private final GreedyPlanner greedyPlanner;
    private final PlanMerger planMerger;
    private final PathAnalyzer pathAnalyzer;
    private final AgentCoordinator agentCoordinator;
    private final DeadlockBreaker deadlockBreaker;
    private final GoalChecker goalChecker;
    private final ConflictDetector conflictDetector;
    
    // Legacy helpers (to be deprecated)
    private final AgentYieldingManager yieldingManager = new AgentYieldingManager();
    private final DeadlockResolver deadlockResolver = new DeadlockResolver();
    private final SafeZoneCalculator safeZoneCalculator = new SafeZoneCalculator();

    /** Tracks displaced boxes to avoid infinite loops. */
    private Set<String> displacementHistory = new HashSet<>();
    
    /** Counts displacement attempts; forces CBS after MAX_DISPLACEMENT_ATTEMPTS. */
    private int displacementAttempts = 0;
    private static final int MAX_DISPLACEMENT_ATTEMPTS = 3;
    
    /** Pre-computed goal execution order from LevelAnalyzer (optional). */
    private List<Position> precomputedGoalOrder = null;

    // Logging helpers
    private void logMinimal(String msg) {
        if (SearchConfig.isMinimal())
            System.err.println(msg);
    }

    private void logNormal(String msg) {
        if (SearchConfig.isNormal())
            System.err.println(msg);
    }

    private void logVerbose(String msg) {
        if (SearchConfig.isVerbose())
            System.err.println(msg);
    }

    public PriorityPlanningStrategy(Heuristic heuristic, SearchConfig config) {
        this.heuristic = heuristic;
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
        
        // Initialize refactored helpers
        this.subgoalManager = new SubgoalManager(heuristic);
        this.subgoalSearcher = new SubgoalSearcher(heuristic);
        this.topologicalAnalyzer = new TopologicalAnalyzer();
        this.conflictResolver = new ConflictResolver();
        this.immovableDetector = new ImmovableBoxDetector();
        
        // Initialize Phase 2 helpers
        this.boxSearchPlanner = new BoxSearchPlanner(heuristic);
        this.greedyPlanner = new GreedyPlanner();
        this.planMerger = new PlanMerger();
        this.pathAnalyzer = new PathAnalyzer();
        this.agentCoordinator = new AgentCoordinator();
        this.deadlockBreaker = new DeadlockBreaker();
        this.goalChecker = new GoalChecker();
        this.conflictDetector = new ConflictDetector();
    }

    @Override
    public String getName() {
        return "Priority Planning";
    }

    @Override
    public void setTimeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }

    @Override
    public void setMaxStates(int maxStates) {
        this.maxStates = maxStates;
    }
    
    /**
     * Sets pre-computed goal execution order from LevelAnalyzer.
     * When set, goals will be executed in this order instead of dynamic sorting.
     */
    public void setGoalExecutionOrder(List<Position> order) {
        this.precomputedGoalOrder = order;
        if (order != null && SearchConfig.isNormal()) {
            System.err.println("[PriorityPlanning] Using pre-computed goal order: " + order.size() + " goals");
        }
    }

    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        int numAgents = initialState.getNumAgents();

        logMinimal(getName() + ": Planning for " + numAgents + " agents with subgoal decomposition");

        // Reset yielding state for new search
        yieldingManager.reset();
        
        // Reset displacement tracking for new search
        displacementHistory.clear();
        displacementAttempts = 0;

        // Use iterative subgoal planning
        List<Action[]> plan = planWithSubgoals(initialState, level, startTime);

        if (plan != null && !plan.isEmpty()) {
            logMinimal(getName() + ": Total plan length: " + plan.size());
        }

        return plan;
    }

    /**
     * Plans using subgoal decomposition - moves one box at a time.
     * This naturally handles dependencies between agents.
     */
    private List<Action[]> planWithSubgoals(State initialState, Level level, long startTime) {
        List<Action[]> fullPlan = new ArrayList<>();
        State currentState = initialState;
        int numAgents = initialState.getNumAgents();

        int stuckCount = 0;

        while (!currentState.isGoalState(level)) {
            // Check time limit (PRODUCT.md: 3 minutes)
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                logMinimal(getName() + ": Timeout reached");
                break;
            }

            // Check action limit (PRODUCT.md: 20,000 actions)
            if (fullPlan.size() >= SearchConfig.MAX_ACTIONS) {
                logMinimal(getName() + ": Reached action limit (" + SearchConfig.MAX_ACTIONS + ")");
                break;
            }

            if (stuckCount > SearchConfig.MAX_STUCK_ITERATIONS) {
                logMinimal(getName() + ": Stuck after " + stuckCount + " iterations without progress");
                break;
            }

            // Check for cyclic dependencies when stuck
            if (stuckCount == SearchConfig.DEPENDENCY_CHECK_THRESHOLD && SearchConfig.USE_CBS_ON_CYCLE) {
                System.err.println("\n========== DEPENDENCY ANALYSIS (stuckCount=" + stuckCount + ") ==========");
                DependencyAnalyzer.AnalysisResult depAnalysis = 
                    DependencyAnalyzer.analyze(currentState, level);
                
                if (depAnalysis.hasCycle) {
                    System.err.println("[DEPENDENCY] *** CYCLIC DEPENDENCY DETECTED ***");
                    System.err.println(depAnalysis.report);
                    
                    // Check if we've tried displacement too many times
                    if (displacementAttempts >= MAX_DISPLACEMENT_ATTEMPTS) {
                        System.err.println("[DISPLACEMENT] Already tried " + displacementAttempts + 
                            " times, switching to CBS directly...");
                    } else {
                        // Strategy: Try to break the cycle by temporarily displacing one box
                        System.err.println("[DISPLACEMENT] Attempting to break cycle via temporary displacement (attempt " + 
                            (displacementAttempts + 1) + "/" + MAX_DISPLACEMENT_ATTEMPTS + ")...");
                        
                        displacementAttempts++;
                        
                        List<Action[]> displacementPlan = new ArrayList<>();
                        boolean cycleBreakSuccess = deadlockBreaker.attemptCycleBreaking(
                            displacementPlan, currentState, level, numAgents, 
                            depAnalysis.cycles.isEmpty() ? new ArrayList<>() : depAnalysis.cycles.get(0),
                            pathAnalyzer, conflictResolver);
                        
                        if (cycleBreakSuccess && !displacementPlan.isEmpty()) {
                            System.err.println("[DISPLACEMENT] *** SUCCESS! Displaced box with " + 
                                displacementPlan.size() + " steps ***");
                            fullPlan.addAll(displacementPlan);
                            
                            // Update current state
                            for (Action[] jointAction : displacementPlan) {
                                currentState = applyJointAction(jointAction, currentState, level, numAgents);
                            }
                            
                            // Reset stuck count - we made progress
                            stuckCount = 0;
                            continue; // Retry main loop with new state
                        } else {
                            System.err.println("[DISPLACEMENT] Failed to find displacement, trying CBS...");
                        }
                    }
                    
                    // Fallback: Try CBS (for pure agent coordination)
                    System.err.println("[CBS] Attempting CBS as fallback strategy...");
                    CBSStrategy cbs = new CBSStrategy(heuristic, config);
                    cbs.setTimeout(timeoutMs - (System.currentTimeMillis() - startTime));
                    cbs.setMaxStates(maxStates);
                    
                    List<Action[]> cbsPlan = cbs.search(currentState, level);
                    
                    if (cbsPlan != null && !cbsPlan.isEmpty()) {
                        System.err.println("[CBS] *** SUCCESS! Found plan with " + cbsPlan.size() + " steps ***");
                        fullPlan.addAll(cbsPlan);
                        logMinimal(getName() + ": [OK] Solved via CBS fallback");
                        logMinimal(getName() + ": Total plan length: " + fullPlan.size());
                        return fullPlan;
                    } else {
                        System.err.println("[CBS] Failed to find solution, continuing with Priority Planning");
                    }
                } else {
                    System.err.println("[DEPENDENCY] No cyclic dependency found. Stuck due to other reasons.");
                }
                System.err.println("==========================================================\n");
            }

            // Release yielding agents if stuck near MAX_STUCK_ITERATIONS threshold to prevent deadlock
            if (yieldingManager.hasYieldingAgents() && stuckCount == SearchConfig.MAX_STUCK_ITERATIONS - 5) {
                logNormal("[YIELD] Releasing all yielding agents due to potential deadlock (stuckCount=" + stuckCount
                        + ")");
                yieldingManager.clearAllYielding();
            }

            // Process yielding agents first to ensure they actually move
            if (yieldingManager.hasYieldingAgents()) {
                boolean anyYieldingMoved = false;

                for (Map.Entry<Integer, Integer> entry : new HashMap<>(yieldingManager.getYieldingAgents()).entrySet()) {
                    int yieldingAgentId = entry.getKey();
                    int beneficiaryId = entry.getValue();

                    // Use AgentYieldingManager to check if still blocking
                    if (yieldingManager.isBlockingAgent(yieldingAgentId, beneficiaryId, currentState, level)) {
                        Position yieldingPos = currentState.getAgentPosition(yieldingAgentId);
                        logNormal("[YIELD-CHECK] Agent " + yieldingAgentId +
                                " still blocking at " + yieldingPos + ", forcing move");

                        int planSizeBefore = fullPlan.size();
                        boolean moved = agentCoordinator.forceYieldingAgentToMove(fullPlan, currentState, level,
                                numAgents, yieldingAgentId, pathAnalyzer, conflictResolver);

                        if (moved) {
                            // Update state
                            for (int i = planSizeBefore; i < fullPlan.size(); i++) {
                                currentState = applyJointAction(fullPlan.get(i), currentState, level, numAgents);
                            }
                            anyYieldingMoved = true;
                            stuckCount = 0;
                        } else {
                            // Cannot move yielding agent - clear its yielding state to avoid infinite loop
                            logNormal("[YIELD-CHECK] Agent " + yieldingAgentId + 
                                    " cannot move, releasing from yielding state");
                            yieldingManager.clearYielding(yieldingAgentId);
                        }
                    } else {
                        // Yielding agent has moved out of the way
                        Position yieldingPos = currentState.getAgentPosition(yieldingAgentId);
                        logVerbose("[YIELD-CHECK] Agent " + yieldingAgentId +
                                " no longer blocking (at " + yieldingPos + ")");
                        yieldingManager.clearYielding(yieldingAgentId);
                    }
                }

                if (anyYieldingMoved) {
                    continue; // Restart main loop with updated state
                }
            }

            // Get unsatisfied subgoals prioritized by difficulty
            List<Subgoal> unsatisfied = subgoalManager.getUnsatisfiedSubgoals(currentState, level);

            if (unsatisfied.isEmpty()) {
                break;
            }

            // Sort subgoals by topological depth (deepest first)
            final State stateForSort = currentState;

            // Filter out yielding agents and agents that completed their agent goal and yielded
            unsatisfied.removeIf(sg -> {
                if (yieldingManager.isYielding(sg.agentId)) {
                    logVerbose("[YIELD] Skipping subgoal for Agent " + sg.agentId +
                            " (yielding for Agent " + yieldingManager.getBeneficiary(sg.agentId) + ")");
                    return true;
                }
                // Skip agent goals for agents that already completed and proactively yielded
                if (sg.isAgentGoal && yieldingManager.hasAgentGoalCompletedAndYielded(sg.agentId)) {
                    logVerbose("[YIELD] Skipping agent goal for Agent " + sg.agentId +
                            " (already completed and proactively yielded)");
                    return true;
                }
                return false;
            });

            // Use reverse execution order (HTN-style) for goal ordering
            final List<Subgoal> allUnsatisfied = new ArrayList<>(unsatisfied);
            
            unsatisfied.sort((a, b) -> {
                // PRIMARY: Reverse execution order (lower priority = execute first)
                int reverseA = topologicalAnalyzer.getReverseExecutionPriority(a.goalPos, level);
                int reverseB = topologicalAnalyzer.getReverseExecutionPriority(b.goalPos, level);
                
                if (reverseA != reverseB) {
                    return Integer.compare(reverseA, reverseB); // Lower priority first
                }
                
                // SECONDARY: Blocking score for tie-breaking
                int scoreA = topologicalAnalyzer.computeBlockingScore(a, stateForSort, level, allUnsatisfied, subgoalManager);
                int scoreB = topologicalAnalyzer.computeBlockingScore(b, stateForSort, level, allUnsatisfied, subgoalManager);
                
                if (scoreA != scoreB) {
                    return Integer.compare(scoreA, scoreB);
                }
                
                // TERTIARY: Topological depth (higher = more inner = higher priority)
                int depthA = topologicalAnalyzer.getTopologicalDepth(a.goalPos, level);
                int depthB = topologicalAnalyzer.getTopologicalDepth(b.goalPos, level);

                if (depthA != depthB) {
                    return Integer.compare(depthB, depthA); // Descending
                }

                // Last resort: standard difficulty (easier first)
                int diffA = subgoalManager.estimateSubgoalDifficulty(a, stateForSort, level);
                int diffB = subgoalManager.estimateSubgoalDifficulty(b, stateForSort, level);
                return Integer.compare(diffA, diffB);
            });
            
            // Log the sorted order for debugging
            if (SearchConfig.isNormal() && !unsatisfied.isEmpty()) {
                System.err.println("[REVERSE-ORDER] Subgoal execution order:");
                for (int i = 0; i < Math.min(5, unsatisfied.size()); i++) {
                    Subgoal sg = unsatisfied.get(i);
                    int reversePrio = topologicalAnalyzer.getReverseExecutionPriority(sg.goalPos, level);
                    int depth = topologicalAnalyzer.getTopologicalDepth(sg.goalPos, level);
                    if (sg.isAgentGoal) {
                        System.err.println("  " + (i+1) + ". Agent " + sg.agentId + " -> " + sg.goalPos + 
                            " (reverse=" + reversePrio + ", depth=" + depth + ")");
                    } else {
                        System.err.println("  " + (i+1) + ". Box " + sg.boxType + " -> " + sg.goalPos + 
                            " (reverse=" + reversePrio + ", depth=" + depth + ")");
                    }
                }
            }

            // Group goals by reverse execution priority (lowest = innermost = execute first)
            List<Subgoal> executableGoals = new ArrayList<>();
            List<Subgoal> deferredGoals = new ArrayList<>();
            
            int lowestReversePrio = unsatisfied.isEmpty() ? 0 
                    : topologicalAnalyzer.getReverseExecutionPriority(unsatisfied.get(0).goalPos, level);
            
            for (Subgoal sg : unsatisfied) {
                int reversePrio = topologicalAnalyzer.getReverseExecutionPriority(sg.goalPos, level);
                if (reversePrio == lowestReversePrio) {
                    executableGoals.add(sg);
                } else {
                    deferredGoals.add(sg);
                }
            }
            
            List<Subgoal> highestDepthGoals = executableGoals;
            List<Subgoal> lowerDepthGoals = deferredGoals;

            boolean madeProgress = false;
            Subgoal highestPriorityFailedGoal = null;

            // Try executable goals (lowest reverse priority)
            for (Subgoal subgoal : highestDepthGoals) {
                if (System.currentTimeMillis() - startTime > timeoutMs) {
                    break;
                }

                List<Action> path = null;
                Position boxToMove = null;

                if (subgoal.isAgentGoal) {
                    // Agent goal: move agent to target position
                    path = boxSearchPlanner.searchForAgentGoal(subgoal.agentId, subgoal.goalPos, currentState, level);
                } else {
                    // Box goal: find box and move to goal
                    boxToMove = subgoalManager.findBestBoxForGoal(subgoal, currentState, level);
                    if (boxToMove == null) {
                        continue;
                    }
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxToMove,
                            subgoal.goalPos, subgoal.boxType, currentState, level, new HashSet<>());
                }

                if (path != null && !path.isEmpty()) {
                    // Plan merging: let other agents act too
                    for (Action action : path) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                                subgoal.agentId, action, currentState, level, numAgents, subgoal.isAgentGoal);

                        // Resolve conflicts - primary agent has right of way
                        jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, subgoal.agentId);

                        fullPlan.add(jointAction);
                        currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    }

                    // Validate goal reached
                    boolean goalReached = false;
                    if (subgoal.isAgentGoal) {
                        Position currentPos = currentState.getAgentPosition(subgoal.agentId);
                        goalReached = currentPos.equals(subgoal.goalPos);

                        if (goalReached) {
                            logMinimal(getName() + ": [OK] Agent " + subgoal.agentId +
                                    " moved to goal position " + subgoal.goalPos + " (path: " + path.size() + " steps)");
                        } else {
                            logNormal(getName() + ": [PARTIAL] Agent " + subgoal.agentId +
                                    " executed path but did NOT reach goal " + subgoal.goalPos +
                                    " (current: " + currentPos + ", expected: " + subgoal.goalPos + ")");
                            // Do NOT break - continue trying to complete this goal
                            continue;
                        }
                    } else {
                        // Box goal - verify box is at goal position
                        char actualBox = currentState.getBoxAt(subgoal.goalPos);
                        goalReached = (actualBox == subgoal.boxType);

                        if (goalReached) {
                            logMinimal(getName() + ": [OK] Agent " + subgoal.agentId +
                                    " moved box " + subgoal.boxType + " (path: " + path.size() + " steps)");
                        } else {
                            logNormal(getName() + ": [PARTIAL] Agent " + subgoal.agentId +
                                    " executed path but box " + subgoal.boxType + " NOT at goal " +
                                    subgoal.goalPos);
                            continue;
                        }
                    }

                    // Only mark progress if goal was actually reached
                    if (goalReached) {
                        madeProgress = true;
                        stuckCount = 0;

                        // Release any agents that were yielding for this agent
                        agentCoordinator.clearYieldingForBeneficiary(subgoal.agentId, yieldingManager);
                        
                        // Proactive yielding: move agent to safe position after task completion
                        if (performProactiveYielding(subgoal.agentId, fullPlan, currentState, level, numAgents)) {
                            // Update currentState after proactive yielding moves
                            State tempState = initialState;
                            for (Action[] jointAction : fullPlan) {
                                tempState = applyJointAction(jointAction, tempState, level, numAgents);
                            }
                            currentState = tempState;
                        }
                        
                        break;
                    }
                } else {
                    // Track the highest priority goal that failed
                    // This is the FIRST goal in sorted list (deepest) that failed
                    if (highestPriorityFailedGoal == null) {
                        highestPriorityFailedGoal = subgoal;
                        logVerbose("[BLOCKED] Highest priority goal blocked: Agent " + subgoal.agentId +
                                (subgoal.isAgentGoal ? " to " : " box " + subgoal.boxType + " to ") +
                                subgoal.goalPos + " (depth=" + topologicalAnalyzer.getTopologicalDepth(subgoal.goalPos, level)
                                + ")");
                    }
                }
            }

            // NEW: If a high-priority goal failed, try to clear the path BEFORE falling
            // back to lower priority goals
            if (!madeProgress && highestPriorityFailedGoal != null && stuckCount < 3) {
                // TODO: Fix tryPreemptivePathClearing signature
                /*
                boolean clearedForHighPriority = deadlockBreaker.tryPreemptivePathClearing(
                        fullPlan, currentState, level, numAgents, highestPriorityFailedGoal,
                        yieldingManager, goalChecker, pathAnalyzer, conflictResolver, planMerger);
                if (clearedForHighPriority) {
                    for (int i = fullPlan.size() - 1; i >= 0; i--) {
                        // Update currentState based on added actions
                    }
                    // Re-calculate currentState from all actions
                    State tempState = initialState;
                    for (Action[] jointAction : fullPlan) {
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    }
                    currentState = tempState;
                    logNormal(getName() + ": Cleared path for high-priority goal, retrying...");
                    continue;
                }
                */
                // For now, skip this optimization
            }

            if (!madeProgress) {
                stuckCount++;
                // Log which subgoals are failing
                if (stuckCount == 1 || stuckCount % SearchConfig.STUCK_LOG_INTERVAL == 0) {
                    logNormal(getName() + ": Stuck iteration " + stuckCount +
                            ", remaining subgoals:");
                    for (Subgoal sg : unsatisfied) {
                        int diff = subgoalManager.estimateSubgoalDifficulty(sg, currentState, level);
                        if (sg.isAgentGoal) {
                            logNormal("  - Agent " + sg.agentId + " -> goal position " +
                                    sg.goalPos + " (difficulty: " + diff + ")");
                        } else {
                            logNormal("  - Agent " + sg.agentId + " box " + sg.boxType +
                                    " -> goal " + sg.goalPos + " (difficulty: " + diff + ")");
                        }
                    }
                }

                // Try random re-ordering of subgoals when stuck
                boolean foundWithReorder = false;
                for (int reorderAttempt = 0; reorderAttempt < SearchConfig.MAX_REORDER_ATTEMPTS; reorderAttempt++) {
                    Collections.shuffle(highestDepthGoals, random);

                    for (Subgoal subgoal : highestDepthGoals) {
                        List<Action> path = null;

                        if (subgoal.isAgentGoal) {
                            path = boxSearchPlanner.searchForAgentGoal(subgoal.agentId, subgoal.goalPos, currentState, level);
                        } else {
                            Position boxToMove = subgoalManager.findBestBoxForGoal(subgoal, currentState, level);
                            if (boxToMove == null)
                                continue;
                            path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxToMove,
                                    subgoal.goalPos, subgoal.boxType, currentState, level, new HashSet<>());
                        }

                        if (path != null && !path.isEmpty()) {
                            // Execute with plan merging
                            for (Action action : path) {
                                Action[] jointAction = planMerger.createJointActionWithMerging(
                                        subgoal.agentId, action, currentState, level, numAgents, subgoal.isAgentGoal);
                                // Resolve conflicts - primary agent has right of way
                                jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, subgoal.agentId);

                                fullPlan.add(jointAction);
                                currentState = applyJointAction(jointAction, currentState, level, numAgents);
                            }
                            // Validate goal reached
                            boolean goalReached = false;
                            if (subgoal.isAgentGoal) {
                                Position currentPos = currentState.getAgentPosition(subgoal.agentId);
                                goalReached = currentPos.equals(subgoal.goalPos);

                                if (goalReached) {
                                    logNormal(getName() + ": Agent " + subgoal.agentId +
                                            " moved to goal position after reorder (path: " + path.size() + " steps)");
                                } else {
                                    logNormal(getName() + ": [PARTIAL] Agent " + subgoal.agentId +
                                            " executed reordered path but did NOT reach goal " + subgoal.goalPos +
                                            " (current: " + currentPos + ")");
                                    continue;
                                }
                            } else {
                                // Box goal - verify box is at goal position
                                char actualBox = currentState.getBoxAt(subgoal.goalPos);
                                goalReached = (actualBox == subgoal.boxType);

                                if (goalReached) {
                                    logNormal(getName() + ": Agent " + subgoal.agentId +
                                            " moved box " + subgoal.boxType + " after reorder (path: " + path.size()
                                            + " steps)");
                                } else {
                                    logNormal(getName() + ": [PARTIAL] Agent " + subgoal.agentId +
                                            " executed reordered path but box " + subgoal.boxType + " NOT at goal " +
                                            subgoal.goalPos);
                                    continue;
                                }
                            }

                            if (goalReached) {
                                foundWithReorder = true;
                                stuckCount = 0;
                                agentCoordinator.clearYieldingForBeneficiary(subgoal.agentId, yieldingManager);
                                
                                // PROACTIVE YIELDING: After completing task via reorder
                                if (performProactiveYielding(subgoal.agentId, fullPlan, currentState, level, numAgents)) {
                                    // Update currentState
                                    State tempState = initialState;
                                    for (Action[] jointAction : fullPlan) {
                                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                                    }
                                    currentState = tempState;
                                }
                                
                                break;
                            }
                        }
                    }
                    if (foundWithReorder)
                        break;
                }

                // IMPROVEMENT 2: If still stuck, try greedy step with plan merging
                if (!foundWithReorder) {
                    // Try clearing blocking agents before giving up
                    boolean clearedPath = false;
                    if (stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING && !highestDepthGoals.isEmpty()) {
                        // Find critical positions that need to be cleared for the highest priority goal
                        Subgoal blockedGoal = highestDepthGoals.get(0);
                        Set<Position> criticalPositions = new HashSet<>();
                        criticalPositions.add(blockedGoal.goalPos);
                        
                        // Add positions on the path to the goal
                        Position agentPos = currentState.getAgentPosition(blockedGoal.agentId);
                        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(currentState, level);
                        Set<Position> pathPositions = pathAnalyzer.findCriticalPositionsForAgentGoal(
                            currentState, level, blockedGoal.agentId, blockedGoal.goalPos, satisfiedGoals);
                        criticalPositions.addAll(pathPositions);
                        
                        int planSizeBefore = fullPlan.size();
                        AgentCoordinator.ClearingResult clearResult = agentCoordinator.tryIdleAgentClearingWithResult(
                            fullPlan, currentState, level, numAgents, blockedGoal.agentId, criticalPositions,
                            pathAnalyzer, conflictResolver);
                        
                        if (clearResult.success) {
                            // CRITICAL: Mark the cleared agent as yielding for the blocked agent
                            yieldingManager.setYielding(clearResult.clearedAgentId, blockedGoal.agentId);
                            
                            // Apply all new actions to update currentState
                            for (int i = planSizeBefore; i < fullPlan.size(); i++) {
                                currentState = applyJointAction(fullPlan.get(i), currentState, level, numAgents);
                            }
                            stuckCount = 0;
                            clearedPath = true;
                            logNormal(getName() + ": Cleared Agent " + clearResult.clearedAgentId + 
                                " (now yielding for Agent " + blockedGoal.agentId + ")");
                        }
                    }
                    
                    // Try deadlock box displacement if agent clearing didn't help
                    if (!clearedPath && stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING + 1) {
                        // Collect blocked agent IDs
                        List<Integer> blockedAgentIds = new ArrayList<>();
                        for (Subgoal sg : highestDepthGoals) {
                            blockedAgentIds.add(sg.agentId);
                        }
                        
                        // Analyze blocking relationships
                        List<DeadlockResolver.BlockingInfo> blockingInfos = 
                            deadlockResolver.analyzeBlocking(currentState, level, blockedAgentIds);
                        
                        if (!blockingInfos.isEmpty()) {
                            logNormal("\n[DEADLOCK] Analyzing blocking relationships:");
                            for (DeadlockResolver.BlockingInfo info : blockingInfos) {
                                logNormal("[DEADLOCK]   " + info);
                            }
                            
                            // Try displacement
                            DeadlockResolver.DisplacementPlan displacementPlan = 
                                deadlockResolver.createDisplacementPlan(blockingInfos, currentState, level, displacementHistory);
                            
                            if (displacementPlan != null) {
                                logNormal("[DEADLOCK] Attempting displacement: " + displacementPlan);
                                
                                // Record this displacement attempt
                                String historyKey = displacementPlan.boxType + "@" + displacementPlan.boxPosition;
                                displacementHistory.add(historyKey);
                                
                                // Execute displacement
                                List<Action> displacePath = boxSearchPlanner.planBoxDisplacement(
                                    displacementPlan.agentId, displacementPlan.boxPosition, 
                                    displacementPlan.tempPosition, displacementPlan.boxType,
                                    currentState, level);
                                
                                if (displacePath != null && !displacePath.isEmpty()) {
                                    logNormal("[DEADLOCK] *** SUCCESS! Displacing box " + displacementPlan.boxType +
                                        " with " + displacePath.size() + " steps ***");
                                    
                                    // Execute displacement plan
                                    for (Action action : displacePath) {
                                        Action[] jointAction = planMerger.createJointActionWithMerging(
                                            displacementPlan.agentId, action, currentState, level, 
                                            numAgents, false);
                                        jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, 
                                            displacementPlan.agentId);
                                        fullPlan.add(jointAction);
                                        currentState = applyJointAction(jointAction, currentState, level, numAgents);
                                    }
                                    
                                    stuckCount = 0;
                                    clearedPath = true;
                                    logNormal("[DEADLOCK] Box displaced, continuing with normal planning");
                                } else {
                                    logNormal("[DEADLOCK] Failed to find displacement path");
                                }
                            } else {
                                logNormal("[DEADLOCK] No suitable displacement found");
                            }
                        }
                    }

                    if (!clearedPath) {
                        GreedyPlanner.AgentStateProvider stateProvider = new GreedyPlanner.AgentStateProvider() {
                            public boolean isYielding(int agentId) { return yieldingManager.isYielding(agentId); }
                            public boolean hasCompletedTask(int agentId) { return false; }
                            public int getBeneficiary(int agentId) { return yieldingManager.getBeneficiary(agentId); }
                        };
                        GreedyPlanner.ActionEvaluator evaluator = new GreedyPlanner.ActionEvaluator() {
                            public int estimateAgentCost(int agentId, State state, Level level) {
                                return PriorityPlanningStrategy.this.estimateAgentCost(agentId, state, level);
                            }
                        };
                        boolean anyMove = greedyPlanner.tryGreedyStepWithMerging(fullPlan, currentState, level, numAgents,
                                stateProvider, evaluator, conflictResolver, planMerger);
                        if (anyMove) {
                            Action[] lastAction = fullPlan.get(fullPlan.size() - 1);
                            currentState = applyJointAction(lastAction, currentState, level, numAgents);
                        }
                    }
                } else {
                    madeProgress = true;
                }
            }
        }

        if (currentState.isGoalState(level)) {
            logMinimal(getName() + ": [OK] Goal state reached!");
        } else {
            logMinimal(getName() + ": [FAIL] Could not reach goal state");
        }

        return fullPlan.isEmpty() ? null : fullPlan;
    }

    /** Estimates total cost for an agent using true distance heuristic. */
    private int estimateAgentCost(int agentId, State state, Level level) {
        int cost = 0;
        Color agentColor = level.getAgentColor(agentId);

        // Calculate box goal costs
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            char boxType = box.getValue();
            if (level.getBoxColor(boxType) == agentColor) {
                Position boxPos = box.getKey();

                // Skip already satisfied goals
                if (level.getBoxGoal(boxPos) == boxType) {
                    continue;
                }

                // Find closest unsatisfied goal for this box type
                int minDist = Integer.MAX_VALUE;
                for (int r = 0; r < level.getRows(); r++) {
                    for (int c = 0; c < level.getCols(); c++) {
                        if (level.getBoxGoal(r, c) == boxType) {
                            Position goalPos = new Position(r, c);
                            Character currentBox = state.getBoxes().get(goalPos);
                            // Skip already satisfied goals
                            if (currentBox != null && currentBox == boxType) {
                                continue;
                            }
                            int dist;
                            if (heuristic instanceof TrueDistanceHeuristic) {
                                dist = ((TrueDistanceHeuristic) heuristic).getDistance(boxPos, goalPos);
                                if (dist == Integer.MAX_VALUE) {
                                    dist = boxPos.manhattanDistance(goalPos);
                                }
                            } else {
                                dist = boxPos.manhattanDistance(goalPos);
                            }
                            minDist = Math.min(minDist, dist);
                        }
                    }
                }

                if (minDist < Integer.MAX_VALUE) {
                    cost += minDist;
                }
            }
        }

        // Calculate agent position goal cost only after all box goals are satisfied
        if (GoalChecker.allBoxGoalsSatisfied(state, level)) {
            Position agentPos = state.getAgentPosition(agentId);

            // Add HUGE penalty if agent is in a position that blocks higher-priority agents
            Set<Position> blockingPositions = getPositionsThatWouldBlockHigherPriority(agentId, state, level);
            if (blockingPositions.contains(agentPos)) {
                cost += 10000;
            }

            for (int row = 0; row < level.getRows(); row++) {
                for (int col = 0; col < level.getCols(); col++) {
                    if (level.getAgentGoal(row, col) == agentId) {
                        Position goalPos = new Position(row, col);
                        if (!agentPos.equals(goalPos)) {
                            if (heuristic instanceof TrueDistanceHeuristic) {
                                int dist = ((TrueDistanceHeuristic) heuristic).getDistance(agentPos, goalPos);
                                cost += (dist < Integer.MAX_VALUE) ? dist : agentPos.manhattanDistance(goalPos);
                            } else {
                                cost += agentPos.manhattanDistance(goalPos);
                            }
                        }
                        break;
                    }
                }
            }
        }

        return cost;
    }

    /** Represents a subgoal: moving a box or agent to a goal position. */
    public static class Subgoal {
        final int agentId;
        final char boxType; // '\0' for agent goals
        final Position goalPos;
        final boolean isAgentGoal; // true if this is an agent position goal

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

    /** Applies a joint action to the state and returns the new state. */
    private State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        return planMerger.applyJointAction(jointAction, state, numAgents);
    }

    /** Move completed agents out of corridors to avoid blocking. */
    private boolean performProactiveYielding(int completedAgentId, List<Action[]> plan, 
            State currentState, Level level, int numAgents) {
        
        Position currentPos = currentState.getAgentPosition(completedAgentId);
        
        // Check if agent has an agent goal to reach
        Position agentGoal = GoalChecker.findAgentGoalPosition(completedAgentId, level);
        if (agentGoal != null && !currentPos.equals(agentGoal)) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " has agent goal at " + agentGoal + 
                    ", skipping proactive yield (will go to goal in Phase 2)");
            return false;
        }
        
        yieldingManager.markTaskCompleted(completedAgentId);
        
        int passableNeighbors = pathAnalyzer.countPassableNeighbors(currentPos, level);
        int freeNeighbors = pathAnalyzer.countFreeNeighbors(currentPos, currentState, level);
        
        logNormal("[PROACTIVE-YIELD-CHECK] Agent " + completedAgentId + " at " + currentPos + 
                ", passableNeighbors=" + passableNeighbors + " (map freeNeighbors=" + freeNeighbors + ")");
        
        List<Integer> blockedAgents = yieldingManager.findBlockedAgents(completedAgentId, currentState, level);
        boolean inCorridor = (passableNeighbors <= 2);
        
        if (blockedAgents.isEmpty() && !inCorridor) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " not blocking anyone and not in corridor");
            return false;
        }
        
        if (inCorridor) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " is in CORRIDOR, must move out");
        }
        if (!blockedAgents.isEmpty()) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " blocking agents: " + blockedAgents);
        }
        
        Position safePos = findBestYieldPosition(completedAgentId, currentState, level, blockedAgents);
        
        if (safePos == null) {
            logNormal("[PROACTIVE-YIELD] No yield position found");
            // TODO: Implement findCorridorExitMove in AgentCoordinator
            /*
            Action yieldMove = agentCoordinator.findCorridorExitMove(completedAgentId, currentState, level, pathAnalyzer);
            if (yieldMove != null) {
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[completedAgentId] = yieldMove;
                plan.add(jointAction);
                logNormal("[PROACTIVE-YIELD] Performed corridor exit move");
                return true;
            }
            */
            return false;
        }
        
        List<Action> pathToSafe = yieldingManager.planPathToPosition(completedAgentId, safePos, currentState, level);
        
        if (pathToSafe.isEmpty()) {
            logNormal("[PROACTIVE-YIELD] No path to yield position");
            return false;
        }
        
        logNormal("[PROACTIVE-YIELD] Moving to yield position " + safePos + " (" + pathToSafe.size() + " steps)");
        
        // If agent was at its agent goal, mark it as completed and yielded to prevent re-planning
        if (agentGoal != null && currentPos.equals(agentGoal)) {
            yieldingManager.markAgentGoalCompletedAndYielded(completedAgentId);
        }
        
        State tempState = currentState;
        for (Action action : pathToSafe) {
            Action[] jointAction = new Action[numAgents];
            Arrays.fill(jointAction, Action.noOp());
            jointAction[completedAgentId] = action;
            
            if (!tempState.isApplicable(action, completedAgentId, level)) {
                logNormal("[PROACTIVE-YIELD] Path interrupted");
                break;
            }
            
            plan.add(jointAction);
            tempState = applyJointAction(jointAction, tempState, level, numAgents);
        }
        
        return true;
    }

    /** Finds safe yield position preferring junctions, avoiding global working area. */
    private Position findBestYieldPosition(int agentId, State state, Level level, List<Integer> blockedAgents) {
        Position currentPos = state.getAgentPosition(agentId);
        
        // Use SafeZoneCalculator for proper global working area analysis (cached)
        Position safePosFromCalculator = safeZoneCalculator.findSafePosition(agentId, state, level);
        if (safePosFromCalculator != null) {
            int passableNeighbors = pathAnalyzer.countPassableNeighbors(safePosFromCalculator, level);
            logNormal("[YIELD] SafeZoneCalculator found position " + safePosFromCalculator + 
                    " (passableNeighbors=" + passableNeighbors + ")");
            return safePosFromCalculator;
        }
        
        // SafeZoneCalculator already tried everything, use BFS with global working area
        // (Note: removed duplicate call to yieldingManager.findNearestSafePosition which internally
        //  calls the same safeZoneCalculator.findSafePosition - now returns same cached result)
        
        // Get cached global working area
        Set<Position> globalWorkingArea = safeZoneCalculator.computeGlobalWorkingArea(agentId, state, level);
        
        // Last resort: BFS search with global working area check
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Integer> distanceMap = new HashMap<>();
        
        queue.add(currentPos);
        visited.add(currentPos);
        distanceMap.put(currentPos, 0);
        
        Position bestPos = null;
        int bestScore = -1; // Higher is better
        int maxDistance = Math.min(level.getRows() + level.getCols(), 50);
        
        while (!queue.isEmpty()) {
            Position pos = queue.poll();
            int distance = distanceMap.get(pos);
            
            if (distance > maxDistance) break;
            
            // Skip current position
            if (!pos.equals(currentPos)) {
                // Use passableNeighbors that considers current state
                int passableNeighbors = pathAnalyzer.countPassableNeighbors(pos, level);
                int freeNeighbors = pathAnalyzer.countFreeNeighbors(pos, state, level); // Map structure
                
                // Check not occupied
                if (state.getBoxAt(pos) == '\0' && !PlanningUtils.isPositionOccupiedByAgent(pos, state, state.getNumAgents())) {
                    // Only consider non-corridors (passableNeighbors >= 3) or dead-ends (freeNeighbors == 1)
                    boolean isGoodSpot = (passableNeighbors >= 3) || (freeNeighbors == 1);
                    
                    if (!isGoodSpot) {
                        // Skip corridor positions - don't move from one corridor to another!
                        // But still explore through them
                    } else {
                        // Score: prefer more passable neighbors, penalize distance
                        int score = passableNeighbors * 10 - distance;
                        
                        // Bonus for dead-ends (good parking spots that won't block anyone)
                        if (freeNeighbors == 1) {
                            score += 20; // Dead-ends are excellent parking
                        }
                        
                        // IMPROVED: Heavy penalty if in global working area (would block ANY agent's future work)
                        if (globalWorkingArea.contains(pos)) {
                            score -= 200; // Very heavy penalty - this position is needed by someone
                        }
                        
                        // Additional penalty if this position would block any specifically blocked agent
                        boolean wouldBlock = false;
                        for (int otherId : blockedAgents) {
                            Position otherPos = state.getAgentPosition(otherId);
                            Position otherGoal = GoalChecker.findAgentGoalPosition(otherId, level);
                            if (otherGoal != null) {
                                Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);
                                Set<Position> criticalPath = pathAnalyzer.findCriticalPositionsForAgentGoal(
                                    state, level, otherId, otherGoal, satisfiedGoals);
                                if (criticalPath.contains(pos)) {
                                    wouldBlock = true;
                                    break;
                                }
                            }
                        }
                        if (wouldBlock) {
                            score -= 100; // Heavy penalty
                        }
                        
                        if (score > bestScore) {
                            bestScore = score;
                            bestPos = pos;
                        }
                    }
                }
            }
            
            // Explore neighbors (even through corridors)
            for (Direction dir : Direction.values()) {
                Position next = pos.move(dir);
                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    distanceMap.put(next, distance + 1);
                    queue.add(next);
                }
            }
        }
        
        if (bestPos != null) {
            int passable = pathAnalyzer.countPassableNeighbors(bestPos, level);
            logNormal("[YIELD] Found best yield position " + bestPos + " (passableNeighbors=" + passable + ", score=" + bestScore + ")");
        }
        
        return bestPos;
    }

    /** Gets positions that would block higher-priority agents. */
    private Set<Position> getPositionsThatWouldBlockHigherPriority(int agentId, State state, Level level) {
        Set<Position> blocking = new HashSet<>();

        // Only relevant during Agent Goal phase
        if (!GoalChecker.allBoxGoalsSatisfied(state, level)) {
            return blocking;
        }

        Position myGoal = GoalChecker.findAgentGoalPosition(agentId, level);
        int myDepth = (myGoal != null) ? topologicalAnalyzer.getTopologicalDepth(myGoal, level) : 0;

        for (int otherId = 0; otherId < state.getNumAgents(); otherId++) {
            if (otherId == agentId)
                continue;

            Position otherGoal = GoalChecker.findAgentGoalPosition(otherId, level);
            if (otherGoal == null)
                continue;

            // Check if other agent has already reached goal
            if (state.getAgentPosition(otherId).equals(otherGoal))
                continue;

            int otherDepth = topologicalAnalyzer.getTopologicalDepth(otherGoal, level);

            // If other agent has higher priority (deeper goal)
            if (otherDepth > myDepth) {
                Position otherPos = state.getAgentPosition(otherId);
                Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);
                Set<Position> otherCriticalPath = pathAnalyzer.findCriticalPositionsForAgentGoal(
                    state, level, otherId, otherGoal, satisfiedGoals);
                blocking.addAll(otherCriticalPath);
            }
        }

        return blocking;
    }
}