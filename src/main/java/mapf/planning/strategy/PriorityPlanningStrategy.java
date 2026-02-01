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
 * Priority-based planning strategy for multi-agent path finding with subgoal
 * decomposition.
 * 
 * Algorithm improvements based on course materials (ARCHITECTURE.md):
 * 1. Priority Re-ordering: If an agent fails, try different priority orderings
 * 2. Subgoal Decomposition: Plan one box at a time instead of all boxes
 * simultaneously
 * 3. True Distance Heuristic: Use precomputed BFS distances instead of
 * Manhattan distance
 * 
 * This addresses tightly-coupled scenarios where agents must coordinate (e.g.,
 * Agent 2
 * must move boxes out of Agent 0's path before Agent 0 can proceed).
 * 
 * Constraints from PRODUCT.md:
 * - Time limit: 3 minutes per level
 * - Action limit: 20,000 joint actions per level
 * - Agents can only move boxes of the same color
 */
public class PriorityPlanningStrategy implements SearchStrategy {

    private final Heuristic heuristic;
    private final SearchConfig config;
    private long timeoutMs;
    private int maxStates;
    private final Random random = new Random(SearchConfig.RANDOM_SEED);

    // ========== Refactored Helper Classes (SRP compliant) ==========
    private final SubgoalManager subgoalManager;
    private final SubgoalSearcher subgoalSearcher;
    private final TopologicalAnalyzer topologicalAnalyzer;
    private final ConflictResolver conflictResolver;
    private final ImmovableBoxDetector immovableDetector;
    
    // Legacy helpers (to be refactored further)
    private final AgentYieldingManager yieldingManager = new AgentYieldingManager();
    private final DeadlockResolver deadlockResolver = new DeadlockResolver();
    private final SafeZoneCalculator safeZoneCalculator = new SafeZoneCalculator();

    /**
     * Tracks displacement attempts to avoid infinite loops.
     * Key: "box@(x,y)" format representing a box that was displaced
     * This prevents repeatedly displacing the same box back and forth
     */
    private Set<String> displacementHistory = new HashSet<>();
    
    /**
     * Counts how many times we've attempted displacement in current search.
     * After MAX_DISPLACEMENT_ATTEMPTS, force CBS instead.
     */
    private int displacementAttempts = 0;
    private static final int MAX_DISPLACEMENT_ATTEMPTS = 3;

    // ========== Conditional Logging Helpers ==========
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

            // Dependency analysis: when stuck, check if cyclic dependencies exist
            // This helps diagnose WHY we're stuck and whether CBS would help
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
                        
                        List<Action[]> displacementPlan = attemptCycleBreaking(
                            currentState, level, depAnalysis.cycles, numAgents, 
                            timeoutMs - (System.currentTimeMillis() - startTime));
                        
                        if (displacementPlan != null && !displacementPlan.isEmpty()) {
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

            // Check for yielding timeout - if agents have been yielding for too long,
            // release them to prevent permanent deadlock.
            // CRITICAL: Only trigger ONCE at the exact threshold, not every iteration after.
            // Also only trigger if we're about to give up anyway (near MAX_STUCK_ITERATIONS).
            if (yieldingManager.hasYieldingAgents() && stuckCount == SearchConfig.MAX_STUCK_ITERATIONS - 5) {
                logNormal("[YIELD] Releasing all yielding agents due to potential deadlock (stuckCount=" + stuckCount
                        + ")");
                yieldingManager.clearAllYielding();
            }

            // Process yielding agents FIRST - make sure they actually move!
            // If an agent is marked as YIELDING but still blocking the beneficiary's path,
            // we must force it to move before continuing with normal planning.
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
                        boolean moved = forceYieldingAgentToMove(fullPlan, currentState, level,
                                numAgents, yieldingAgentId, beneficiaryId);

                        if (moved) {
                            // Update state
                            for (int i = planSizeBefore; i < fullPlan.size(); i++) {
                                currentState = applyJointAction(fullPlan.get(i), currentState, level, numAgents);
                            }
                            anyYieldingMoved = true;
                            stuckCount = 0;
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
            List<Subgoal> unsatisfied = getUnsatisfiedSubgoals(currentState, level);

            if (unsatisfied.isEmpty()) {
                break; // All done
            }

            // Sort subgoals using TOPOLOGICAL DEPTH (deepest first)
            // Key insight: Goals deep inside dead-ends MUST be filled FIRST
            // because outer goals will permanently block access to inner goals.
            //
            // Also: Agents in YIELDING state should NOT be given tasks that
            // would move them back toward their original positions.
            ensureTopologicalDepthsComputed(level);
            final State stateForSort = currentState;

            // Filter out subgoals for yielding agents (they must stay parked)
            unsatisfied.removeIf(sg -> {
                if (yieldingManager.isYielding(sg.agentId)) {
                    logVerbose("[YIELD] Skipping subgoal for Agent " + sg.agentId +
                            " (yielding for Agent " + yieldingManager.getBeneficiary(sg.agentId) + ")");
                    return true;
                }
                return false;
            });

            // NEW: Use REVERSE PLANNING for goal ordering (HTN-style)
            // Key insight: Ask "which goal should be filled LAST?" and work backwards
            // This automatically discovers correct order for corridors/spirals
            ensureReverseOrderComputed(level);
            final List<Subgoal> allUnsatisfied = new ArrayList<>(unsatisfied);
            
            unsatisfied.sort((a, b) -> {
                // PRIMARY: Reverse execution order (lower priority = execute FIRST)
                // This is the HTN-discovered "recipe" for the level
                int reverseA = getReverseExecutionPriority(a.goalPos, level);
                int reverseB = getReverseExecutionPriority(b.goalPos, level);
                
                if (reverseA != reverseB) {
                    return Integer.compare(reverseA, reverseB); // Lower priority first
                }
                
                // SECONDARY: Blocking score for tie-breaking
                int scoreA = computeBlockingScore(a, stateForSort, level, allUnsatisfied);
                int scoreB = computeBlockingScore(b, stateForSort, level, allUnsatisfied);
                
                if (scoreA != scoreB) {
                    return Integer.compare(scoreA, scoreB);
                }
                
                // TERTIARY: Topological depth (higher = more inner = higher priority)
                int depthA = goalTopologicalDepths.getOrDefault(a.goalPos, 0);
                int depthB = goalTopologicalDepths.getOrDefault(b.goalPos, 0);

                if (depthA != depthB) {
                    return Integer.compare(depthB, depthA); // Descending
                }

                // Last resort: standard difficulty (easier first)
                int diffA = estimateSubgoalDifficulty(a, stateForSort, level);
                int diffB = estimateSubgoalDifficulty(b, stateForSort, level);
                return Integer.compare(diffA, diffB);
            });
            
            // Log the sorted order for debugging
            if (SearchConfig.isNormal() && !unsatisfied.isEmpty()) {
                System.err.println("[REVERSE-ORDER] Subgoal execution order:");
                for (int i = 0; i < Math.min(5, unsatisfied.size()); i++) {
                    Subgoal sg = unsatisfied.get(i);
                    int reversePrio = getReverseExecutionPriority(sg.goalPos, level);
                    int depth = goalTopologicalDepths.getOrDefault(sg.goalPos, 0);
                    if (sg.isAgentGoal) {
                        System.err.println("  " + (i+1) + ". Agent " + sg.agentId + " -> " + sg.goalPos + 
                            " (reverse=" + reversePrio + ", depth=" + depth + ")");
                    } else {
                        System.err.println("  " + (i+1) + ". Box " + sg.boxType + " -> " + sg.goalPos + 
                            " (reverse=" + reversePrio + ", depth=" + depth + ")");
                    }
                }
            }

            // Group goals by reverse execution priority:
            // Goals with the LOWEST reverse priority should be executed FIRST
            // (they are the innermost goals that must be filled before outer ones)
            List<Subgoal> executableGoals = new ArrayList<>();
            List<Subgoal> deferredGoals = new ArrayList<>();
            
            int lowestReversePrio = unsatisfied.isEmpty() ? 0 
                    : getReverseExecutionPriority(unsatisfied.get(0).goalPos, level);
            
            for (Subgoal sg : unsatisfied) {
                int reversePrio = getReverseExecutionPriority(sg.goalPos, level);
                // Goals with same reverse priority are equally executable
                if (reversePrio == lowestReversePrio) {
                    executableGoals.add(sg);
                } else {
                    deferredGoals.add(sg);
                }
            }
            
            // Keep the old naming for compatibility with rest of code
            List<Subgoal> highestDepthGoals = executableGoals;
            List<Subgoal> lowerDepthGoals = deferredGoals;

            boolean madeProgress = false;
            Subgoal highestPriorityFailedGoal = null; // Track the deepest goal that failed

            // First: ONLY try executable goals (those with lowest reverse priority)
            for (Subgoal subgoal : highestDepthGoals) {
                if (System.currentTimeMillis() - startTime > timeoutMs) {
                    break;
                }

                List<Action> path = null;
                Position boxToMove = null;

                if (subgoal.isAgentGoal) {
                    // Agent goal: move agent to target position
                    path = searchForAgentGoal(subgoal.agentId, subgoal.goalPos, currentState, level);
                } else {
                    // Box goal: find box and move to goal
                    boxToMove = findBestBoxForGoal(subgoal, currentState, level);
                    if (boxToMove == null) {
                        continue;
                    }
                    path = searchForSubgoal(subgoal.agentId, boxToMove,
                            subgoal.goalPos, subgoal.boxType, currentState, level);
                }

                if (path != null && !path.isEmpty()) {
                    // IMPROVEMENT 3: Plan Merging - let other agents act too (slides06)
                    for (Action action : path) {
                        Action[] jointAction = createJointActionWithMerging(
                                subgoal.agentId, action, currentState, level, numAgents, subgoal.isAgentGoal);

                        // Resolve any conflicts - primary agent has right of way
                        jointAction = resolveConflicts(jointAction, currentState, level, subgoal.agentId);

                        // SAFETY CHECK: Verify primary agent's action wasn't corrupted
                        if (jointAction[subgoal.agentId].type != action.type) {
                            logNormal("[WARNING] Primary agent " + subgoal.agentId +
                                    "'s action was modified from " + action + " to " +
                                    jointAction[subgoal.agentId] + " - this should not happen!");
                        }

                        fullPlan.add(jointAction);

                        // Update current state
                        currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    }

                    // CRITICAL FIX: Validate agent actually reached goal
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
                            continue;  // Go to next subgoal attempt
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
                        clearYieldingForBeneficiary(subgoal.agentId);
                        
                        // PROACTIVE YIELDING: After completing a task, check if this agent 
                        // is now blocking other agents' paths and move to a safe position
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
                                subgoal.goalPos + " (depth=" + goalTopologicalDepths.getOrDefault(subgoal.goalPos, 0)
                                + ")");
                    }
                }
            }

            // NEW: If a high-priority goal failed, try to clear the path BEFORE falling
            // back to lower priority goals
            if (!madeProgress && highestPriorityFailedGoal != null && stuckCount < 3) {
                boolean clearedForHighPriority = tryPreemptivePathClearing(
                        fullPlan, currentState, level, numAgents, highestPriorityFailedGoal);
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
                    continue; // Retry the main loop with cleared path
                }
            }

            if (!madeProgress) {
                stuckCount++;
                // Log which subgoals are failing
                if (stuckCount == 1 || stuckCount % SearchConfig.STUCK_LOG_INTERVAL == 0) {
                    logNormal(getName() + ": Stuck iteration " + stuckCount +
                            ", remaining subgoals:");
                    for (Subgoal sg : unsatisfied) {
                        int diff = estimateSubgoalDifficulty(sg, currentState, level);
                        if (sg.isAgentGoal) {
                            logNormal("  - Agent " + sg.agentId + " -> goal position " +
                                    sg.goalPos + " (difficulty: " + diff + ")");
                        } else {
                            logNormal("  - Agent " + sg.agentId + " box " + sg.boxType +
                                    " -> goal " + sg.goalPos + " (difficulty: " + diff + ")");
                        }
                    }
                }

                // IMPROVEMENT 1: Try random re-ordering of subgoals (Subgoal Serialization)
                // Theory: slides05 - when stuck, the issue might be wrong execution order
                // CRITICAL: Only reorder within HIGHEST DEPTH goals to avoid blocking
                boolean foundWithReorder = false;
                for (int reorderAttempt = 0; reorderAttempt < SearchConfig.MAX_REORDER_ATTEMPTS; reorderAttempt++) {
                    Collections.shuffle(highestDepthGoals, random);

                    for (Subgoal subgoal : highestDepthGoals) {
                        List<Action> path = null;

                        if (subgoal.isAgentGoal) {
                            path = searchForAgentGoal(subgoal.agentId, subgoal.goalPos, currentState, level);
                        } else {
                            Position boxToMove = findBestBoxForGoal(subgoal, currentState, level);
                            if (boxToMove == null)
                                continue;
                            path = searchForSubgoal(subgoal.agentId, boxToMove,
                                    subgoal.goalPos, subgoal.boxType, currentState, level);
                        }

                        if (path != null && !path.isEmpty()) {
                            // Execute with plan merging
                            for (Action action : path) {
                                Action[] jointAction = createJointActionWithMerging(
                                        subgoal.agentId, action, currentState, level, numAgents, subgoal.isAgentGoal);
                                // Resolve conflicts - primary agent has right of way
                                jointAction = resolveConflicts(jointAction, currentState, level, subgoal.agentId);

                                // SAFETY CHECK: Verify primary agent's action wasn't corrupted
                                if (jointAction[subgoal.agentId].type != action.type) {
                                    logNormal("[WARNING] Primary agent " + subgoal.agentId +
                                            "'s action was modified from " + action + " to " +
                                            jointAction[subgoal.agentId] + " - this should not happen!");
                                }

                                fullPlan.add(jointAction);
                                currentState = applyJointAction(jointAction, currentState, level, numAgents);
                            }
                            // CRITICAL FIX: Validate goal actually reached (same as main path)
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
                                    continue;  // Try next subgoal in reorder
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

                            // Only mark success if goal was actually reached
                            if (goalReached) {
                                foundWithReorder = true;
                                stuckCount = 0;
                                clearYieldingForBeneficiary(subgoal.agentId);
                                
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
                    // NEW: Try clearing blocking agents before giving up
                    boolean clearedPath = false;
                    if (stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING) {
                        int planSizeBefore = fullPlan.size();
                        clearedPath = tryIdleAgentClearing(fullPlan, currentState, level,
                                numAgents, highestDepthGoals); // Use highestDepthGoals, not all
                        if (clearedPath) {
                            // Apply all new actions to update currentState
                            for (int i = planSizeBefore; i < fullPlan.size(); i++) {
                                currentState = applyJointAction(fullPlan.get(i), currentState, level, numAgents);
                            }
                            stuckCount = 0;
                            logNormal(getName() + ": Cleared blocking agent path");
                        }
                    }
                    
                    // NEW: Try DEADLOCK BOX DISPLACEMENT if agent clearing didn't help
                    // This handles cases where BOXES (not agents) are blocking each other
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
                                List<Action> displacePath = planBoxDisplacement(
                                    displacementPlan.agentId, displacementPlan.boxPosition, 
                                    displacementPlan.tempPosition, displacementPlan.boxType,
                                    currentState, level);
                                
                                if (displacePath != null && !displacePath.isEmpty()) {
                                    logNormal("[DEADLOCK] *** SUCCESS! Displacing box " + displacementPlan.boxType +
                                        " with " + displacePath.size() + " steps ***");
                                    
                                    // Execute displacement plan
                                    for (Action action : displacePath) {
                                        Action[] jointAction = createJointActionWithMerging(
                                            displacementPlan.agentId, action, currentState, level, 
                                            numAgents, false);
                                        jointAction = resolveConflicts(jointAction, currentState, level, 
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
                        boolean anyMove = tryGreedyStepWithMerging(fullPlan, currentState, level, numAgents);
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

    /**
     * Gets all unsatisfied subgoals (box goals not yet achieved).
     * Filters out pre-satisfied static goals (decorative elements like GROUP TWENTY).
     * 
     * CRITICAL FIX: Also returns agent goals for agents that have completed their box tasks,
     * even if other agents still have box goals. This prevents agents from blocking each other
     * while waiting for Phase 2 to start.
     */
    private List<Subgoal> getUnsatisfiedSubgoals(State state, Level level) {
        // Delegate to SubgoalManager (SRP: single responsibility for subgoal identification)
        return subgoalManager.getUnsatisfiedSubgoals(state, level, immovableDetector);
    }

    /**
     * Estimates difficulty of a subgoal using distance considering immovable boxes.
     * Returns Integer.MAX_VALUE if agent cannot reach any box of the required type.
     */
    private int estimateSubgoalDifficulty(Subgoal subgoal, State state, Level level) {
        // Delegate to SubgoalManager (SRP: single responsibility for difficulty estimation)
        return subgoalManager.estimateSubgoalDifficulty(subgoal, state, level, immovableDetector);
    }

    /**
     * Calculates distance between two positions using true distance heuristic if available.
     */
    private int getDistance(Position from, Position to, Level level) {
        if (heuristic instanceof TrueDistanceHeuristic) {
            int dist = ((TrueDistanceHeuristic) heuristic).getDistance(from, to);
            if (dist < Integer.MAX_VALUE) {
                return dist;
            }
        }
        // Fallback to Manhattan distance
        return from.manhattanDistance(to);
    }

    /**
     * LAYER 2: Incremental State Hardening
     * Computes the set of positions where boxes are already at satisfied goals.
     * These positions should be treated as WALLS during search - the boxes should
     * not be moved.
     * This prevents later subgoals from disrupting already-completed goals.
     */
    /**
     * Computes the set of positions where boxes are already at satisfied goals.
     */
    private Set<Position> computeSatisfiedGoalPositions(State state, Level level) {
        // Delegate to GoalChecker (SRP: single responsibility for goal-related utilities)
        return GoalChecker.computeSatisfiedGoalPositions(state, level);
    }

    /**
     * Checks if an action would disturb a satisfied goal position.
     * This implements LAYER 2: Incremental State Hardening.
     * 
     * For PUSH actions: The box being pushed must NOT be at a satisfied goal.
     * For PULL actions: The box being pulled must NOT be at a satisfied goal.
     * 
     * @param action         The action to check
     * @param agentId        The agent performing the action
     * @param state          The current state
     * @param satisfiedGoals Set of positions with satisfied goals (boxes that
     *                       should not move)
     * @return true if the action would disturb a satisfied goal
     */
    private boolean wouldDisturbSatisfiedGoal(Action action, int agentId, State state, Set<Position> satisfiedGoals) {
        if (satisfiedGoals.isEmpty()) {
            return false;
        }

        Position agentPos = state.getAgentPosition(agentId);
        if (agentPos == null) {
            return false;
        }

        switch (action.type) {
            case PUSH: {
                // The box is in front of the agent (in agentDir)
                Position boxPos = agentPos.move(action.agentDir);
                return satisfiedGoals.contains(boxPos);
            }
            case PULL: {
                // The box is behind the agent (opposite of boxDir)
                Position boxPos = agentPos.move(action.boxDir.opposite());
                return satisfiedGoals.contains(boxPos);
            }
            default:
                return false;
        }
    }

    // ========== Cycle Breaking via Temporary Displacement ==========
    
    /**
     * Attempts to break a cyclic dependency by temporarily displacing one box.
     * 
     * Strategy:
     * 1. Select a "victim" from the cycle - an agent whose box can be temporarily moved
     * 2. Find a safe position (not blocking anyone's path)
     * 3. Plan to move the box to the safe position
     * 4. This breaks the cycle, allowing other agents to proceed
     * 
     * @param state Current state
     * @param level Level information
     * @param cycles Detected cycles (list of agent IDs in each cycle)
     * @param numAgents Number of agents
     * @param remainingTimeMs Time remaining for planning
     * @return Plan to displace the box, or null if not possible
     */
    private List<Action[]> attemptCycleBreaking(State state, Level level, 
            List<List<Integer>> cycles, int numAgents, long remainingTimeMs) {
        
        if (cycles.isEmpty()) return null;
        
        long startTime = System.currentTimeMillis();
        long timeLimit = Math.min(remainingTimeMs, 10000); // Max 10 seconds for displacement
        
        // Take the first cycle
        List<Integer> cycle = cycles.get(0);
        System.err.println("[DISPLACEMENT] Trying to break cycle with " + cycle.size() + " agents...");
        
        // Prioritize cycle members first, then try a few nearby agents
        Set<Integer> agentsToTry = new LinkedHashSet<>(cycle);
        // Only add up to 3 more agents to avoid explosion
        int addedCount = 0;
        for (int i = 0; i < state.getNumAgents() && addedCount < 3; i++) {
            if (!agentsToTry.contains(i)) {
                agentsToTry.add(i);
                addedCount++;
            }
        }
        
        // Try each agent as a potential victim
        int attemptCount = 0;
        for (int victimAgentId : agentsToTry) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > timeLimit) {
                System.err.println("[DISPLACEMENT] Timeout after checking " + attemptCount + " agents");
                break;
            }
            
            attemptCount++;
            Position agentPos = state.getAgentPosition(victimAgentId);
            Color agentColor = level.getAgentColor(victimAgentId);
            
            // Find boxes this agent can push - try only closest ones
            List<Map.Entry<Position, Character>> candidateBoxes = new ArrayList<>();
            for (Map.Entry<Position, Character> boxEntry : state.getBoxes().entrySet()) {
                Position boxPos = boxEntry.getKey();
                char boxType = boxEntry.getValue();
                
                // Check if this agent can push this box
                if (level.getBoxColor(boxType) != agentColor) continue;
                
                // Check distance - only nearby boxes
                int dist = Math.abs(boxPos.row - agentPos.row) + Math.abs(boxPos.col - agentPos.col);
                if (dist <= 8) { // Tighter limit
                    candidateBoxes.add(boxEntry);
                }
            }
            
            // Sort by distance, try closest first
            candidateBoxes.sort((a, b) -> {
                int distA = Math.abs(a.getKey().row - agentPos.row) + Math.abs(a.getKey().col - agentPos.col);
                int distB = Math.abs(b.getKey().row - agentPos.row) + Math.abs(b.getKey().col - agentPos.col);
                return Integer.compare(distA, distB);
            });
            
            // Try only top 2 boxes per agent
            int boxTried = 0;
            for (Map.Entry<Position, Character> boxEntry : candidateBoxes) {
                if (boxTried >= 2) break;
                boxTried++;
                
                Position boxPos = boxEntry.getKey();
                char boxType = boxEntry.getValue();
                
                // Check if we've already tried to displace this box
                String boxKey = boxType + "@" + boxPos;
                if (displacementHistory.contains(boxKey)) {
                    continue; // Skip - already tried this box
                }
                
                // Find a safe position to displace this box
                Position safePos = findSafeDisplacementPosition(boxPos, state, level, cycle);
                if (safePos == null) continue;
                
                // Plan to move this box to the safe position
                long searchTimeLimit = timeLimit - (System.currentTimeMillis() - startTime);
                List<Action> path = searchForDisplacement(victimAgentId, boxPos, safePos, boxType, state, level, searchTimeLimit);
                
                if (path != null && !path.isEmpty()) {
                    System.err.println("[DISPLACEMENT] *** SUCCESS! Moving box " + boxType + 
                        " from " + boxPos + " to " + safePos + " (" + path.size() + " steps) ***");
                    
                    // Record this displacement to avoid repeating
                    displacementHistory.add(boxKey);
                    
                    // Convert single-agent path to joint actions
                    List<Action[]> jointPlan = new ArrayList<>();
                    State tempState = state;
                    for (Action action : path) {
                        Action[] jointAction = new Action[numAgents];
                        Arrays.fill(jointAction, Action.noOp());
                        jointAction[victimAgentId] = action;
                        jointPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    }
                    return jointPlan;
                }
            }
        }
        
        System.err.println("[DISPLACEMENT] Could not find any displacement opportunity");
        return null;
    }
    
    /**
     * Find a safe position to temporarily place a box.
     * Safe position requirements:
     * 1. Not a wall
     * 2. Not currently occupied
     * 3. Not on any goal position
     * 4. Not blocking the direct path of agents in the cycle
     * 5. Ideally in a "dead-end" or corner where it won't interfere
     */
    private Position findSafeDisplacementPosition(Position boxPos, State state, Level level, 
            List<Integer> cycleAgents) {
        
        // Collect positions to avoid
        Set<Position> avoid = new HashSet<>();
        
        // Avoid all current agent and box positions
        for (int i = 0; i < state.getNumAgents(); i++) {
            avoid.add(state.getAgentPosition(i));
        }
        for (Position pos : state.getBoxes().keySet()) {
            avoid.add(pos);
        }
        
        // Avoid all goal positions
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getBoxGoal(row, col) != 0 || level.getAgentGoal(row, col) >= 0) {
                    avoid.add(new Position(row, col));
                }
            }
        }
        
        // BFS to find nearest safe position - LIMIT SEARCH
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        // Start from positions adjacent to the box
        for (Direction dir : Direction.values()) {
            Position adj = boxPos.move(dir);
            if (!level.isWall(adj) && !avoid.contains(adj)) {
                queue.add(adj);
                visited.add(adj);
            }
        }
        
        int searchLimit = 50; // Drastically reduced from 200
        int explored = 0;
        
        while (!queue.isEmpty() && explored < searchLimit) {
            Position current = queue.poll();
            explored++;
            
            // Check if this is a valid safe position
            if (!avoid.contains(current) && !level.isWall(current)) {
                // Prefer positions with fewer neighbors (corners/dead-ends)
                int openNeighbors = 0;
                for (Direction dir : Direction.values()) {
                    Position neighbor = current.move(dir);
                    if (!level.isWall(neighbor)) openNeighbors++;
                }
                
                // Accept positions with 1-3 open neighbors
                if (openNeighbors <= 3) {
                    // Quick check: not too close to cycle agents
                    boolean tooClose = false;
                    for (int agentId : cycleAgents) {
                        Position agentPos = state.getAgentPosition(agentId);
                        if (Math.abs(current.row - agentPos.row) <= 1 && 
                            Math.abs(current.col - agentPos.col) <= 1) {
                            tooClose = true;
                            break;
                        }
                    }
                    
                    if (!tooClose) {
                        return current;
                    }
                }
            }
            
            // Expand neighbors
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }
        
        // Fallback: return first valid position we found
        for (Position pos : visited) {
            if (!avoid.contains(pos) && !level.isWall(pos)) {
                return pos;
            }
        }
        
        return null;
    }
    
    /**
     * Search for a path to displace a box to a safe position.
     * Similar to searchForSubgoal but without goal protection (we're explicitly moving things out of the way).
     */
    private List<Action> searchForDisplacement(int agentId, Position boxStart, Position targetPos,
            char boxType, State initialState, Level level, long timeLimitMs) {
        
        long startTime = System.currentTimeMillis();
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        int h = getDistance(boxStart, targetPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart);
        StateKey startKey = new StateKey(initialState, agentId, boxStart);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;
        int maxExplore = 2000; // Reduced from 5000

        while (!openList.isEmpty() && exploredCount < maxExplore) {
            // Check timeout every 100 nodes to avoid overhead
            if (exploredCount % 100 == 0 && System.currentTimeMillis() - startTime > timeLimitMs) {
                return null;
            }
            
            SearchNode current = openList.poll();
            exploredCount++;

            // Check if the box reached the target position
            Character boxAtTarget = current.state.getBoxes().get(targetPos);
            if (boxAtTarget != null && boxAtTarget == boxType) {
                return reconstructPath(current);
            }

            // Expand node
            for (Action action : getAllActions()) {
                if (action.type == Action.ActionType.NOOP) continue;

                if (!current.state.isApplicable(action, agentId, level)) continue;

                State newState = current.state.apply(action, agentId);

                // Track current box position
                Position newBoxPos = current.targetBoxPos;
                if (action.type == Action.ActionType.PUSH || action.type == Action.ActionType.PULL) {
                    // Find where the box moved
                    Position agentPos = current.state.getAgentPosition(agentId);
                    Position oldBoxPos;
                    if (action.type == Action.ActionType.PUSH) {
                        oldBoxPos = agentPos.move(action.agentDir);
                        newBoxPos = oldBoxPos.move(action.boxDir);
                    } else {
                        oldBoxPos = agentPos.move(action.boxDir.opposite());
                        newBoxPos = agentPos;
                    }
                    
                    // Only track our target box
                    Character movedBox = current.state.getBoxes().get(oldBoxPos);
                    if (movedBox == null || movedBox != boxType) {
                        newBoxPos = current.targetBoxPos;
                    }
                }

                StateKey newKey = new StateKey(newState, agentId, newBoxPos);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) continue;

                bestG.put(newKey, newG);

                int newH = getDistance(newBoxPos, targetPos, level);
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newBoxPos);
                openList.add(newNode);
            }
        }

        return null; // No path found
    }

    /**
     * A* search to move a specific box to a goal position.
     * Uses optimized StateKey that only tracks agent and target box positions.
     * 
     * LAYER 2 ENHANCEMENT: Protected goals are treated as immovable.
     * Boxes at satisfied goal positions will NOT be pushed/pulled during search.
     */
    private List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level) {
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        // LAYER 2: Compute satisfied goals at search start - these are FROZEN
        Set<Position> frozenGoals = computeSatisfiedGoalPositions(initialState, level);
        if (!frozenGoals.isEmpty()) {
            logVerbose("[LAYER2] Protecting " + frozenGoals.size() + " satisfied goals during search for " +
                    boxType + " -> " + goalPos);
        }

        int h = getDistance(boxStart, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart);
        StateKey startKey = new StateKey(initialState, agentId, boxStart);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;

        while (!openList.isEmpty() && exploredCount < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            SearchNode current = openList.poll();
            exploredCount++;

            // Check if goal is achieved: the goal position has the correct box type
            Character boxAtGoal = current.state.getBoxes().get(goalPos);
            if (boxAtGoal != null && boxAtGoal == boxType) {
                return reconstructPath(current);
            }

            // Expand node
            for (Action action : getAllActions()) {
                if (action.type == Action.ActionType.NOOP) {
                    continue;
                }

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                // LAYER 2: Skip actions that would disturb satisfied goals (frozen boxes)
                if (wouldDisturbSatisfiedGoal(action, agentId, current.state, frozenGoals)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);

                // Track target box position for optimized StateKey
                Position newTargetBoxPos = findTargetBoxPosition(newState, boxType, current.targetBoxPos);
                StateKey newKey = new StateKey(newState, agentId, newTargetBoxPos);
                int newG = current.g + 1;

                // Skip if we've seen this state with lower cost
                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                // Compute heuristic: distance of target box to goal
                int newH = (newTargetBoxPos != null) ? getDistance(newTargetBoxPos, goalPos, level) : 0;

                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newTargetBoxPos);
                openList.add(newNode);
            }
        }

        return null; // No path found
    }

    /**
     * Plans a path to displace a blocking box to a temporary parking position.
     * Similar to searchForSubgoal but doesn't protect other goals as strictly.
     * Used by DeadlockResolver to break blocking cycles.
     * 
     * @param agentId Agent that will push the box
     * @param boxPos Current position of the blocking box
     * @param targetPos Target parking position for the box
     * @param boxType Type of the box
     * @param initialState Current state
     * @param level Level information
     * @return List of actions to displace the box, or null if not possible
     */
    private List<Action> planBoxDisplacement(int agentId, Position boxPos, Position targetPos,
            char boxType, State initialState, Level level) {
        
        // Check if box is actually at the specified position
        Character actualBox = initialState.getBoxAt(boxPos);
        if (actualBox == null || actualBox != boxType) {
            logVerbose("[DISPLACEMENT] Box " + boxType + " not found at " + boxPos);
            return null;
        }

        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        // Less strict goal protection - only protect goals that are ALREADY satisfied
        Set<Position> frozenGoals = new HashSet<>();
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position goalPos = new Position(row, col);
                    char currentBox = initialState.getBoxAt(goalPos);
                    if (currentBox == goalType) {
                        frozenGoals.add(goalPos);
                    }
                }
            }
        }
        
        // IMPORTANT: Remove target position from frozen goals (we're explicitly moving there)
        frozenGoals.remove(targetPos);
        // Also remove the box's current position (we're explicitly moving it)
        frozenGoals.remove(boxPos);

        logVerbose("[DISPLACEMENT] Planning displacement of " + boxType + " from " + boxPos + 
            " to " + targetPos + ", protecting " + frozenGoals.size() + " satisfied goals");

        int h = getDistance(boxPos, targetPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxPos);
        StateKey startKey = new StateKey(initialState, agentId, boxPos);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;
        int maxStates = SearchConfig.MAX_STATES_PER_SUBGOAL / 2; // Faster search for displacement

        while (!openList.isEmpty() && exploredCount < maxStates) {
            SearchNode current = openList.poll();
            exploredCount++;

            // Goal: box reaches target position
            Position currentBoxPos = findBoxPosition(current.state, boxType, current.targetBoxPos);
            if (currentBoxPos != null && currentBoxPos.equals(targetPos)) {
                logVerbose("[DISPLACEMENT] Found path in " + exploredCount + " states");
                return reconstructPath(current);
            }

            // Expand node
            for (Action action : getAllActions()) {
                if (action.type == Action.ActionType.NOOP) {
                    continue;
                }

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                // Only protect goals that are NOT our target
                if (wouldDisturbSatisfiedGoal(action, agentId, current.state, frozenGoals)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);

                Position newTargetBoxPos = findTargetBoxPosition(newState, boxType, current.targetBoxPos);
                StateKey newKey = new StateKey(newState, agentId, newTargetBoxPos);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                int newH = (newTargetBoxPos != null) ? getDistance(newTargetBoxPos, targetPos, level) : 0;
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newTargetBoxPos);
                openList.add(newNode);
            }
        }

        logVerbose("[DISPLACEMENT] No displacement path found in " + exploredCount + " states");
        return null;
    }

    /**
     * Finds the current position of a specific box type.
     */
    private Position findBoxPosition(State state, char boxType, Position hint) {
        // First check hint position
        if (hint != null) {
            Character boxAtHint = state.getBoxAt(hint);
            if (boxAtHint != null && boxAtHint == boxType) {
                return hint;
            }
        }
        
        // Search all positions
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                return entry.getKey();
            }
        }
        return null;
    }

    /**
     * A* search to move an agent to its goal position (no box involved).
     * This is Phase 2: Agent Goal Planning.
     */
    private List<Action> searchForAgentGoal(int agentId, Position goalPos,
            State initialState, Level level) {
        Position startPos = initialState.getAgentPosition(agentId);
        if (startPos.equals(goalPos)) {
            return Collections.emptyList(); // Already at goal
        }

        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<Position, Integer> bestG = new HashMap<>(); // Only track agent position for agent goals

        int h = getDistance(startPos, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, null);
        openList.add(startNode);
        bestG.put(startPos, 0);

        int exploredCount = 0;

        while (!openList.isEmpty() && exploredCount < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            SearchNode current = openList.poll();
            exploredCount++;

            // Check if agent reached goal
            Position currentAgentPos = current.state.getAgentPosition(agentId);
            if (currentAgentPos.equals(goalPos)) {
                return reconstructPath(current);
            }

            // Only try Move actions for agent goals
            for (Direction dir : Direction.values()) {
                Action action = Action.move(dir);

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);
                Position newAgentPos = newState.getAgentPosition(agentId);
                int newG = current.g + 1;

                // Skip if we've seen this position with lower cost
                Integer existingG = bestG.get(newAgentPos);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newAgentPos, newG);

                int newH = getDistance(newAgentPos, goalPos, level);
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, null);
                openList.add(newNode);
            }
        }

        return null; // No path found
    }

    /**
     * Computes heuristic for subgoal: minimum distance of any matching box to goal.
     */
    private int computeSubgoalHeuristic(State state, Position goalPos, char boxType, Level level) {
        // If goal already has correct box, distance is 0
        Character atGoal = state.getBoxes().get(goalPos);
        if (atGoal != null && atGoal == boxType) {
            return 0;
        }

        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                int dist = getDistance(entry.getKey(), goalPos, level);
                minDist = Math.min(minDist, dist);
            }
        }

        return minDist == Integer.MAX_VALUE ? 0 : minDist;
    }

    /**
     * Reconstructs path from goal node to start.
     */
    private List<Action> reconstructPath(SearchNode goalNode) {
        List<Action> path = new ArrayList<>();
        SearchNode current = goalNode;

        while (current.parent != null) {
            path.add(current.action);
            current = current.parent;
        }

        Collections.reverse(path);
        return path;
    }

    /**
     * Resolves conflicts in joint action by making conflicting agents wait.
     * CRITICAL: The primary agent (currently executing a task) is NEVER made to wait.
     * Other agents must yield to the primary agent.
     *
     * @param jointAction The joint action to resolve conflicts for
     * @param state Current state
     * @param level The level
     * @param primaryAgentId The agent currently executing a task (has right of way), or -1 if none
     * @return Joint action with conflicts resolved
     */
    private Action[] resolveConflicts(Action[] jointAction, State state, Level level, int primaryAgentId) {
        // Delegate to ConflictResolver (SRP: single responsibility for conflict resolution)
        return conflictResolver.resolveConflicts(jointAction, state, level, primaryAgentId);
    }

    /**
     * Resolves conflicts in joint action by making conflicting agents wait.
     * Uses standard "higher ID waits" rule when no primary agent is specified.
     */
    private Action[] resolveConflicts(Action[] jointAction, State state, Level level) {
        // Delegate to ConflictResolver (SRP: single responsibility for conflict resolution)
        return conflictResolver.resolveConflicts(jointAction, state, level);
    }

    /**
     * Tries to make a greedy single step when stuck.
     * Uses heuristic to guide exploration.
     */
    private boolean tryGreedyStep(List<Action[]> plan, State state, Level level, int numAgents) {
        // Try each agent
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (isAgentGoalSatisfied(agentId, state, level)) {
                continue;
            }

            // Skip agents that are currently yielding - they must stay parked
            if (yieldingManager.isYielding(agentId)) {
                continue;
            }

            Action bestAction = findBestGreedyAction(agentId, state, level);

            if (bestAction != null && bestAction.type != Action.ActionType.NOOP) {
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[agentId] = bestAction;

                jointAction = resolveConflicts(jointAction, state, level);
                plan.add(jointAction);
                return true;
            }
        }

        return false;
    }

    /**
     * Finds the best greedy action for an agent using the heuristic.
     * LAYER 2: Respects satisfied goal protection.
     */
    private Action findBestGreedyAction(int agentId, State state, Level level) {
        Action bestAction = Action.noOp();
        int bestH = estimateAgentCost(agentId, state, level);

        // LAYER 2: Compute satisfied goals for protection
        Set<Position> frozenGoals = computeSatisfiedGoalPositions(state, level);

        for (Action action : getAllActions()) {
            if (action.type == Action.ActionType.NOOP) {
                continue;
            }

            if (!state.isApplicable(action, agentId, level)) {
                continue;
            }

            // LAYER 2: Skip actions that would disturb satisfied goals
            if (wouldDisturbSatisfiedGoal(action, agentId, state, frozenGoals)) {
                continue;
            }

            State newState = state.apply(action, agentId);
            int newH = estimateAgentCost(agentId, newState, level);

            if (newH < bestH) {
                bestH = newH;
                bestAction = action;
            }
        }

        return bestAction;
    }

    /**
     * Estimates total cost for an agent using true distance heuristic.
     * Includes both box goals and agent position goals.
     */
    private int estimateAgentCost(int agentId, State state, Level level) {
        int cost = 0;
        Color agentColor = level.getAgentColor(agentId);

        // Calculate box goal costs
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            char boxType = box.getValue();
            if (level.getBoxColor(boxType) == agentColor) {
                Position boxPos = box.getKey();

                // Skip if already at a satisfied goal
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
                            int dist = getDistance(boxPos, goalPos, level);
                            minDist = Math.min(minDist, dist);
                        }
                    }
                }

                if (minDist < Integer.MAX_VALUE) {
                    cost += minDist;
                }
            }
        }

        // Calculate agent position goal cost (if any)
        // CRITICAL: Only consider agent goal distance AFTER ALL box goals are
        // globally satisfied! This prevents Plan Merging from driving ANY agent
        // toward agent goals while other agents are still working on box goals.
        if (allBoxGoalsSatisfied(state, level)) {
            Position agentPos = state.getAgentPosition(agentId);

            // Add HUGE penalty if agent is in a position that blocks higher-priority agents
            Set<Position> blockingPositions = getPositionsThatWouldBlockHigherPriority(agentId, state, level);
            if (blockingPositions.contains(agentPos)) {
                cost += 10000; // Huge penalty to discourage this position
            }

            for (int row = 0; row < level.getRows(); row++) {
                for (int col = 0; col < level.getCols(); col++) {
                    if (level.getAgentGoal(row, col) == agentId) {
                        Position goalPos = new Position(row, col);
                        if (!agentPos.equals(goalPos)) {
                            cost += getDistance(agentPos, goalPos, level);
                        }
                        break;
                    }
                }
            }
        }

        return cost;
    }

    /**
     * Checks if an agent's goals are all satisfied.
     */
    private boolean isAgentGoalSatisfied(int agentId, State state, Level level) {
        // Delegate to GoalChecker (SRP: single responsibility for goal checking)
        return GoalChecker.isAgentGoalSatisfied(agentId, state, level);
    }

    /**
     * Checks if ALL box goals in the level are satisfied (regardless of agent color).
     */
    private boolean allBoxGoalsSatisfied(State state, Level level) {
        // Delegate to GoalChecker (SRP: single responsibility for goal checking)
        return GoalChecker.allBoxGoalsSatisfied(state, level);
    }

    /**
     * Finds the agent that can move boxes of a given color.
     */
    private int findAgentForColor(Color color, Level level, int numAgents) {
        // Delegate to GoalChecker (SRP: single responsibility for goal-related utilities)
        return GoalChecker.findAgentForColor(color, level, numAgents);
    }

    /**
     * Generates all possible actions (cached for performance).
     */
    private List<Action> getAllActions() {
        // Delegate to PlanningUtils (SRP: single responsibility for planning utilities)
        return PlanningUtils.getAllActions();
    }

    // ========== Helper Classes ==========

    /**
     * Represents a subgoal: moving a box type to a specific goal position,
     * or moving an agent to its goal position.
     */
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

        // Backward compatibility constructor for box goals
        public Subgoal(int agentId, char boxType, Position goalPos) {
            this(agentId, boxType, goalPos, false);
        }
    }

    /**
     * State key for duplicate detection in search.
     * 
     * OPTIMIZATION based on IW theory (slides05):
     * Only track the agent position and target box position.
     * Other boxes are treated as static obstacles during the search.
     * This dramatically reduces state space while maintaining correctness.
     */
    private static class StateKey {
        final Position agentPos;
        final Position targetBoxPos;

        StateKey(State state, int agentId, Position targetBoxPos) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (!(obj instanceof StateKey))
                return false;
            StateKey other = (StateKey) obj;
            return agentPos.equals(other.agentPos) &&
                    Objects.equals(targetBoxPos, other.targetBoxPos);
        }

        @Override
        public int hashCode() {
            return Objects.hash(agentPos, targetBoxPos);
        }
    }

    /**
     * Search node for A* algorithm.
     * Includes target box position for optimized state tracking.
     */
    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action action;
        final int g;
        final int f;
        final Position targetBoxPos; // Track the box we're moving

        SearchNode(State state, SearchNode parent, Action action, int g, int h, Position targetBoxPos) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.g = g;
            this.f = g + h;
            this.targetBoxPos = targetBoxPos;
        }

        @Override
        public int compareTo(SearchNode other) {
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0)
                return fCompare;
            // Tie-break: prefer deeper nodes (higher g)
            return Integer.compare(other.g, this.g);
        }
    }

    // ========== New Helper Methods ==========

    /**
     * Finds the position of the target box after an action.
     * Used for optimized StateKey tracking.
     */
    private Position findTargetBoxPosition(State state, char boxType, Position lastKnownPos) {
        // First check if box is still at last known position
        Character boxAtLast = state.getBoxes().get(lastKnownPos);
        if (boxAtLast != null && boxAtLast == boxType) {
            return lastKnownPos;
        }

        // Box was moved, find its new position (should be adjacent to last position)
        for (Direction dir : Direction.values()) {
            Position newPos = lastKnownPos.move(dir);
            Character boxAtNew = state.getBoxes().get(newPos);
            if (boxAtNew != null && boxAtNew == boxType) {
                return newPos;
            }
        }

        // Fallback: search all boxes (should rarely happen)
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                return entry.getKey();
            }
        }

        return null;
    }

    /**
     * Creates a joint action with plan merging - other agents can act if they don't
     * conflict.
     * Based on slides06: Plan Merging / Post-Processing.
     */
    private Action[] createJointActionWithMerging(int primaryAgentId, Action primaryAction,
            State state, Level level, int numAgents, boolean isAgentGoal) {
        Action[] jointAction = new Action[numAgents];
        Arrays.fill(jointAction, Action.noOp());
        jointAction[primaryAgentId] = primaryAction;

        // CRITICAL: Skip plan merging if primary agent is executing an agent goal
        // This prevents other agents from creating dynamic obstacles that invalidate the planned path
        if (isAgentGoal) {
            logVerbose("[NO MERGE] Skipping plan merging for agent goal execution");
            return jointAction;
        }

        // Try to let other agents make progress too (only for box goals)
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgentId)
                continue;
            if (isAgentGoalSatisfied(agentId, state, level))
                continue;

            // YIELDING CHECK: Don't let yielding agents move during plan merging
            if (yieldingManager.isYielding(agentId)) {
                logVerbose("[SKIP] Agent " + agentId + " is YIELDING, skipping in plan merge");
                continue;
            }

            // COMPLETED AGENT CHECK: Don't move agents that have completed their task
            // and successfully yielded to a safe position. They should stay parked!
            if (yieldingManager.hasCompletedTask(agentId)) {
                Position agentPos = state.getAgentPosition(agentId);
                int passableNeighbors = countPassableNeighbors(agentPos, state, level, agentId);
                // Only protect if agent is in a safe position (not a corridor)
                if (passableNeighbors >= 3 || countFreeNeighbors(agentPos, level) == 1) {
                    logVerbose("[SKIP] Agent " + agentId + " has completed task and is in safe position, skipping");
                    continue;
                }
            }

            // GLOBAL BOX GOAL CHECK: Don't let ANY agent move toward agent goals
            // during plan merging until ALL box goals are globally satisfied.
            // This respects the Phase 1 (Box Goals) / Phase 2 (Agent Goals) separation
            // and ensures Agent Goal topological ordering takes effect properly.
            if (!allBoxGoalsSatisfied(state, level)) {
                continue;
            }

            // AGENT GOAL DEPENDENCY CHECK: Don't let agents move toward their goals
            // if doing so would block a higher-priority agent's path.
            // Higher priority = deeper topological depth (should complete first).
            if (wouldBlockHigherPriorityAgentGoal(agentId, state, level)) {
                logVerbose("[SKIP] Agent " + agentId + " would block higher-priority agent goal, skipping");
                continue;
            }

            // Find a useful action for this agent
            Action bestAction = findBestGreedyAction(agentId, state, level);
            if (bestAction != null && bestAction.type != Action.ActionType.NOOP) {
                // Temporarily add action and check for conflicts
                jointAction[agentId] = bestAction;
                List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(state, jointAction, level);

                // If this agent's action causes conflict, revert to NoOp
                boolean hasConflict = false;
                for (ConflictDetector.Conflict conflict : conflicts) {
                    if (conflict.agent1 == agentId || conflict.agent2 == agentId) {
                        hasConflict = true;
                        break;
                    }
                }

                if (hasConflict) {
                    jointAction[agentId] = Action.noOp();
                }
            }
        }

        return jointAction;
    }

    /**
     * Applies a joint action to the state and returns the new state.
     */
    private State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        State newState = state;
        for (int a = 0; a < numAgents; a++) {
            if (jointAction[a].type != Action.ActionType.NOOP) {
                if (newState.isApplicable(jointAction[a], a, level)) {
                    newState = newState.apply(jointAction[a], a);
                }
            }
        }
        return newState;
    }

    /**
     * Tries to make a greedy step with plan merging.
     */
    private boolean tryGreedyStepWithMerging(List<Action[]> plan, State state, Level level, int numAgents) {
        // Find the first agent that can make a useful move
        int primaryAgent = -1;
        Action primaryAction = null;

        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (isAgentGoalSatisfied(agentId, state, level))
                continue;

            // Skip agents that are currently yielding - they must stay parked
            if (yieldingManager.isYielding(agentId)) {
                logVerbose("[GREEDY] Skipping Agent " + agentId + " - currently YIELDING for Agent "
                        + yieldingManager.getBeneficiary(agentId));
                continue;
            }

            Action bestAction = findBestGreedyAction(agentId, state, level);
            if (bestAction != null && bestAction.type != Action.ActionType.NOOP) {
                primaryAgent = agentId;
                primaryAction = bestAction;
                break;
            }
        }

        if (primaryAgent == -1) {
            // No agent can make progress, try random move to break deadlock
            // IMPORTANT: Only use agents that have incomplete goals
            for (int agentId = 0; agentId < numAgents; agentId++) {
                // Skip agents that have completed all their goals
                if (isAgentGoalSatisfied(agentId, state, level))
                    continue;

                // Skip agents that are currently yielding - they must stay parked
                if (yieldingManager.isYielding(agentId))
                    continue;

                for (Action action : getAllActions()) {
                    if (action.type == Action.ActionType.MOVE &&
                            state.isApplicable(action, agentId, level)) {
                        primaryAgent = agentId;
                        primaryAction = action;
                        break;
                    }
                }
                if (primaryAgent != -1)
                    break;
            }
        }

        if (primaryAgent != -1 && primaryAction != null) {
            // Check if we're in agent goal phase
            boolean isAgentGoalPhase = allBoxGoalsSatisfied(state, level);
            Action[] jointAction = createJointActionWithMerging(
                    primaryAgent, primaryAction, state, level, numAgents, isAgentGoalPhase);
            jointAction = resolveConflicts(jointAction, state, level);
            plan.add(jointAction);
            return true;
        }

        return false;
    }

    // ========== Idle Agent Clearing System ==========

    /**
     * Attempts to clear blocking agents by moving them to parking positions.
     * This handles the case where completed agents block paths of other agents.
     * 
     * Algorithm:
     * 1. For each stuck subgoal, find path requirements (positions needed)
     * 2. Identify completed agents that are blocking these positions
     * 3. Find safe parking positions for blocking agents
     * 4. Plan and execute clearing moves
     */
    private boolean tryIdleAgentClearing(List<Action[]> plan, State state, Level level,
            int numAgents, List<Subgoal> stuckSubgoals) {
        // Find agents that have completed their box tasks
        Set<Integer> completedAgents = findCompletedAgents(state, level, numAgents);

        if (completedAgents.isEmpty()) {
            return false; // No idle agents to clear
        }

        // For each stuck subgoal, check if a completed agent is blocking
        for (Subgoal subgoal : stuckSubgoals) {
            Set<Position> criticalPositions;

            if (subgoal.isAgentGoal) {
                // Agent goal: find critical positions for agent to reach its goal
                criticalPositions = findCriticalPositionsForAgentGoal(
                        state.getAgentPosition(subgoal.agentId), subgoal.goalPos, level);
            } else {
                // Box goal: find critical positions for box movement
                Position boxPos = findBestBoxForGoal(subgoal, state, level);
                if (boxPos == null)
                    continue;
                criticalPositions = findCriticalPositions(
                        state.getAgentPosition(subgoal.agentId), boxPos, subgoal.goalPos, level);
            }

            // Check which completed agents are blocking
            for (int blockingAgentId : completedAgents) {
                if (blockingAgentId == subgoal.agentId)
                    continue;

                Position blockingPos = state.getAgentPosition(blockingAgentId);
                if (criticalPositions.contains(blockingPos)) {
                    logVerbose("[CLEARING] Agent " + blockingAgentId + " at " + blockingPos +
                            " blocking " + (subgoal.isAgentGoal ? "Agent" + subgoal.agentId : "box") +
                            " path to " + subgoal.goalPos);

                    // CRITICAL: Set yielding state FIRST, BEFORE attempting to clear.
                    // This ensures the blocking agent cannot return to its goal position
                    // even if the clearing attempt fails. This breaks the deadlock cycle.
                    setAgentYielding(blockingAgentId, subgoal.agentId);

                    // Try to move the blocking agent to parking position
                    boolean cleared = clearBlockingAgent(plan, state, level, numAgents,
                            blockingAgentId, criticalPositions);

                    if (cleared) {
                        return true;
                    } else {
                        // Clearing failed - but agent is still marked as YIELDING!
                        // Try a fallback: make the blocking agent take ANY valid move
                        // that gets it out of the critical path
                        logVerbose("[CLEARING] FAILED to find parking, trying random escape move");
                        boolean escaped = tryRandomEscapeMove(plan, state, level, numAgents,
                                blockingAgentId, criticalPositions);
                        if (escaped) {
                            return true;
                        }
                        // Even if this fails, the YIELDING state remains set,
                        // preventing the agent from returning to its goal
                    }
                }
            }
        }

        return false;
    }

    /**
     * Finds agents that have completed all their box tasks.
     */
    private Set<Integer> findCompletedAgents(State state, Level level, int numAgents) {
        Set<Integer> completed = new HashSet<>();

        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (hasCompletedBoxTasks(agentId, state, level)) {
                completed.add(agentId);
            }
        }

        return completed;
    }

    /**
     * Checks if an agent has completed all its box-moving tasks.
     */
    private boolean hasCompletedBoxTasks(int agentId, State state, Level level) {
        // Delegate to GoalChecker (SRP: single responsibility for goal-related utilities)
        return GoalChecker.hasCompletedBoxTasks(agentId, state, level);
    }

    /**
     * Finds critical positions that might be needed for moving a box to its goal.
     * Uses BFS to explore a corridor of positions.
     */
    private Set<Position> findCriticalPositions(Position agentPos, Position boxPos,
            Position goalPos, Level level) {
        Set<Position> critical = new HashSet<>();

        // Add direct path positions using BFS
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();

        // Start from box position towards goal
        queue.add(boxPos);
        visited.add(boxPos);
        critical.add(boxPos);

        while (!queue.isEmpty() && critical.size() < SearchConfig.MAX_PARKING_DISTANCE * 4) {
            Position current = queue.poll();

            if (current.equals(goalPos)) {
                continue; // Reached goal, stop expanding from here
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    critical.add(next);
                    queue.add(next);

                    // Stop if we've reached the goal area
                    if (next.manhattanDistance(goalPos) <= 1) {
                        break;
                    }
                }
            }
        }

        // Also add positions near the agent
        for (Direction dir : Direction.values()) {
            Position adjAgent = agentPos.move(dir);
            if (!level.isWall(adjAgent)) {
                critical.add(adjAgent);
            }
        }

        return critical;
    }

    /**
     * Attempts to move a blocking agent to a parking position.
     */
    private boolean clearBlockingAgent(List<Action[]> plan, State state, Level level,
            int numAgents, int blockingAgentId,
            Set<Position> forbiddenPositions) {
        Position currentPos = state.getAgentPosition(blockingAgentId);

        // Find a safe parking position
        Position parkingPos = findParkingPosition(currentPos, state, level,
                forbiddenPositions, blockingAgentId);

        if (parkingPos == null) {
            return false; // No parking position found
        }

        // Plan path from current position to parking position
        List<Action> clearingPath = planAgentPath(blockingAgentId, currentPos, parkingPos,
                state, level, forbiddenPositions);

        if (clearingPath == null || clearingPath.isEmpty()) {
            return false;
        }

        // Execute the clearing path
        State workingState = state;
        for (Action action : clearingPath) {
            Action[] jointAction = new Action[numAgents];
            Arrays.fill(jointAction, Action.noOp());
            jointAction[blockingAgentId] = action;

            // Note: Don't let other agents move during clearing to avoid conflicts
            // The clearing path was planned based on initial state, so adding other
            // agent moves could invalidate the path

            jointAction = resolveConflicts(jointAction, workingState, level);
            plan.add(jointAction);
            workingState = applyJointAction(jointAction, workingState, level, numAgents);
        }

        logVerbose(getName() + ": Moved Agent " + blockingAgentId +
                " from " + currentPos + " to " + parkingPos + " (clearing path)");

        return true;
    }

    /**
     * Fallback method when clearBlockingAgent fails.
     * Attempts to move the blocking agent just ONE step in any direction
     * that gets it out of the critical path.
     */
    private boolean tryRandomEscapeMove(List<Action[]> plan, State state, Level level,
            int numAgents, int agentId, Set<Position> criticalPositions) {
        Position currentPos = state.getAgentPosition(agentId);

        // Try each direction
        for (Direction dir : Direction.values()) {
            Position newPos = currentPos.move(dir);

            // Check if this position is valid and NOT in critical path
            if (!level.isWall(newPos) &&
                    !state.getBoxes().containsKey(newPos) &&
                    !criticalPositions.contains(newPos)) {

                // Check not occupied by another agent
                boolean occupied = false;
                for (int i = 0; i < numAgents; i++) {
                    if (state.getAgentPosition(i).equals(newPos)) {
                        occupied = true;
                        break;
                    }
                }

                if (!occupied) {
                    Action moveAction = Action.move(dir);
                    if (state.isApplicable(moveAction, agentId, level)) {
                        Action[] jointAction = new Action[numAgents];
                        Arrays.fill(jointAction, Action.noOp());
                        jointAction[agentId] = moveAction;
                        jointAction = resolveConflicts(jointAction, state, level);
                        plan.add(jointAction);

                        logVerbose("[ESCAPE] Agent " + agentId + " escaped " +
                                currentPos + " -> " + newPos);
                        return true;
                    }
                }
            }
        }

        logVerbose("[ESCAPE] Agent " + agentId + " could not find any escape direction");
        return false;
    }

    /**
     * Finds a safe parking position for a blocking agent.
     * Searches for positions that are:
     * 1. Not in the forbidden set (critical path positions)
     * 2. Not walls
     * 3. Not occupied by boxes or other agents
     */
    private Position findParkingPosition(Position start, State state, Level level,
            Set<Position> forbidden, int agentId) {
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();

        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            // Check if this is a valid parking position
            if (!current.equals(start) && isValidParkingPosition(current, state, level,
                    forbidden, agentId)) {
                return current;
            }

            // Expand neighbors (BFS to find closest valid position)
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!visited.contains(next) && !level.isWall(next) &&
                        visited.size() < SearchConfig.MAX_PARKING_DISTANCE * SearchConfig.MAX_PARKING_DISTANCE) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }

        return null; // No parking position found
    }

    /**
     * Checks if a position is valid for parking (not blocking, not occupied).
     */
    private boolean isValidParkingPosition(Position pos, State state, Level level,
            Set<Position> forbidden, int agentId) {
        // Not in forbidden positions
        if (forbidden.contains(pos)) {
            return false;
        }

        // Not a wall
        if (level.isWall(pos)) {
            return false;
        }

        // Not occupied by a box
        if (state.getBoxes().containsKey(pos)) {
            return false;
        }

        // Not occupied by another agent
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (i != agentId && state.getAgentPosition(i).equals(pos)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Plans a simple path for an agent to move from start to goal.
     * Uses BFS to find shortest path avoiding obstacles.
     */
    private List<Action> planAgentPath(int agentId, Position start, Position goal,
            State state, Level level, Set<Position> forbidden) {
        if (start.equals(goal)) {
            return Collections.emptyList();
        }

        // BFS to find path
        Queue<PathNode> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();

        queue.add(new PathNode(start, null, null));
        visited.add(start);

        int explored = 0;

        while (!queue.isEmpty() && explored < SearchConfig.MAX_STATES_PER_CLEARING) {
            PathNode current = queue.poll();
            explored++;

            if (current.position.equals(goal)) {
                // Reconstruct path
                List<Action> path = new ArrayList<>();
                PathNode node = current;
                while (node.parent != null) {
                    path.add(node.action);
                    node = node.parent;
                }
                Collections.reverse(path);
                return path;
            }

            // Try each direction
            for (Direction dir : Direction.values()) {
                Position next = current.position.move(dir);

                if (visited.contains(next))
                    continue;
                if (level.isWall(next))
                    continue;

                // Check if position is free (no box, no other agent)
                if (state.getBoxes().containsKey(next))
                    continue;

                boolean occupiedByOther = false;
                for (int i = 0; i < state.getNumAgents(); i++) {
                    if (i != agentId && state.getAgentPosition(i).equals(next)) {
                        occupiedByOther = true;
                        break;
                    }
                }
                if (occupiedByOther)
                    continue;

                visited.add(next);
                Action moveAction = Action.move(dir);
                queue.add(new PathNode(next, current, moveAction));
            }
        }

        return null; // No path found
    }

    /**
     * Adds moves for other completed agents that can help clear the path.
     * Only moves agents that have completed box tasks but are blocking critical
     * paths.
     */
    private Action[] addOtherAgentMoves(Action[] jointAction, State state, Level level,
            int numAgents, int primaryAgent,
            Set<Position> forbidden) {
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgent)
                continue;
            if (jointAction[agentId].type != Action.ActionType.NOOP)
                continue;

            // Skip if agent has completed ALL goals (no need to move at all)
            if (isAgentGoalSatisfied(agentId, state, level))
                continue;

            // Skip agents that are currently yielding - they must stay parked
            if (yieldingManager.isYielding(agentId))
                continue;

            // Only move agents that have completed their box tasks
            if (!hasCompletedBoxTasks(agentId, state, level))
                continue;

            Position agentPos = state.getAgentPosition(agentId);

            // If this agent is in forbidden positions, try to move it out
            if (forbidden.contains(agentPos)) {
                for (Direction dir : Direction.values()) {
                    Position newPos = agentPos.move(dir);
                    if (!level.isWall(newPos) && !state.getBoxes().containsKey(newPos) &&
                            !forbidden.contains(newPos)) {

                        Action moveAction = Action.move(dir);
                        if (state.isApplicable(moveAction, agentId, level)) {
                            jointAction[agentId] = moveAction;
                            break;
                        }
                    }
                }
            }
        }

        return jointAction;
    }

    /**
     * Simple path node for BFS pathfinding.
     */
    private static class PathNode {
        final Position position;
        final PathNode parent;
        final Action action;

        PathNode(Position position, PathNode parent, Action action) {
            this.position = position;
            this.parent = parent;
            this.action = action;
        }
    }

    // ========== Preemptive Path Clearing ==========

    /**
     * Attempts to clear the path for a high-priority goal BEFORE executing
     * lower-priority goals.
     * This is crucial for dead-end filling scenarios where:
     * 1. Agent A needs to push box B to deep goal G
     * 2. Agent A cannot reach box B because agent/box C is in the way
     * 3. Instead of letting C execute (which might block G permanently), we move C
     * out of A's path first
     * 
     * Algorithm:
     * 1. Find the path from agent to box (for box goals) or agent to goal (for
     * agent goals)
     * 2. Identify ALL blockers (agents and boxes) on this path
     * 3. For each blocker, find a parking position NOT on the critical path
     * 4. Move blockers to their parking positions
     */
    private boolean tryPreemptivePathClearing(List<Action[]> plan, State state, Level level,
            int numAgents, Subgoal blockedGoal) {
        Position agentPos = state.getAgentPosition(blockedGoal.agentId);
        Position targetPos;

        if (blockedGoal.isAgentGoal) {
            targetPos = blockedGoal.goalPos;
        } else {
            targetPos = findBestBoxForGoal(blockedGoal, state, level);
            if (targetPos == null)
                return false;
        }

        logVerbose("[PREEMPTIVE] Trying to clear path for Agent " + blockedGoal.agentId +
                " from " + agentPos + " to " + targetPos);

        // Find path ignoring dynamic obstacles (other agents/boxes) to determine what's
        // blocking
        Set<Position> neededPath = findPathIgnoringDynamicObstacles(agentPos, targetPos, level);
        if (neededPath.isEmpty()) {
            logVerbose("[PREEMPTIVE] No path exists even ignoring dynamic obstacles");
            return false;
        }

        // Also add the goal position and positions needed to push the box to goal
        if (!blockedGoal.isAgentGoal) {
            Set<Position> boxToGoalPath = findPathIgnoringDynamicObstacles(targetPos, blockedGoal.goalPos, level);
            neededPath.addAll(boxToGoalPath);
        }

        // Find blocking agents (not the goal owner) on the path
        boolean anyCleared = false;
        State workingState = state;

        for (int otherAgentId = 0; otherAgentId < numAgents; otherAgentId++) {
            if (otherAgentId == blockedGoal.agentId)
                continue;

            Position otherPos = workingState.getAgentPosition(otherAgentId);
            if (neededPath.contains(otherPos)) {
                logVerbose("[PREEMPTIVE] Agent " + otherAgentId + " at " + otherPos + " is blocking");

                // Set yielding state
                setAgentYielding(otherAgentId, blockedGoal.agentId);

                // Find parking position for this agent
                Position parkingPos = findParkingPosition(otherPos, workingState, level, neededPath, otherAgentId);
                if (parkingPos != null) {
                    List<Action> clearingPath = planAgentPath(otherAgentId, otherPos, parkingPos,
                            workingState, level, neededPath);

                    if (clearingPath != null && !clearingPath.isEmpty()) {
                        // Execute clearing path
                        for (Action action : clearingPath) {
                            Action[] jointAction = new Action[numAgents];
                            Arrays.fill(jointAction, Action.noOp());
                            jointAction[otherAgentId] = action;
                            jointAction = resolveConflicts(jointAction, workingState, level);
                            plan.add(jointAction);
                            workingState = applyJointAction(jointAction, workingState, level, numAgents);
                        }

                        logNormal("[PREEMPTIVE] Moved Agent " + otherAgentId + " from " + otherPos +
                                " to " + parkingPos + " to clear path for Agent " + blockedGoal.agentId);
                        anyCleared = true;
                    }
                }
            }
        }

        // Find blocking boxes that the blocked agent can move
        // LAYER 2: Compute satisfied goals - these boxes MUST NOT be moved
        Set<Position> satisfiedGoals = computeSatisfiedGoalPositions(workingState, level);

        Color agentColor = level.getAgentColor(blockedGoal.agentId);
        for (Map.Entry<Position, Character> boxEntry : new HashMap<>(workingState.getBoxes()).entrySet()) {
            Position boxPos = boxEntry.getKey();
            char boxType = boxEntry.getValue();

            if (neededPath.contains(boxPos) && level.getBoxColor(boxType) == agentColor) {
                // LAYER 2: Skip boxes at satisfied goal positions
                if (satisfiedGoals.contains(boxPos)) {
                    logVerbose("[LAYER2] Box " + boxType + " at " + boxPos +
                            " is at satisfied goal - NOT moving it");
                    continue;
                }

                logVerbose("[PREEMPTIVE] Box " + boxType + " at " + boxPos + " is blocking, trying to move it");

                // Try to push this box out of the way
                // This is more complex - need to find where agent can push from
                // For now, skip if it's the target box itself
                if (!blockedGoal.isAgentGoal && boxPos.equals(targetPos)) {
                    continue; // This is the box we want to push to goal, don't move it away
                }

                // Try to find a position to push box to (not in needed path)
                boolean boxMoved = tryPushBoxOutOfWay(plan, workingState, level, numAgents,
                        blockedGoal.agentId, boxPos, boxType, neededPath);
                if (boxMoved) {
                    // Update working state
                    State tempState = state;
                    for (int i = plan.size() - 10; i >= 0 && i < plan.size(); i++) {
                        if (i >= 0) {
                            tempState = applyJointAction(plan.get(i), tempState, level, numAgents);
                        }
                    }
                    workingState = tempState;
                    anyCleared = true;
                }
            }
        }

        return anyCleared;
    }

    /**
     * Finds a path from start to goal ignoring dynamic obstacles (agents and
     * boxes).
     * Returns all positions on shortest paths.
     */
    private Set<Position> findPathIgnoringDynamicObstacles(Position start, Position goal, Level level) {
        Set<Position> pathPositions = new HashSet<>();

        // BFS from start
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distances = new HashMap<>();

        queue.add(start);
        distances.put(start, 0);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int currentDist = distances.get(current);

            if (currentDist > goal.manhattanDistance(start) * 2 + 20) {
                continue; // Don't search too far
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!distances.containsKey(next) && !level.isWall(next)) {
                    distances.put(next, currentDist + 1);
                    queue.add(next);
                }
            }
        }

        // Backtrack from goal to find all positions on shortest paths
        if (!distances.containsKey(goal)) {
            return pathPositions; // No path exists
        }

        Queue<Position> backtrack = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        backtrack.add(goal);
        visited.add(goal);
        pathPositions.add(goal);
        pathPositions.add(start);

        while (!backtrack.isEmpty()) {
            Position current = backtrack.poll();
            int currentDist = distances.get(current);

            for (Direction dir : Direction.values()) {
                Position prev = current.move(dir);

                if (!visited.contains(prev) && distances.containsKey(prev)) {
                    if (distances.get(prev) == currentDist - 1) {
                        visited.add(prev);
                        backtrack.add(prev);
                        pathPositions.add(prev);
                    }
                }
            }
        }

        return pathPositions;
    }

    /**
     * Tries to push a blocking box out of the way.
     * LAYER 2: Will NOT push boxes that are already at satisfied goal positions.
     */
    private boolean tryPushBoxOutOfWay(List<Action[]> plan, State state, Level level, int numAgents,
            int agentId, Position boxPos, char boxType, Set<Position> forbidden) {
        // LAYER 2: Don't push boxes that are at satisfied goal positions
        char goalAtBoxPos = level.getBoxGoal(boxPos);
        if (goalAtBoxPos == boxType) {
            logVerbose("[LAYER2] Not pushing box " + boxType + " at " + boxPos +
                    " - it's at a satisfied goal position");
            return false;
        }

        Position agentPos = state.getAgentPosition(agentId);

        // Find a position adjacent to box where agent can push from
        for (Direction pushDir : Direction.values()) {
            Position pushFromPos = boxPos.move(pushDir.opposite());
            Position pushToPos = boxPos.move(pushDir);

            // Check if push is valid
            if (level.isWall(pushFromPos) || level.isWall(pushToPos))
                continue;
            if (state.getBoxes().containsKey(pushToPos))
                continue;
            if (forbidden.contains(pushToPos))
                continue; // Don't push into forbidden area

            // Check if any agent is at push destination
            boolean blocked = false;
            for (int i = 0; i < numAgents; i++) {
                if (state.getAgentPosition(i).equals(pushToPos)) {
                    blocked = true;
                    break;
                }
            }
            if (blocked)
                continue;

            // Try to get agent to push position
            List<Action> pathToPush = planAgentPath(agentId, agentPos, pushFromPos, state, level,
                    Collections.singleton(boxPos)); // Avoid the box position

            if (pathToPush != null) {
                State workingState = state;

                // Execute path to push position
                for (Action action : pathToPush) {
                    Action[] jointAction = new Action[numAgents];
                    Arrays.fill(jointAction, Action.noOp());
                    jointAction[agentId] = action;
                    jointAction = resolveConflicts(jointAction, workingState, level);
                    plan.add(jointAction);
                    workingState = applyJointAction(jointAction, workingState, level, numAgents);
                }

                // Execute push
                Action pushAction = Action.push(pushDir.opposite(), pushDir);
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[agentId] = pushAction;
                plan.add(jointAction);

                logVerbose("[PREEMPTIVE] Pushed box " + boxType + " from " + boxPos + " to " + pushToPos);
                return true;
            }
        }

        return false;
    }

    // ========== Yielding State Management ==========

    /**
     * Sets an agent to yielding state, meaning it should not move back to its goal
     * until the beneficiary agent has completed its task.
     */
    private void setAgentYielding(int yieldingAgentId, int beneficiaryId) {
        yieldingManager.setYielding(yieldingAgentId, beneficiaryId);
        logVerbose("[YIELD] Agent " + yieldingAgentId + " now YIELDING for Agent " + beneficiaryId);
    }

    // ========== Topological Depth Analysis ==========

    /**
     * Ensures topological depths are computed for all goal positions in the level.
     * This is cached and only computed once per level.
     */
    private void ensureTopologicalDepthsComputed(Level level) {
        if (goalTopologicalDepths != null && cachedLevel == level) {
            return; // Already computed for this level
        }

        cachedLevel = level;
        goalTopologicalDepths = computeTopologicalDepths(level);

        // Log the computed depths for debugging
        if (SearchConfig.isNormal()) {
            System.err.println("[TOPO] Computed topological depths for " + goalTopologicalDepths.size() + " goals:");
            List<Map.Entry<Position, Integer>> sorted = new ArrayList<>(goalTopologicalDepths.entrySet());
            sorted.sort((a, b) -> Integer.compare(b.getValue(), a.getValue())); // Descending
            for (Map.Entry<Position, Integer> e : sorted) {
                char goalType = level.getBoxGoal(e.getKey());
                if (goalType == '\0') {
                    int agentGoal = level.getAgentGoal(e.getKey());
                    System.err.println("  Agent " + agentGoal + " goal at " + e.getKey() + " -> depth " + e.getValue());
                } else {
                    System.err.println("  Box " + goalType + " goal at " + e.getKey() + " -> depth " + e.getValue());
                }
            }
        }
    }

    /**
     * Computes TOPOLOGICAL DEPTH for all goal positions using BFS from free space.
     * 
     * Algorithm:
     * 1. Identify all "free space" cells (non-wall, non-goal cells that are
     * reachable)
     * 2. BFS flood-fill from free space INTO goal regions
     * 3. Goals reached later (higher BFS distance) are DEEPER inside the dead-end
     * 4. Deeper goals MUST be filled FIRST to avoid blocking
     * 
     * This works for ANY shape: L-shaped, corridors, mazes, nested regions.
     */
    private Map<Position, Integer> computeTopologicalDepths(Level level) {
        Map<Position, Integer> depths = new HashMap<>();
        Set<Position> allGoals = new HashSet<>();
        Set<Position> freeSpaceBorder = new HashSet<>();

        // Step 1: Collect all goal positions
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                Position pos = new Position(row, col);
                if (level.getBoxGoal(row, col) != '\0' || level.getAgentGoal(row, col) >= 0) {
                    allGoals.add(pos);
                }
            }
        }

        // Step 2: Find "free space border" - non-goal cells adjacent to goal cells
        // These are the entry points into goal regions
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                Position pos = new Position(row, col);
                if (level.isWall(pos))
                    continue;
                if (allGoals.contains(pos))
                    continue; // Skip goal cells

                // Check if this free cell is adjacent to any goal
                for (Direction dir : Direction.values()) {
                    Position neighbor = pos.move(dir);
                    if (allGoals.contains(neighbor)) {
                        freeSpaceBorder.add(pos);
                        break;
                    }
                }
            }
        }

        // If no border found, use all non-wall, non-goal cells as starting points
        if (freeSpaceBorder.isEmpty()) {
            for (int row = 0; row < level.getRows(); row++) {
                for (int col = 0; col < level.getCols(); col++) {
                    Position pos = new Position(row, col);
                    if (!level.isWall(pos) && !allGoals.contains(pos)) {
                        freeSpaceBorder.add(pos);
                    }
                }
            }
        }

        // Step 3: BFS from free space border into goal regions
        // Distance = topological depth (higher = deeper inside)
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> visited = new HashMap<>();

        // Initialize: all free space border cells have depth 0
        for (Position border : freeSpaceBorder) {
            queue.add(border);
            visited.put(border, 0);
        }

        // BFS flood fill
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int currentDepth = visited.get(current);

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (level.isWall(next))
                    continue;
                if (visited.containsKey(next))
                    continue;

                int nextDepth = currentDepth + 1;
                visited.put(next, nextDepth);
                queue.add(next);

                // If this is a goal position, record its depth
                if (allGoals.contains(next)) {
                    depths.put(next, nextDepth);
                }
            }
        }

        // For any goals not reached by BFS (isolated), give them max depth
        for (Position goal : allGoals) {
            if (!depths.containsKey(goal)) {
                depths.put(goal, Integer.MAX_VALUE / 2);
            }
        }

        return depths;
    }

    // ========== Box Blocking Analysis (for Dependency-based Ordering) ==========

    /**
     * Counts how many OTHER boxes are blocking the path from a box to its goal.
     * This is the key insight for dependency-based ordering:
     * - A box with 0 blockers can be moved immediately
     * - A box with N blockers must wait for those N boxes to clear first
     * 
     * In spiral corridors like Spiraling.lvl, the innermost box (A) blocks all others,
     * so it should be moved FIRST (even though its goal might not be the "deepest").
     * 
     * @param boxPos Current position of the box to analyze
     * @param goalPos Target goal position for this box
     * @param state Current state (to know where all boxes are)
     * @param level Level information
     * @return Number of other boxes blocking the path, or Integer.MAX_VALUE if unreachable
     */
    private int countBlockingBoxes(Position boxPos, Position goalPos, State state, Level level) {
        // BFS to find path from box to goal, counting boxes encountered
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> visited = new HashMap<>(); // Position -> boxes encountered so far
        
        queue.add(boxPos);
        visited.put(boxPos, 0);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int currentBlockers = visited.get(current);
            
            if (current.equals(goalPos)) {
                return currentBlockers;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (level.isWall(next)) continue;
                if (visited.containsKey(next)) continue;
                
                int nextBlockers = currentBlockers;
                
                // Check if there's a box at this position (and it's not our starting box)
                char boxAtNext = state.getBoxAt(next);
                if (boxAtNext != '\0' && !next.equals(boxPos)) {
                    nextBlockers++;
                }
                
                // Also check if there's an agent (agents can move, so count as 0.5 blocker)
                // Actually, for simplicity, don't count agents as blockers
                
                visited.put(next, nextBlockers);
                queue.add(next);
            }
        }
        
        return Integer.MAX_VALUE; // Unreachable
    }

    /**
     * Computes a "blocking score" for a subgoal that considers both:
     * 1. How many boxes block THIS subgoal's execution
     * 2. How many OTHER subgoals are blocked by the box associated with this subgoal
     * 
     * Lower score = should be done first (fewer dependencies, or blocks more others)
     * 
     * This implements the MAPF principle: "Move boxes that block the most others first"
     */
    private int computeBlockingScore(Subgoal subgoal, State state, Level level, List<Subgoal> allSubgoals) {
        if (subgoal.isAgentGoal) {
            // For agent goals, just count boxes in the way
            return countBlockingBoxes(state.getAgentPosition(subgoal.agentId), subgoal.goalPos, state, level);
        }
        
        // For box goals, find the best box to move
        Position boxPos = findBestBoxForGoal(subgoal, state, level);
        if (boxPos == null) {
            return Integer.MAX_VALUE;
        }
        
        // Count how many boxes block this box from reaching its goal
        int blockedBy = countBlockingBoxes(boxPos, subgoal.goalPos, state, level);
        
        // Count how many other subgoals this box is blocking
        int blocksOthers = 0;
        for (Subgoal other : allSubgoals) {
            if (other == subgoal) continue;
            if (other.isAgentGoal) continue;
            
            Position otherBoxPos = findBestBoxForGoal(other, state, level);
            if (otherBoxPos == null) continue;
            
            // Check if our box is on the path from otherBox to otherGoal
            if (isBoxOnPath(boxPos, otherBoxPos, other.goalPos, state, level)) {
                blocksOthers++;
            }
        }
        
        // Score: prioritize boxes that are blocked by few AND block many others
        // Formula: blockedBy * 10 - blocksOthers
        // Lower score = higher priority
        // - Low blockedBy means we can move it now
        // - High blocksOthers means moving it will unblock many others
        return blockedBy * 10 - blocksOthers;
    }

    /**
     * Checks if a box position is on ANY shortest path between two positions.
     */
    private boolean isBoxOnPath(Position boxPos, Position from, Position to, State state, Level level) {
        // Simple check: is boxPos between from and to?
        // Use BFS to find if all shortest paths go through boxPos
        
        // First, find distance from 'from' to 'to' ignoring boxes
        int directDist = bfsDistance(from, to, level, null);
        if (directDist == Integer.MAX_VALUE) return false;
        
        // Now find distance avoiding boxPos
        Set<Position> avoid = new HashSet<>();
        avoid.add(boxPos);
        int avoidDist = bfsDistance(from, to, level, avoid);
        
        // If avoiding boxPos makes path longer, it's on the path
        return avoidDist > directDist;
    }

    /**
     * BFS distance between two positions, optionally avoiding certain positions.
     */
    private int bfsDistance(Position from, Position to, Level level, Set<Position> avoid) {
        // Delegate to PlanningUtils (SRP: single responsibility for planning utilities)
        return PlanningUtils.bfsDistance(from, to, level, avoid);
    }

    // ========== Reverse Planning (HTN-style Goal Ordering) ==========
    
    /**
     * Cache for reverse execution order. Computed once per level.
     * Maps goalPosition -> execution priority (lower = execute first)
     */
    private Map<Position, Integer> reverseExecutionOrder = null;
    private Level cachedLevelForReverse = null;
    
    /**
     * Computes the optimal execution order for box goals using CORRIDOR DEPTH analysis.
     * 
     * Key insight: In corridor/spiral layouts like Spiraling.lvl, the correct order
     * is to fill goals that are DEEPER in dead-ends FIRST (before their entrances get blocked).
     * 
     * Algorithm:
     * 1. For each goal position, compute its "corridor depth" = distance to open space
     * 2. Goals in dead-ends (far from open areas) have HIGH depth
     * 3. Goals near intersections have LOW depth
     * 4. HIGHER depth = should execute FIRST = LOWER priority number
     * 
     * @param level The level to analyze
     * @return Map from goal position to execution priority (lower = execute first)
     */
    private Map<Position, Integer> computeReverseExecutionOrder(Level level) {
        Map<Position, Integer> order = new HashMap<>();
        
        // Collect all box goals
        Set<Position> allGoals = new HashSet<>();
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getBoxGoal(row, col) != '\0') {
                    allGoals.add(new Position(row, col));
                }
            }
        }
        
        if (allGoals.isEmpty()) {
            return order;
        }
        
        // Compute corridor depth for each goal
        Map<Position, Integer> corridorDepths = new HashMap<>();
        int maxDepth = 0;
        
        for (Position goal : allGoals) {
            int depth = computeCorridorDepth(goal, level);
            corridorDepths.put(goal, depth);
            maxDepth = Math.max(maxDepth, depth);
        }
        
        // Convert depth to priority: HIGHER depth = LOWER priority number (execute first)
        for (Position goal : allGoals) {
            int depth = corridorDepths.get(goal);
            // Invert: deeper goals get lower priority number
            int priority = (maxDepth - depth) + 1;
            order.put(goal, priority);
        }
        
        // Log the computed order
        if (SearchConfig.isNormal()) {
            System.err.println("[REVERSE] Computed execution order by corridor depth:");
            List<Map.Entry<Position, Integer>> sorted = new ArrayList<>(order.entrySet());
            sorted.sort(Map.Entry.comparingByValue()); // Sort by priority (low first)
            for (Map.Entry<Position, Integer> e : sorted) {
                char goalType = level.getBoxGoal(e.getKey());
                int depth = corridorDepths.get(e.getKey());
                System.err.println("  Priority " + e.getValue() + " (depth=" + depth + "): Box " + 
                                   goalType + " at " + e.getKey());
            }
        }
        
        return order;
    }
    
    /**
     * Computes how deep a position is inside a corridor/dead-end.
     * Depth = distance to nearest "open space" (position with 3+ free neighbors).
     * 
     * A position deep in a dead-end has high depth.
     * A position at an intersection has depth 0.
     */
    private int computeCorridorDepth(Position start, Level level) {
        // BFS to find distance to nearest "open space"
        Queue<int[]> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        queue.add(new int[]{start.row, start.col, 0});
        visited.add(start);
        
        int searchLimit = 100;
        
        while (!queue.isEmpty()) {
            int[] current = queue.poll();
            Position pos = new Position(current[0], current[1]);
            int dist = current[2];
            
            if (dist > searchLimit) break;
            
            // Count free neighbors
            int freeNeighbors = 0;
            for (Direction dir : Direction.values()) {
                Position neighbor = pos.move(dir);
                if (!level.isWall(neighbor)) {
                    freeNeighbors++;
                }
            }
            
            // Open space = 3+ free neighbors
            if (freeNeighbors >= 3) {
                return dist;
            }
            
            // Continue BFS
            for (Direction dir : Direction.values()) {
                Position next = pos.move(dir);
                if (!level.isWall(next) && !visited.contains(next)) {
                    visited.add(next);
                    queue.add(new int[]{next.row, next.col, dist + 1});
                }
            }
        }
        
        // No open space found - very deep dead-end
        return searchLimit;
    }
    
    /**
     * Ensures reverse execution order is computed for the level.
     */
    private void ensureReverseOrderComputed(Level level) {
        if (reverseExecutionOrder != null && cachedLevelForReverse == level) {
            return;
        }
        
        cachedLevelForReverse = level;
        reverseExecutionOrder = computeReverseExecutionOrder(level);
    }
    
    /**
     * Gets the execution priority for a goal position.
     * Lower value = should be executed FIRST.
     */
    private int getReverseExecutionPriority(Position goalPos, Level level) {
        ensureReverseOrderComputed(level);
        return reverseExecutionOrder.getOrDefault(goalPos, Integer.MAX_VALUE / 2);
    }

    /**
     * Clears the yielding state for all agents that were yielding for a specific
     * beneficiary.
     * Called when the beneficiary completes its subgoal.
     */
    private void clearYieldingForBeneficiary(int beneficiaryId) {
        Map<Integer, Integer> yieldingAgents = yieldingManager.getYieldingAgents();
        for (Map.Entry<Integer, Integer> entry : yieldingAgents.entrySet()) {
            if (entry.getValue() == beneficiaryId) {
                int agentId = entry.getKey();
                yieldingManager.clearYielding(agentId);
                logVerbose("[YIELD] Agent " + agentId + " RELEASED from yielding (beneficiary Agent " + beneficiaryId
                        + " completed)");
            }
        }
    }

    /**
     * Proactive yielding: After completing a task, check if this agent should move out of the way.
     * 
     * Two scenarios trigger proactive yielding:
     * 1. Agent is blocking another agent's critical path (reactive)
     * 2. Agent is in a CORRIDOR (width=1) - should always move out to avoid future blocking
     * 
     * Key insight: In corridor-heavy maps like Spiraling, an agent completing a task
     * in a corridor will block ALL subsequent agents, even if not detected yet.
     * 
     * CRITICAL FIX: If agent still has an agent goal to reach, do NOT yield to a distant
     * "safe position". Instead, let the agent continue to its agent goal in Phase 2.
     * This fixes the issue where agents complete box goals but never go to their agent goals.
     * 
     * @return true if proactive yielding actions were added to the plan
     */
    private boolean performProactiveYielding(int completedAgentId, List<Action[]> plan, 
            State currentState, Level level, int numAgents) {
        
        Position currentPos = currentState.getAgentPosition(completedAgentId);
        
        // CRITICAL FIX: Check if agent has an agent goal to reach
        // If yes, do NOT mark as completed and do NOT yield to a distant position
        // The agent should continue to its agent goal in Phase 2
        Position agentGoal = findAgentGoalPosition(completedAgentId, level);
        if (agentGoal != null && !currentPos.equals(agentGoal)) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " has agent goal at " + agentGoal + 
                    ", skipping proactive yield (will go to goal in Phase 2)");
            // Do NOT mark as completed - agent still has work to do
            return false;
        }
        
        // Agent has no agent goal or is already at agent goal - can be marked as completed
        yieldingManager.markTaskCompleted(completedAgentId);
        
        // Use PASSABLE neighbors (considers boxes and agents) for corridor detection
        // This is critical because a box at a neighbor position effectively creates a corridor
        int passableNeighbors = countPassableNeighbors(currentPos, currentState, level, completedAgentId);
        int freeNeighbors = countFreeNeighbors(currentPos, level); // Map structure only
        
        logNormal("[PROACTIVE-YIELD-CHECK] Agent " + completedAgentId + " at " + currentPos + 
                ", passableNeighbors=" + passableNeighbors + " (map freeNeighbors=" + freeNeighbors + ")");
        
        // Check if this agent is blocking anyone
        List<Integer> blockedAgents = yieldingManager.findBlockedAgents(completedAgentId, currentState, level);
        
        // CRITICAL: Use passableNeighbors for corridor detection
        // An agent is in a "corridor situation" if it can only move in ≤2 directions
        // This catches cases where boxes create temporary corridor-like constraints
        boolean inCorridor = (passableNeighbors <= 2);
        
        if (blockedAgents.isEmpty() && !inCorridor) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " not blocking anyone and not in corridor");
            return false;
        }
        
        if (inCorridor) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " is in CORRIDOR at " + currentPos + 
                    " (passableNeighbors=" + passableNeighbors + "), must move out");
        }
        if (!blockedAgents.isEmpty()) {
            logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " blocking agents: " + blockedAgents);
        }
        
        // Find a safe position for this agent (with relaxed requirements for corridor maps)
        Position safePos = findBestYieldPosition(completedAgentId, currentState, level, blockedAgents);
        
        if (safePos == null) {
            logNormal("[PROACTIVE-YIELD] No yield position found for Agent " + completedAgentId);
            // Fallback: try single step move out of corridor
            Action yieldMove = findCorridorExitMove(completedAgentId, currentState, level);
            if (yieldMove != null) {
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[completedAgentId] = yieldMove;
                plan.add(jointAction);
                logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " performed corridor exit move");
                return true;
            }
            return false;
        }
        
        // Plan path to safe position
        List<Action> pathToSafe = yieldingManager.planPathToPosition(completedAgentId, safePos, currentState, level);
        
        if (pathToSafe.isEmpty()) {
            logNormal("[PROACTIVE-YIELD] No path to yield position for Agent " + completedAgentId);
            return false;
        }
        
        // Execute path to safe position
        logNormal("[PROACTIVE-YIELD] Agent " + completedAgentId + " moving to yield position " + safePos + 
                " (" + pathToSafe.size() + " steps)");
        
        State tempState = currentState;
        for (Action action : pathToSafe) {
            Action[] jointAction = new Action[numAgents];
            Arrays.fill(jointAction, Action.noOp());
            jointAction[completedAgentId] = action;
            
            // Verify action is still applicable
            if (!tempState.isApplicable(action, completedAgentId, level)) {
                logNormal("[PROACTIVE-YIELD] Path interrupted at action " + action);
                break;
            }
            
            plan.add(jointAction);
            tempState = applyJointAction(jointAction, tempState, level, numAgents);
        }
        
        return true;
    }

    /**
     * Finds the best yield position for an agent that has completed its task.
     * Uses a relaxed definition of "safe" for corridor-heavy maps:
     * 
     * Priority (highest to lowest):
     * 1. Position with 3+ free neighbors (true junction/open area)
     * 2. Position with 2 neighbors but in a "wider" area (near junction)
     * 3. Dead-end position (1 neighbor) - at least won't block main corridor
     * 
     * CRITICAL: Must use passableNeighbors (considers boxes) not just freeNeighbors (map structure only)!
     * A position with 3 free neighbors on the map might only have 1 passable neighbor if boxes are nearby.
     * 
     * IMPROVED: Now uses SafeZoneCalculator to check global working area (all agents' future paths)
     * 
     * @return Best yield position, or null if none found
     */
    private Position findBestYieldPosition(int agentId, State state, Level level, List<Integer> blockedAgents) {
        Position currentPos = state.getAgentPosition(agentId);
        
        // IMPROVED: Use SafeZoneCalculator for proper global working area analysis
        Position safePosFromCalculator = safeZoneCalculator.findSafePosition(agentId, state, level);
        if (safePosFromCalculator != null) {
            int passableNeighbors = countPassableNeighbors(safePosFromCalculator, state, level, agentId);
            logNormal("[YIELD] SafeZoneCalculator found position " + safePosFromCalculator + 
                    " (passableNeighbors=" + passableNeighbors + ")");
            return safePosFromCalculator;
        }
        
        // Fallback: Use AgentYieldingManager's strict safe position (now also uses SafeZoneCalculator internally)
        Position strictSafe = yieldingManager.findNearestSafePosition(agentId, state, level);
        if (strictSafe != null) {
            // Verify it's actually safe with current state (not just map structure)
            int passableNeighbors = countPassableNeighbors(strictSafe, state, level, agentId);
            if (passableNeighbors >= 3) {
                logNormal("[YIELD] Found strict safe position " + strictSafe + " (passableNeighbors=" + passableNeighbors + ")");
                return strictSafe;
            }
        }
        
        // IMPROVED: Compute global working area to avoid blocking any agent's future work
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
                // CRITICAL: Use passableNeighbors that considers current state (boxes, agents)
                int passableNeighbors = countPassableNeighbors(pos, state, level, agentId);
                int freeNeighbors = countFreeNeighbors(pos, level); // Map structure
                
                // Check not occupied
                if (state.getBoxAt(pos) == '\0' && !isPositionOccupiedByAgent(pos, state, state.getNumAgents())) {
                    // CRITICAL: Only consider positions that are NOT corridors (passableNeighbors >= 3)
                    // Exception: dead-ends (freeNeighbors == 1) are acceptable parking spots
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
                            Position otherGoal = findAgentGoalPosition(otherId, level);
                            if (otherGoal != null) {
                                Set<Position> criticalPath = findCriticalPositionsForAgentGoal(otherPos, otherGoal, level);
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
            int passable = countPassableNeighbors(bestPos, state, level, agentId);
            logNormal("[YIELD] Found best yield position " + bestPos + " (passableNeighbors=" + passable + ", score=" + bestScore + ")");
        }
        
        return bestPos;
    }

    /**
     * Finds a single move to exit a corridor (move toward more open space).
     */
    private Action findCorridorExitMove(int agentId, State state, Level level) {
        Position currentPos = state.getAgentPosition(agentId);
        int currentFreeNeighbors = countFreeNeighbors(currentPos, level);
        
        // Try each direction, prefer one that leads to more free space
        Direction bestDir = null;
        int bestNeighborCount = currentFreeNeighbors;
        
        for (Direction dir : Direction.values()) {
            Position newPos = currentPos.move(dir);
            
            if (level.isWall(newPos)) continue;
            if (state.getBoxes().containsKey(newPos)) continue;
            if (isPositionOccupiedByAgent(newPos, state, state.getNumAgents())) continue;
            
            int newFreeNeighbors = countFreeNeighbors(newPos, level);
            
            // Prefer direction with more free neighbors (moving toward open space)
            if (newFreeNeighbors > bestNeighborCount) {
                bestNeighborCount = newFreeNeighbors;
                bestDir = dir;
            } else if (bestDir == null && newFreeNeighbors >= currentFreeNeighbors) {
                // At least don't move to a tighter spot
                bestDir = dir;
            }
        }
        
        if (bestDir != null) {
            Action moveAction = Action.move(bestDir);
            if (state.isApplicable(moveAction, agentId, level)) {
                return moveAction;
            }
        }
        
        return null;
    }

    /**
     * Counts free (non-wall) neighbors of a position.
     * NOTE: This only considers map structure, not current state (boxes/agents).
     * For corridor detection with current state, use countPassableNeighbors().
     */
    private int countFreeNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (!level.isWall(neighbor)) {
                count++;
            }
        }
        return count;
    }

    /**
     * Counts passable (not blocked) neighbors considering current state.
     * A neighbor is passable if: not a wall, not occupied by a box, not occupied by another agent.
     * 
     * This is used for determining if an agent is effectively in a "corridor" situation,
     * where boxes can create temporary corridor-like constraints.
     */
    private int countPassableNeighbors(Position pos, State state, Level level, int excludeAgentId) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (level.isWall(neighbor)) continue;
            if (state.getBoxAt(neighbor) != '\0') continue;  // '\0' means no box
            
            // Check if another agent is there
            boolean hasAgent = false;
            for (int i = 0; i < state.getNumAgents(); i++) {
                if (i != excludeAgentId && state.getAgentPosition(i).equals(neighbor)) {
                    hasAgent = true;
                    break;
                }
            }
            if (hasAgent) continue;
            
            count++;
        }
        return count;
    }

    /**
     * Finds critical positions that an agent needs to pass through to reach its
     * goal.
     * Uses BFS to find ONE shortest path from agent to goal (not all shortest
     * paths).
     * 
     * SIMPLIFIED: Returns only positions on a single shortest path to avoid making
     * the forbidden area too large, which would leave no escape routes for blocking
     * agents.
     */
    private Set<Position> findCriticalPositionsForAgentGoal(Position agentPos, Position goalPos, Level level) {
        Set<Position> critical = new HashSet<>();

        // BFS to find ONE shortest path
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> cameFrom = new HashMap<>();

        queue.add(agentPos);
        cameFrom.put(agentPos, null);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (current.equals(goalPos)) {
                // Trace back the single path
                Position pos = goalPos;
                while (pos != null) {
                    critical.add(pos);
                    pos = cameFrom.get(pos);
                }
                logVerbose("[CRITICAL PATH] Agent " + agentPos + " -> " + goalPos +
                        ": " + critical.size() + " critical positions (single path)");
                return critical;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (!cameFrom.containsKey(next) && !level.isWall(next)) {
                    cameFrom.put(next, current);
                    queue.add(next);
                }
            }
        }

        // No path found
        logVerbose("[CRITICAL PATH] No path from " + agentPos + " to " + goalPos);
        return critical;
    }

    /**
     * Force a yielding agent to move, using progressively more aggressive
     * strategies.
     * This is called when normal clearing methods fail.
     */
    private boolean forceYieldingAgentToMove(List<Action[]> plan, State state, Level level,
            int numAgents, int yieldingAgentId, int beneficiaryId) {
        Position currentPos = state.getAgentPosition(yieldingAgentId);
        Position beneficiaryGoal = findAgentGoalPosition(beneficiaryId, level);

        if (beneficiaryGoal == null) {
            return false;
        }

        logNormal("[FORCE-YIELD] Forcing Agent " + yieldingAgentId +
                " to move (blocking Agent " + beneficiaryId + ")");

        // Sort directions: prefer moving AWAY from beneficiary's goal
        List<Direction> sortedDirs = new ArrayList<>(Arrays.asList(Direction.values()));
        sortedDirs.sort((d1, d2) -> {
            Position p1 = currentPos.move(d1);
            Position p2 = currentPos.move(d2);
            int dist1 = p1.manhattanDistance(beneficiaryGoal);
            int dist2 = p2.manhattanDistance(beneficiaryGoal);
            return Integer.compare(dist2, dist1); // Descending - farther is better
        });

        for (Direction dir : sortedDirs) {
            Position newPos = currentPos.move(dir);

            // Skip walls
            if (level.isWall(newPos))
                continue;

            // Skip if occupied by another agent
            boolean agentOccupied = false;
            for (int i = 0; i < numAgents; i++) {
                if (state.getAgentPosition(i).equals(newPos)) {
                    agentOccupied = true;
                    break;
                }
            }
            if (agentOccupied)
                continue;

            // If occupied by box, try to push it (if same color)
            Character boxAtPos = state.getBoxes().get(newPos);
            if (boxAtPos != null) {
                Color agentColor = level.getAgentColor(yieldingAgentId);
                Color boxColor = level.getBoxColor(boxAtPos);

                if (agentColor == boxColor) {
                    // Try to push the box
                    for (Direction pushDir : Direction.values()) {
                        Position boxDest = newPos.move(pushDir);
                        if (!level.isWall(boxDest) &&
                                !state.getBoxes().containsKey(boxDest) &&
                                !isPositionOccupiedByAgent(boxDest, state, numAgents)) {

                            Action pushAction = Action.push(dir, pushDir);
                            if (state.isApplicable(pushAction, yieldingAgentId, level)) {
                                Action[] jointAction = new Action[numAgents];
                                Arrays.fill(jointAction, Action.noOp());
                                jointAction[yieldingAgentId] = pushAction;
                                plan.add(jointAction);

                                logNormal("[FORCE-YIELD] Agent " + yieldingAgentId +
                                        " pushed box from " + newPos + " to " + boxDest);
                                return true;
                            }
                        }
                    }
                }
                continue; // Can't push this box, try another direction
            }

            // Position is free, just move there
            Action moveAction = Action.move(dir);
            if (state.isApplicable(moveAction, yieldingAgentId, level)) {
                Action[] jointAction = new Action[numAgents];
                Arrays.fill(jointAction, Action.noOp());
                jointAction[yieldingAgentId] = moveAction;
                plan.add(jointAction);

                logNormal("[FORCE-YIELD] Agent " + yieldingAgentId +
                        " moved from " + currentPos + " to " + newPos);
                return true;
            }
        }

        logNormal("[FORCE-YIELD] Agent " + yieldingAgentId + " has NO valid moves!");
        return false;
    }

    /**
     * Finds the agent goal position for a given agent ID.
     */
    private Position findAgentGoalPosition(int agentId, Level level) {
        // Delegate to GoalChecker (SRP: single responsibility for goal-related utilities)
        return GoalChecker.findAgentGoalPosition(agentId, level);
    }

    /**
     * Checks if a position is occupied by any agent.
     */
    private boolean isPositionOccupiedByAgent(Position pos, State state, int numAgents) {
        // Delegate to PlanningUtils (SRP: single responsibility for planning utilities)
        return PlanningUtils.isPositionOccupiedByAgent(pos, state, numAgents);
    }

    /**
     * Checks if an agent's goal position would block any higher-priority agent's
     * path.
     * Higher priority = deeper topological depth (should complete first).
     */
    private boolean wouldBlockHigherPriorityAgentGoal(int agentId, State state, Level level) {
        Position myGoal = findAgentGoalPosition(agentId, level);
        if (myGoal == null) {
            return false;
        }

        ensureTopologicalDepthsComputed(level);
        int myDepth = goalTopologicalDepths.getOrDefault(myGoal, 0);

        for (int otherId = 0; otherId < state.getNumAgents(); otherId++) {
            if (otherId == agentId)
                continue;

            Position otherGoal = findAgentGoalPosition(otherId, level);
            if (otherGoal == null)
                continue;

            // Check if other agent has already reached goal
            if (state.getAgentPosition(otherId).equals(otherGoal))
                continue;

            int otherDepth = goalTopologicalDepths.getOrDefault(otherGoal, 0);

            // If other agent has higher priority (deeper goal)
            if (otherDepth > myDepth) {
                Position otherPos = state.getAgentPosition(otherId);
                Set<Position> otherCriticalPath = findCriticalPositionsForAgentGoal(otherPos, otherGoal, level);

                if (otherCriticalPath.contains(myGoal)) {
                    logVerbose("[BLOCK-CHECK] Agent " + agentId + " goal " + myGoal +
                            " would block Agent " + otherId + "'s path to " + otherGoal);
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Gets all positions that this agent should avoid because moving there
     * would block a higher-priority agent's path to their goal.
     */
    private Set<Position> getPositionsThatWouldBlockHigherPriority(int agentId, State state, Level level) {
        Set<Position> blocking = new HashSet<>();

        // Only relevant during Agent Goal phase
        if (!allBoxGoalsSatisfied(state, level)) {
            return blocking;
        }

        ensureTopologicalDepthsComputed(level);
        Position myGoal = findAgentGoalPosition(agentId, level);
        int myDepth = (myGoal != null) ? goalTopologicalDepths.getOrDefault(myGoal, 0) : 0;

        for (int otherId = 0; otherId < state.getNumAgents(); otherId++) {
            if (otherId == agentId)
                continue;

            Position otherGoal = findAgentGoalPosition(otherId, level);
            if (otherGoal == null)
                continue;

            // Check if other agent has already reached goal
            if (state.getAgentPosition(otherId).equals(otherGoal))
                continue;

            int otherDepth = goalTopologicalDepths.getOrDefault(otherGoal, 0);

            // If other agent has higher priority (deeper goal)
            if (otherDepth > myDepth) {
                Position otherPos = state.getAgentPosition(otherId);
                Set<Position> otherCriticalPath = findCriticalPositionsForAgentGoal(otherPos, otherGoal, level);
                blocking.addAll(otherCriticalPath);
            }
        }

        return blocking;
    }
}