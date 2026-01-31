package mapf.planning;

import mapf.domain.*;
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
    private final ConflictDetector conflictDetector;
    private long timeoutMs;
    private int maxStates;
    private final Random random = new Random(SearchConfig.RANDOM_SEED); // For priority re-ordering

    /**
     * Tracks agents that are temporarily "yielding" - giving up their goal position
     * to let another agent pass through. Maps yieldingAgentId -> beneficiaryAgentId
     */
    private Map<Integer, Integer> yieldingAgents = new HashMap<>();

    /**
     * Cache for goal topological depths. Computed once per level.
     * Higher depth = deeper inside dead-end region = should be filled FIRST.
     */
    private Map<Position, Integer> goalTopologicalDepths = null;
    private Level cachedLevel = null;

    /**
     * Cache for immovable box positions (boxes that no agent can push).
     * These are treated as walls for pathfinding purposes.
     */
    private Set<Position> immovableBoxPositions = null;
    private State cachedStateForImmovable = null;

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
        this.conflictDetector = new ConflictDetector();
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
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
        yieldingAgents.clear();

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

            // Check for yielding timeout - if agents have been yielding for too long,
            // release them to prevent permanent deadlock.
            // CRITICAL: Only trigger ONCE at the exact threshold, not every iteration
            // after.
            // Also only trigger if we're about to give up anyway (near
            // MAX_STUCK_ITERATIONS).
            if (!yieldingAgents.isEmpty() && stuckCount == SearchConfig.MAX_STUCK_ITERATIONS - 5) {
                logNormal("[YIELD] Releasing all yielding agents due to potential deadlock (stuckCount=" + stuckCount
                        + ")");
                yieldingAgents.clear();
            }

            // NEW: Process yielding agents FIRST - make sure they actually move!
            // If an agent is marked as YIELDING but still blocking the beneficiary's path,
            // we must force it to move before continuing with normal planning.
            if (!yieldingAgents.isEmpty()) {
                boolean anyYieldingMoved = false;

                for (Map.Entry<Integer, Integer> entry : new HashMap<>(yieldingAgents).entrySet()) {
                    int yieldingAgentId = entry.getKey();
                    int beneficiaryId = entry.getValue();

                    Position yieldingPos = currentState.getAgentPosition(yieldingAgentId);
                    Position beneficiaryPos = currentState.getAgentPosition(beneficiaryId);
                    Position beneficiaryGoal = findAgentGoalPosition(beneficiaryId, level);

                    if (beneficiaryGoal != null) {
                        Set<Position> criticalPath = findCriticalPositionsForAgentGoal(
                                beneficiaryPos, beneficiaryGoal, level);

                        if (criticalPath.contains(yieldingPos)) {
                            // Yielding agent is STILL blocking! Force it to move.
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
                            logVerbose("[YIELD-CHECK] Agent " + yieldingAgentId +
                                    " no longer blocking (at " + yieldingPos + ")");
                        }
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
                if (yieldingAgents.containsKey(sg.agentId)) {
                    logVerbose("[YIELD] Skipping subgoal for Agent " + sg.agentId +
                            " (yielding for Agent " + yieldingAgents.get(sg.agentId) + ")");
                    return true;
                }
                return false;
            });

            unsatisfied.sort((a, b) -> {
                // First priority: TOPOLOGICAL DEPTH (higher depth = more inner = higher
                // priority)
                int depthA = goalTopologicalDepths.getOrDefault(a.goalPos, 0);
                int depthB = goalTopologicalDepths.getOrDefault(b.goalPos, 0);

                // DESCENDING order: deeper goals (higher depth) should be done FIRST
                if (depthA != depthB) {
                    return Integer.compare(depthB, depthA); // Note: B before A for descending
                }

                // Second priority: If same depth, use standard difficulty (easier first)
                int diffA = estimateSubgoalDifficulty(a, stateForSort, level);
                int diffB = estimateSubgoalDifficulty(b, stateForSort, level);
                return Integer.compare(diffA, diffB);
            });

            // CRITICAL: Only try goals at the HIGHEST depth level first
            // Do NOT skip to lower-depth goals if high-depth goals fail
            int highestDepth = unsatisfied.isEmpty() ? 0
                    : goalTopologicalDepths.getOrDefault(unsatisfied.get(0).goalPos, 0);

            List<Subgoal> highestDepthGoals = new ArrayList<>();
            List<Subgoal> lowerDepthGoals = new ArrayList<>();
            for (Subgoal sg : unsatisfied) {
                int depth = goalTopologicalDepths.getOrDefault(sg.goalPos, 0);
                if (depth == highestDepth) {
                    highestDepthGoals.add(sg);
                } else {
                    lowerDepthGoals.add(sg);
                }
            }

            boolean madeProgress = false;
            Subgoal highestPriorityFailedGoal = null; // Track the deepest goal that failed

            // First: ONLY try highest-depth goals
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
                                subgoal.agentId, action, currentState, level, numAgents);

                        // Resolve any conflicts
                        jointAction = resolveConflicts(jointAction, currentState, level);

                        fullPlan.add(jointAction);

                        // Update current state
                        currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    }

                    madeProgress = true;
                    stuckCount = 0;

                    // Release any agents that were yielding for this agent
                    clearYieldingForBeneficiary(subgoal.agentId);

                    if (subgoal.isAgentGoal) {
                        logMinimal(getName() + ": [OK] Agent " + subgoal.agentId +
                                " moved to goal position " + subgoal.goalPos + " (path: " + path.size() + " steps)");
                    } else {
                        logMinimal(getName() + ": [OK] Agent " + subgoal.agentId +
                                " moved box " + subgoal.boxType + " (path: " + path.size() + " steps)");
                    }
                    break;
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
                                        subgoal.agentId, action, currentState, level, numAgents);
                                jointAction = resolveConflicts(jointAction, currentState, level);
                                fullPlan.add(jointAction);
                                currentState = applyJointAction(jointAction, currentState, level, numAgents);
                            }
                            foundWithReorder = true;
                            stuckCount = 0;
                            clearYieldingForBeneficiary(subgoal.agentId);
                            if (subgoal.isAgentGoal) {
                                logNormal(getName() + ": Agent " + subgoal.agentId +
                                        " moved to goal position after reorder (path: " + path.size() + " steps)");
                            } else {
                                logNormal(getName() + ": Agent " + subgoal.agentId +
                                        " moved box " + subgoal.boxType + " after reorder (path: " + path.size()
                                        + " steps)");
                            }
                            break;
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
     */
    private List<Subgoal> getUnsatisfiedSubgoals(State state, Level level) {
        List<Subgoal> unsatisfied = new ArrayList<>();

        // Phase 1: Box goals
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position goalPos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(goalPos);

                    // Check if goal is not satisfied
                    if (actualBox == null || actualBox != goalType) {
                        Color boxColor = level.getBoxColor(goalType);
                        int agentId = findAgentForColor(boxColor, level, state.getNumAgents());
                        if (agentId != -1) {
                            unsatisfied.add(new Subgoal(agentId, goalType, goalPos, false));
                        }
                    }
                }
            }
        }

        // Phase 2: Agent goals (only after all box goals are satisfied or when making
        // progress on box goals is blocked)
        if (unsatisfied.isEmpty()) {
            for (int row = 0; row < level.getRows(); row++) {
                for (int col = 0; col < level.getCols(); col++) {
                    int agentGoal = level.getAgentGoal(row, col);
                    if (agentGoal >= 0 && agentGoal < state.getNumAgents()) {
                        Position goalPos = new Position(row, col);
                        Position agentPos = state.getAgentPosition(agentGoal);

                        // Check if agent is not at goal position
                        if (!agentPos.equals(goalPos)) {
                            // Use special boxType '\0' to indicate agent goal
                            unsatisfied.add(new Subgoal(agentGoal, '\0', goalPos, true));
                        }
                    }
                }
            }
        }

        return unsatisfied;
    }

    /**
     * Estimates difficulty of a subgoal using distance considering immovable boxes.
     * Returns Integer.MAX_VALUE if agent cannot reach any box of the required type.
     */
    private int estimateSubgoalDifficulty(Subgoal subgoal, State state, Level level) {
        // Handle agent goals (no box involved)
        if (subgoal.isAgentGoal) {
            Position agentPos = state.getAgentPosition(subgoal.agentId);
            return getDistanceWithImmovableBoxes(agentPos, subgoal.goalPos, state, level);
        }

        Position closestBox = findBestBoxForGoal(subgoal, state, level);
        if (closestBox == null) {
            return Integer.MAX_VALUE;
        }

        // Box to goal distance (considering immovable boxes as walls)
        int boxToGoal = getDistanceWithImmovableBoxes(closestBox, subgoal.goalPos, state, level);

        // Agent to box distance (considering immovable boxes as walls)
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        int agentToBox = getDistanceWithImmovableBoxes(agentPos, closestBox, state, level);

        // If agent can't reach box, return MAX_VALUE
        if (agentToBox == Integer.MAX_VALUE || boxToGoal == Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }

        return boxToGoal + agentToBox;
    }

    /**
     * Finds the best box of the given type to move to the goal.
     * CRITICAL: Must verify that the agent can actually REACH the box!
     * Boxes that the agent cannot reach (due to walls or immovable boxes) are
     * skipped.
     */
    private Position findBestBoxForGoal(Subgoal subgoal, State state, Level level) {
        Position bestBox = null;
        int bestTotalDist = Integer.MAX_VALUE;
        Position agentPos = state.getAgentPosition(subgoal.agentId);

        // Ensure immovable boxes are identified
        ensureImmovableBoxesComputed(state, level);

        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == subgoal.boxType) {
                Position boxPos = entry.getKey();

                // Skip if this box is already at a satisfied goal for this type
                if (level.getBoxGoal(boxPos) == subgoal.boxType) {
                    continue;
                }

                // Skip if this box is immovable (no agent can push it)
                if (immovableBoxPositions.contains(boxPos)) {
                    continue;
                }

                // CRITICAL: Check if agent can actually REACH this box
                // Use BFS that treats immovable boxes as walls
                int agentToBox = getDistanceWithImmovableBoxes(agentPos, boxPos, state, level);
                if (agentToBox == Integer.MAX_VALUE) {
                    // Agent cannot reach this box - skip it entirely!
                    logVerbose("[REACHABILITY] Agent " + subgoal.agentId + " cannot reach box " +
                            subgoal.boxType + " at " + boxPos + " - skipping");
                    continue;
                }

                int boxToGoal = getDistanceWithImmovableBoxes(boxPos, subgoal.goalPos, state, level);
                if (boxToGoal == Integer.MAX_VALUE) {
                    // Box cannot reach goal - skip it
                    continue;
                }

                int totalDist = agentToBox + boxToGoal;
                if (totalDist < bestTotalDist) {
                    bestTotalDist = totalDist;
                    bestBox = boxPos;
                }
            }
        }

        if (bestBox != null) {
            logVerbose("[REACHABILITY] Selected box " + subgoal.boxType + " at " + bestBox +
                    " for goal " + subgoal.goalPos + " (total dist: " + bestTotalDist + ")");
        }

        return bestBox;
    }

    /**
     * Computes and caches which boxes are immovable (no agent can push them).
     * These boxes should be treated as walls for pathfinding.
     */
    private void ensureImmovableBoxesComputed(State state, Level level) {
        // Only recompute if state changed (boxes might have moved)
        if (immovableBoxPositions != null && cachedStateForImmovable == state) {
            return;
        }

        cachedStateForImmovable = state;
        immovableBoxPositions = new HashSet<>();

        // First, find which colors have agents that can push them
        Set<Color> pushableColors = new HashSet<>();
        for (int i = 0; i < state.getNumAgents(); i++) {
            pushableColors.add(level.getAgentColor(i));
        }

        // Mark all boxes whose color has no matching agent as immovable
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            char boxType = entry.getValue();
            Color boxColor = level.getBoxColor(boxType);

            if (!pushableColors.contains(boxColor)) {
                immovableBoxPositions.add(entry.getKey());
            }
        }

        if (!immovableBoxPositions.isEmpty()) {
            logVerbose("[IMMOVABLE] Found " + immovableBoxPositions.size() + " immovable boxes (treated as walls)");
        }
    }

    /**
     * Calculates distance between two positions, treating immovable boxes as walls.
     * Uses BFS to find actual reachable distance.
     */
    private int getDistanceWithImmovableBoxes(Position from, Position to, State state, Level level) {
        if (from.equals(to))
            return 0;

        ensureImmovableBoxesComputed(state, level);

        // BFS to find shortest path treating immovable boxes as walls
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distances = new HashMap<>();

        queue.add(from);
        distances.put(from, 0);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int currentDist = distances.get(current);

            if (current.equals(to)) {
                return currentDist;
            }

            // Don't search too far
            if (currentDist > from.manhattanDistance(to) * 3 + 50) {
                continue;
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (distances.containsKey(next))
                    continue;
                if (level.isWall(next))
                    continue;

                // CRITICAL: Treat immovable boxes as walls!
                if (immovableBoxPositions.contains(next))
                    continue;

                distances.put(next, currentDist + 1);
                queue.add(next);
            }
        }

        return Integer.MAX_VALUE; // Unreachable
    }

    /**
     * Gets distance between two positions using true distance heuristic if
     * available.
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
    private Set<Position> computeSatisfiedGoalPositions(State state, Level level) {
        Set<Position> satisfiedPositions = new HashSet<>();

        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position pos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(pos);
                    if (actualBox != null && actualBox == goalType) {
                        satisfiedPositions.add(pos);
                    }
                }
            }
        }

        return satisfiedPositions;
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
     */
    private Action[] resolveConflicts(Action[] jointAction, State state, Level level) {
        List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(state, jointAction, level);

        for (ConflictDetector.Conflict conflict : conflicts) {
            // Lower priority agent waits (higher ID waits)
            int waitingAgent = Math.max(conflict.agent1, conflict.agent2);
            jointAction[waitingAgent] = Action.noOp();
        }

        return jointAction;
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
            if (yieldingAgents.containsKey(agentId)) {
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
        Color agentColor = level.getAgentColor(agentId);

        // Check box goals for this agent's color
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0' && level.getBoxColor(goalType) == agentColor) {
                    Position pos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(pos);
                    if (actualBox == null || actualBox != goalType) {
                        return false;
                    }
                }
            }
        }

        // Check agent goal position (if any)
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    Position goalPos = new Position(row, col);
                    if (!state.getAgentPosition(agentId).equals(goalPos)) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * Checks if ALL box goals in the level are satisfied (regardless of agent
     * color).
     * This is different from hasCompletedBoxTasks which only checks one agent's
     * color.
     * 
     * This method is used to control when agents can start moving toward their
     * agent goals - they should only do so after ALL box goals are globally
     * satisfied,
     * not just their own color's box goals.
     */
    private boolean allBoxGoalsSatisfied(State state, Level level) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position pos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(pos);
                    if (actualBox == null || actualBox != goalType) {
                        return false; // At least one box goal unsatisfied
                    }
                }
            }
        }
        return true; // All box goals satisfied globally
    }

    /**
     * Finds the agent that can move boxes of a given color.
     */
    private int findAgentForColor(Color color, Level level, int numAgents) {
        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Generates all possible actions (cached for performance).
     */
    private static List<Action> ALL_ACTIONS = null;

    private List<Action> getAllActions() {
        if (ALL_ACTIONS == null) {
            ALL_ACTIONS = new ArrayList<>();
            ALL_ACTIONS.add(Action.noOp());

            for (Direction dir : Direction.values()) {
                ALL_ACTIONS.add(Action.move(dir));
            }

            for (Direction agentDir : Direction.values()) {
                for (Direction boxDir : Direction.values()) {
                    ALL_ACTIONS.add(Action.push(agentDir, boxDir));
                    ALL_ACTIONS.add(Action.pull(agentDir, boxDir));
                }
            }
        }
        return ALL_ACTIONS;
    }

    // ========== Helper Classes ==========

    /**
     * Represents a subgoal: moving a box type to a specific goal position,
     * or moving an agent to its goal position.
     */
    private static class Subgoal {
        final int agentId;
        final char boxType; // '\0' for agent goals
        final Position goalPos;
        final boolean isAgentGoal; // true if this is an agent position goal

        Subgoal(int agentId, char boxType, Position goalPos, boolean isAgentGoal) {
            this.agentId = agentId;
            this.boxType = boxType;
            this.goalPos = goalPos;
            this.isAgentGoal = isAgentGoal;
        }

        // Backward compatibility constructor for box goals
        Subgoal(int agentId, char boxType, Position goalPos) {
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
            State state, Level level, int numAgents) {
        Action[] jointAction = new Action[numAgents];
        Arrays.fill(jointAction, Action.noOp());
        jointAction[primaryAgentId] = primaryAction;

        // Try to let other agents make progress too
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgentId)
                continue;
            if (isAgentGoalSatisfied(agentId, state, level))
                continue;

            // YIELDING CHECK: Don't let yielding agents move during plan merging
            if (yieldingAgents.containsKey(agentId)) {
                logVerbose("[SKIP] Agent " + agentId + " is YIELDING, skipping in plan merge");
                continue;
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
            if (yieldingAgents.containsKey(agentId)) {
                logVerbose("[GREEDY] Skipping Agent " + agentId + " - currently YIELDING for Agent "
                        + yieldingAgents.get(agentId));
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
                if (yieldingAgents.containsKey(agentId))
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
            Action[] jointAction = createJointActionWithMerging(
                    primaryAgent, primaryAction, state, level, numAgents);
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
        Color agentColor = level.getAgentColor(agentId);

        // Check all box goals for this agent's color
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0' && level.getBoxColor(goalType) == agentColor) {
                    Position pos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(pos);
                    if (actualBox == null || actualBox != goalType) {
                        return false; // Still has unsatisfied box goals
                    }
                }
            }
        }

        return true; // All box tasks complete
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
            if (yieldingAgents.containsKey(agentId))
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
        yieldingAgents.put(yieldingAgentId, beneficiaryId);
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

    /**
     * Clears the yielding state for all agents that were yielding for a specific
     * beneficiary.
     * Called when the beneficiary completes its subgoal.
     */
    private void clearYieldingForBeneficiary(int beneficiaryId) {
        Iterator<Map.Entry<Integer, Integer>> it = yieldingAgents.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Integer> entry = it.next();
            if (entry.getValue() == beneficiaryId) {
                int agentId = entry.getKey();
                it.remove();
                logVerbose("[YIELD] Agent " + agentId + " RELEASED from yielding (beneficiary Agent " + beneficiaryId
                        + " completed)");
            }
        }
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

    private Position findAgentGoalPosition(int agentId, Level level) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    return new Position(row, col);
                }
            }
        }
        return null;
    }

    private boolean isPositionOccupiedByAgent(Position pos, State state, int numAgents) {
        for (int i = 0; i < numAgents; i++) {
            if (state.getAgentPosition(i).equals(pos)) {
                return true;
            }
        }
        return false;
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