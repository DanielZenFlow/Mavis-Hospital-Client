package mapf.planning;

import mapf.domain.*;
import java.util.*;

/**
 * Priority-based planning strategy for multi-agent path finding with subgoal decomposition.
 * 
 * Algorithm improvements based on course materials (ARCHITECTURE.md):
 * 1. Priority Re-ordering: If an agent fails, try different priority orderings
 * 2. Subgoal Decomposition: Plan one box at a time instead of all boxes simultaneously
 * 3. True Distance Heuristic: Use precomputed BFS distances instead of Manhattan distance
 * 
 * This addresses tightly-coupled scenarios where agents must coordinate (e.g., Agent 2
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
    private final Random random = new Random(42); // For priority re-ordering
    
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
        
        System.err.println(getName() + ": Planning for " + numAgents + " agents with subgoal decomposition");
        
        // Use iterative subgoal planning
        List<Action[]> plan = planWithSubgoals(initialState, level, startTime);
        
        if (plan != null && !plan.isEmpty()) {
            System.err.println(getName() + ": Total plan length: " + plan.size());
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
        int maxStuckIterations = 50; // Safety limit
        
        while (!currentState.isGoalState(level)) {
            // Check time limit (PRODUCT.md: 3 minutes)
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                System.err.println(getName() + ": Timeout reached");
                break;
            }
            
            // Check action limit (PRODUCT.md: 20,000 actions)
            if (fullPlan.size() >= SearchConfig.MAX_ACTIONS) {
                System.err.println(getName() + ": Reached action limit (" + SearchConfig.MAX_ACTIONS + ")");
                break;
            }
            
            if (stuckCount > maxStuckIterations) {
                System.err.println(getName() + ": Stuck after " + stuckCount + " iterations without progress");
                break;
            }
            
            // Get unsatisfied subgoals prioritized by difficulty
            List<Subgoal> unsatisfied = getUnsatisfiedSubgoals(currentState, level);
            
            if (unsatisfied.isEmpty()) {
                break; // All done
            }
            
            // Sort subgoals by estimated difficulty (easiest first)
            final State stateForSort = currentState;
            unsatisfied.sort((a, b) -> {
                int diffA = estimateSubgoalDifficulty(a, stateForSort, level);
                int diffB = estimateSubgoalDifficulty(b, stateForSort, level);
                return Integer.compare(diffA, diffB);
            });
            
            boolean madeProgress = false;
            
            // Try each subgoal until one succeeds
            for (Subgoal subgoal : unsatisfied) {
                if (System.currentTimeMillis() - startTime > timeoutMs) {
                    break;
                }
                
                List<Action> path = null;
                
                if (subgoal.isAgentGoal) {
                    // Agent goal: move agent to target position
                    path = searchForAgentGoal(subgoal.agentId, subgoal.goalPos, currentState, level);
                } else {
                    // Box goal: find box and move to goal
                    Position boxToMove = findBestBoxForGoal(subgoal, currentState, level);
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
                    if (subgoal.isAgentGoal) {
                        System.err.println(getName() + ": Agent " + subgoal.agentId + 
                                " moved to goal position " + subgoal.goalPos + " (path: " + path.size() + " steps)");
                    } else {
                        System.err.println(getName() + ": Agent " + subgoal.agentId + 
                                " moved box " + subgoal.boxType + " (path: " + path.size() + " steps)");
                    }
                    break;
                }
            }
            
            if (!madeProgress) {
                stuckCount++;
                // Log which subgoals are failing
                if (stuckCount == 1 || stuckCount % 10 == 0) {
                    System.err.println(getName() + ": Stuck iteration " + stuckCount + 
                            ", remaining subgoals:");
                    for (Subgoal sg : unsatisfied) {
                        int diff = estimateSubgoalDifficulty(sg, currentState, level);
                        if (sg.isAgentGoal) {
                            System.err.println("  - Agent " + sg.agentId + " -> goal position " + 
                                    sg.goalPos + " (difficulty: " + diff + ")");
                        } else {
                            System.err.println("  - Agent " + sg.agentId + " box " + sg.boxType + 
                                    " -> goal " + sg.goalPos + " (difficulty: " + diff + ")");
                        }
                    }
                }
                
                // IMPROVEMENT 1: Try random re-ordering of subgoals (Subgoal Serialization)
                // Theory: slides05 - when stuck, the issue might be wrong execution order
                boolean foundWithReorder = false;
                for (int reorderAttempt = 0; reorderAttempt < SearchConfig.MAX_REORDER_ATTEMPTS; reorderAttempt++) {
                    Collections.shuffle(unsatisfied, random);
                    
                    for (Subgoal subgoal : unsatisfied) {
                        List<Action> path = null;
                        
                        if (subgoal.isAgentGoal) {
                            path = searchForAgentGoal(subgoal.agentId, subgoal.goalPos, currentState, level);
                        } else {
                            Position boxToMove = findBestBoxForGoal(subgoal, currentState, level);
                            if (boxToMove == null) continue;
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
                            if (subgoal.isAgentGoal) {
                                System.err.println(getName() + ": Agent " + subgoal.agentId +
                                        " moved to goal position after reorder (path: " + path.size() + " steps)");
                            } else {
                                System.err.println(getName() + ": Agent " + subgoal.agentId +
                                        " moved box " + subgoal.boxType + " after reorder (path: " + path.size() + " steps)");
                            }
                            break;
                        }
                    }
                    if (foundWithReorder) break;
                }
                
                // IMPROVEMENT 2: If still stuck, try greedy step with plan merging
                if (!foundWithReorder) {
                    // NEW: Try clearing blocking agents before giving up
                    boolean clearedPath = false;
                    if (stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING) {
                        int planSizeBefore = fullPlan.size();
                        clearedPath = tryIdleAgentClearing(fullPlan, currentState, level, 
                                numAgents, unsatisfied);
                        if (clearedPath) {
                            // Apply all new actions to update currentState
                            for (int i = planSizeBefore; i < fullPlan.size(); i++) {
                                currentState = applyJointAction(fullPlan.get(i), currentState, level, numAgents);
                            }
                            stuckCount = 0;
                            System.err.println(getName() + ": Cleared blocking agent path");
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
            System.err.println(getName() + ": Goal state reached!");
        } else {
            System.err.println(getName() + ": Could not reach goal state");
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
        
        // Phase 2: Agent goals (only after all box goals are satisfied or when making progress on box goals is blocked)
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
     * Estimates difficulty of a subgoal using true distance heuristic when available.
     * Returns Integer.MAX_VALUE if agent cannot reach any box of the required type.
     */
    private int estimateSubgoalDifficulty(Subgoal subgoal, State state, Level level) {
        // Handle agent goals (no box involved)
        if (subgoal.isAgentGoal) {
            Position agentPos = state.getAgentPosition(subgoal.agentId);
            return getDistance(agentPos, subgoal.goalPos, level);
        }
        
        Position closestBox = findBestBoxForGoal(subgoal, state, level);
        if (closestBox == null) {
            return Integer.MAX_VALUE;
        }
        
        // Box to goal distance
        int boxToGoal = getDistance(closestBox, subgoal.goalPos, level);
        
        // Agent to box distance
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        int agentToBox = getDistance(agentPos, closestBox, level);
        
        // If agent can't reach box (blocked by other boxes/agents), penalize heavily
        if (agentToBox == Integer.MAX_VALUE || boxToGoal == Integer.MAX_VALUE) {
            return Integer.MAX_VALUE - 1; // Still possible but very hard
        }
        
        return boxToGoal + agentToBox;
    }
    
    /**
     * Finds the best box of the given type to move to the goal.
     * Prefers boxes not already at a goal position.
     */
    private Position findBestBoxForGoal(Subgoal subgoal, State state, Level level) {
        Position bestBox = null;
        int bestDist = Integer.MAX_VALUE;
        
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == subgoal.boxType) {
                Position boxPos = entry.getKey();
                
                // Skip if this box is already at a satisfied goal for this type
                if (level.getBoxGoal(boxPos) == subgoal.boxType) {
                    continue;
                }
                
                int dist = getDistance(boxPos, subgoal.goalPos, level);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestBox = boxPos;
                }
            }
        }
        
        return bestBox;
    }
    
    /**
     * Gets distance between two positions using true distance heuristic if available.
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
     * A* search to move a specific box to a goal position.
     * Uses optimized StateKey that only tracks agent and target box positions.
     */
    private List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
                                           char boxType, State initialState, Level level) {
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();
        
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
        Map<Position, Integer> bestG = new HashMap<>();  // Only track agent position for agent goals
        
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
        List<ConflictDetector.Conflict> conflicts = 
                conflictDetector.detectConflicts(state, jointAction, level);
        
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
     */
    private Action findBestGreedyAction(int agentId, State state, Level level) {
        Action bestAction = Action.noOp();
        int bestH = estimateAgentCost(agentId, state, level);
        
        for (Action action : getAllActions()) {
            if (action.type == Action.ActionType.NOOP) {
                continue;
            }
            
            if (!state.isApplicable(action, agentId, level)) {
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
        Position agentPos = state.getAgentPosition(agentId);
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
        final char boxType;  // '\0' for agent goals
        final Position goalPos;
        final boolean isAgentGoal;  // true if this is an agent position goal
        
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
            if (this == obj) return true;
            if (!(obj instanceof StateKey)) return false;
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
            if (fCompare != 0) return fCompare;
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
     * Creates a joint action with plan merging - other agents can act if they don't conflict.
     * Based on slides06: Plan Merging / Post-Processing.
     */
    private Action[] createJointActionWithMerging(int primaryAgentId, Action primaryAction,
                                                   State state, Level level, int numAgents) {
        Action[] jointAction = new Action[numAgents];
        Arrays.fill(jointAction, Action.noOp());
        jointAction[primaryAgentId] = primaryAction;
        
        // Try to let other agents make progress too
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgentId) continue;
            if (isAgentGoalSatisfied(agentId, state, level)) continue;
            
            // Find a useful action for this agent
            Action bestAction = findBestGreedyAction(agentId, state, level);
            if (bestAction != null && bestAction.type != Action.ActionType.NOOP) {
                // Temporarily add action and check for conflicts
                jointAction[agentId] = bestAction;
                List<ConflictDetector.Conflict> conflicts = 
                        conflictDetector.detectConflicts(state, jointAction, level);
                
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
            if (isAgentGoalSatisfied(agentId, state, level)) continue;
            
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
                if (isAgentGoalSatisfied(agentId, state, level)) continue;
                
                for (Action action : getAllActions()) {
                    if (action.type == Action.ActionType.MOVE && 
                        state.isApplicable(action, agentId, level)) {
                        primaryAgent = agentId;
                        primaryAction = action;
                        break;
                    }
                }
                if (primaryAgent != -1) break;
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
            Position boxPos = findBestBoxForGoal(subgoal, state, level);
            if (boxPos == null) continue;
            
            // Find positions along potential paths from box to goal
            Set<Position> criticalPositions = findCriticalPositions(
                    state.getAgentPosition(subgoal.agentId), boxPos, subgoal.goalPos, level);
            
            // Check which completed agents are blocking
            for (int blockingAgentId : completedAgents) {
                if (blockingAgentId == subgoal.agentId) continue;
                
                Position blockingPos = state.getAgentPosition(blockingAgentId);
                if (criticalPositions.contains(blockingPos)) {
                    // Found a blocking agent! Try to move it
                    boolean cleared = clearBlockingAgent(plan, state, level, numAgents,
                            blockingAgentId, criticalPositions);
                    if (cleared) {
                        return true;
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
        
        System.err.println(getName() + ": Moved Agent " + blockingAgentId + 
                " from " + currentPos + " to " + parkingPos + " (clearing path)");
        
        return true;
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
                
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                
                // Check if position is free (no box, no other agent)
                if (state.getBoxes().containsKey(next)) continue;
                
                boolean occupiedByOther = false;
                for (int i = 0; i < state.getNumAgents(); i++) {
                    if (i != agentId && state.getAgentPosition(i).equals(next)) {
                        occupiedByOther = true;
                        break;
                    }
                }
                if (occupiedByOther) continue;
                
                visited.add(next);
                Action moveAction = Action.move(dir);
                queue.add(new PathNode(next, current, moveAction));
            }
        }
        
        return null; // No path found
    }
    
    /**
     * Adds moves for other completed agents that can help clear the path.
     * Only moves agents that have completed box tasks but are blocking critical paths.
     */
    private Action[] addOtherAgentMoves(Action[] jointAction, State state, Level level,
                                         int numAgents, int primaryAgent, 
                                         Set<Position> forbidden) {
        for (int agentId = 0; agentId < numAgents; agentId++) {
            if (agentId == primaryAgent) continue;
            if (jointAction[agentId].type != Action.ActionType.NOOP) continue;
            
            // Skip if agent has completed ALL goals (no need to move at all)
            if (isAgentGoalSatisfied(agentId, state, level)) continue;
            
            // Only move agents that have completed their box tasks
            if (!hasCompletedBoxTasks(agentId, state, level)) continue;
            
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
}
