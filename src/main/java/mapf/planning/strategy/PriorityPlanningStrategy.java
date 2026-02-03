package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
import mapf.planning.analysis.DependencyAnalyzer;
import mapf.planning.cbs.CBSStrategy;
import mapf.planning.coordination.DeadlockResolver;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.spacetime.ReservationTable;
import mapf.planning.pibt.PIBT;
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
    private final TopologicalAnalyzer topologicalAnalyzer;
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
    
    /** Pre-computed goal execution order from LevelAnalyzer (optional). */
    private List<Position> precomputedGoalOrder = null;
    
    /** Immovable boxes (treated as walls in pathfinding). */
    private Set<Position> immovableBoxes = Collections.emptySet();
    
    /** Completed box goals - treated as permanent obstacles per MAPF standard. */
    private Set<Position> completedBoxGoals = new HashSet<>();
    
    /** Space-time reservation table for collision avoidance. */
    private ReservationTable reservationTable = new ReservationTable();
    
    /** Current global time step for space-time planning. */
    private int globalTimeStep = 0;
    
    /** PIBT for narrow corridor coordination. */
    private final PIBT pibt = new PIBT();

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
        this.heuristic = heuristic;
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
        
        this.subgoalManager = new SubgoalManager(heuristic);
        this.topologicalAnalyzer = new TopologicalAnalyzer();
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
    
    /** Sets immovable boxes (treated as walls in pathfinding). */
    public void setImmovableBoxes(Set<Position> immovable) {
        this.immovableBoxes = immovable != null ? immovable : Collections.emptySet();
    }

    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        int numAgents = initialState.getNumAgents();

        logMinimal(getName() + ": Planning for " + numAgents + " agents");

        // Reset for new search
        displacementHistory.clear();
        displacementAttempts = 0;
        completedBoxGoals.clear();
        reservationTable.clear();
        globalTimeStep = 0;

        List<Action[]> plan = planWithSubgoals(initialState, level, startTime);

        if (plan != null && !plan.isEmpty()) {
            logMinimal(getName() + ": Total plan length: " + plan.size());
        }
        return plan;
    }

    /**
     * Core PP algorithm: iteratively solve subgoals in priority order.
     * Per ARCHITECTURE.md: "moves one box at a time, naturally handles dependencies"
     */
    private List<Action[]> planWithSubgoals(State initialState, Level level, long startTime) {
        List<Action[]> fullPlan = new ArrayList<>();
        State currentState = initialState;
        int numAgents = initialState.getNumAgents();
        int stuckCount = 0;

        while (!currentState.isGoalState(level)) {
            // PRODUCT.md constraints: 3 minutes, 20,000 actions
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                logMinimal(getName() + ": Timeout reached");
                break;
            }
            if (fullPlan.size() >= SearchConfig.MAX_ACTIONS) {
                logMinimal(getName() + ": Action limit reached");
                break;
            }
            if (stuckCount > SearchConfig.MAX_STUCK_ITERATIONS) {
                logMinimal(getName() + ": Stuck after " + stuckCount + " iterations");
                break;
            }

            // Try CBS fallback on cyclic dependency detection
            if (stuckCount == SearchConfig.DEPENDENCY_CHECK_THRESHOLD && SearchConfig.USE_CBS_ON_CYCLE) {
                List<Action[]> cbsResult = tryCBSFallback(currentState, level, initialState, fullPlan, startTime, numAgents);
                if (cbsResult != null) return cbsResult;
            }

            // Get and sort unsatisfied subgoals (excluding completed goals)
            List<Subgoal> unsatisfied = subgoalManager.getUnsatisfiedSubgoals(currentState, level, completedBoxGoals);
            if (unsatisfied.isEmpty()) break;

            sortSubgoals(unsatisfied, currentState);
            
            // Log goal order once at start
            if (fullPlan.isEmpty() && SearchConfig.isNormal() && !unsatisfied.isEmpty()) {
                logGoalOrder(unsatisfied);
            }

            // Try to execute highest priority subgoal
            boolean madeProgress = tryExecuteSubgoals(unsatisfied, fullPlan, currentState, level, numAgents, initialState);
            
            if (madeProgress) {
                // Update state from plan
                currentState = recomputeState(initialState, fullPlan, level, numAgents);
                stuckCount = 0;
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
        } else {
            logMinimal(getName() + ": [FAIL] Could not reach goal state");
        }

        return fullPlan.isEmpty() ? null : fullPlan;
    }
    
    /** Sort subgoals by pre-computed order or difficulty. */
    private void sortSubgoals(List<Subgoal> subgoals, State state) {
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
            // Fallback: sort by difficulty (easier first)
            final State s = state;
            subgoals.sort((a, b) -> {
                int diffA = subgoalManager.estimateSubgoalDifficulty(a, s, null);
                int diffB = subgoalManager.estimateSubgoalDifficulty(b, s, null);
                return Integer.compare(diffA, diffB);
            });
        }
    }
    
    /** Log the goal execution order for debugging. */
    private void logGoalOrder(List<Subgoal> subgoals) {
        System.err.println("[PP] Goal execution order (" + subgoals.size() + " subgoals):");
        for (int i = 0; i < Math.min(15, subgoals.size()); i++) {
            Subgoal sg = subgoals.get(i);
            String desc = sg.isAgentGoal ? "Agent " + sg.agentId : "Box " + sg.boxType;
            System.err.println("  " + (i+1) + ". " + desc + " -> " + sg.goalPos);
        }
    }

    /** Try to execute subgoals in priority order. Returns true if progress made. */
    private boolean tryExecuteSubgoals(List<Subgoal> subgoals, List<Action[]> fullPlan, 
            State currentState, Level level, int numAgents, State initialState) {
        
        for (Subgoal subgoal : subgoals) {
            List<Action> path = planSubgoal(subgoal, currentState, level);
            
            // On-demand Clearing with PIBT: if planning fails, chain-clear blocking agents
            if (path == null && !subgoal.isAgentGoal) {
                int clearAttempts = 0;
                final int MAX_CLEAR_ATTEMPTS = 5; // Prevent infinite loops
                
                while (path == null && clearAttempts < MAX_CLEAR_ATTEMPTS) {
                    int blockingAgent = detectStaticBlockingAgent(subgoal, currentState, level);
                    if (blockingAgent == -1 || blockingAgent == subgoal.agentId) break;
                    
                    logVerbose("[PP] Agent " + blockingAgent + " blocks path to " + subgoal.goalPos);
                    boolean cleared = clearBlockingAgent(blockingAgent, subgoal.goalPos,
                            currentState, level, fullPlan, numAgents);
                    if (!cleared) break;
                    
                    currentState = recomputeState(initialState, fullPlan, level, numAgents);
                    path = planSubgoal(subgoal, currentState, level);
                    clearAttempts++;
                    
                    if (path != null) {
                        logNormal("[PP] Cleared " + clearAttempts + " agents, now executing " + subgoal.boxType);
                    }
                }
            }
            
            if (path != null && !path.isEmpty()) {
                // Record agent path in reservation table for space-time collision avoidance
                // For box goals, don't permanently reserve (agent will yield after)
                List<Position> agentPath = extractAgentPath(subgoal.agentId, currentState, path, level);
                boolean permanentEnd = subgoal.isAgentGoal; // Only agent goals stay permanently
                reservationTable.reservePath(subgoal.agentId, agentPath, globalTimeStep, permanentEnd);
                
                // Execute the path
                State tempState = currentState;
                for (Action action : path) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            subgoal.agentId, action, tempState, level, numAgents, subgoal.isAgentGoal);
                    jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, subgoal.agentId);
                    fullPlan.add(jointAction);
                    tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    globalTimeStep++;
                }
                
                // Verify goal was reached
                boolean reached = verifyGoalReached(subgoal, tempState, level);
                if (reached) {
                    // Mark box goal as completed (MAPF standard: treat as permanent obstacle)
                    if (!subgoal.isAgentGoal) {
                        completedBoxGoals.add(subgoal.goalPos);
                    }
                    
                    // Proactive Yielding: if agent is on a box goal, move off
                    tempState = yieldFromBoxGoal(subgoal.agentId, tempState, level, fullPlan, numAgents);
                    
                    logMinimal(getName() + ": [OK] " + 
                        (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) +
                        " -> " + subgoal.goalPos + " (" + path.size() + " steps)");
                    return true;
                }
            }
        }
        return false;
    }
    
    /**
     * Detect if another agent is statically blocking the path to a subgoal.
     * Uses BFS to find which agents sit on the required path.
     */
    private int detectStaticBlockingAgent(Subgoal subgoal, State state, Level level) {
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        Position boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level);
        if (boxPos == null) return -1;
        
        // BFS from agent to box, then box to goal - find first blocking agent
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(agentPos);
        visited.add(agentPos);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next) || level.isWall(next)) continue;
                
                // Check if another agent is here
                for (int i = 0; i < state.getNumAgents(); i++) {
                    if (i != subgoal.agentId && state.getAgentPosition(i).equals(next)) {
                        return i; // Found blocking agent
                    }
                }
                
                // Skip boxes for path exploration (we can push them)
                if (!state.hasBoxAt(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }
        return -1; // No blocking agent found
    }
    
    /**
     * Try to move a blocking agent using PIBT (chain-moving if necessary).
     * Direction is determined based on blocking agent's position relative to goal.
     */
    private boolean clearBlockingAgent(int blockingAgentId, Position goalPos,
            State state, Level level, List<Action[]> fullPlan, int numAgents) {
        
        Position blockingPos = state.getAgentPosition(blockingAgentId);
        
        // Determine clearing direction: move blocking agent AWAY from goal
        Direction clearDir = determineClearDirection(blockingPos, goalPos, state, level);
        if (clearDir == null) {
            logVerbose("[PP] No clear direction for agent " + blockingAgentId);
            return false;
        }
        
        logNormal("[PP] PIBT clearing agent " + blockingAgentId + " direction " + clearDir);
        
        // Use PIBT to chain-move blocking agent(s)
        PIBT.PIBTResult result = pibt.clearAgent(blockingAgentId, clearDir, state, level);
        
        if (!result.success) {
            logVerbose("[PP] PIBT clearing failed for agent " + blockingAgentId);
            return false;
        }
        
        // Add PIBT actions to plan
        for (Action[] jointAction : result.actions) {
            fullPlan.add(jointAction);
            globalTimeStep++;
        }
        
        logNormal("[PP] PIBT cleared with " + result.actions.size() + " steps");
        return true;
    }
    
    /**
     * Determine the best direction to clear a blocking agent.
     * Prefers moving away from the goal (opposite direction).
     */
    private Direction determineClearDirection(Position blockingPos, Position goalPos, 
            State state, Level level) {
        
        // Calculate direction from blocking agent to goal
        int dRow = Integer.signum(goalPos.row - blockingPos.row);
        int dCol = Integer.signum(goalPos.col - blockingPos.col);
        
        // Priority: 1) Away from goal, 2) Perpendicular, 3) Toward goal
        List<Direction> candidates = new ArrayList<>();
        
        // Away from goal (preferred - gets out of the way)
        if (dRow > 0) candidates.add(Direction.N);  // goal is South, go North
        if (dRow < 0) candidates.add(Direction.S);  // goal is North, go South
        if (dCol > 0) candidates.add(Direction.W);  // goal is East, go West
        if (dCol < 0) candidates.add(Direction.E);  // goal is West, go East
        
        // Perpendicular directions
        if (dRow != 0) {
            candidates.add(Direction.E);
            candidates.add(Direction.W);
        }
        if (dCol != 0) {
            candidates.add(Direction.N);
            candidates.add(Direction.S);
        }
        
        // Toward goal (last resort - might work with chain movement)
        if (dRow < 0) candidates.add(Direction.N);
        if (dRow > 0) candidates.add(Direction.S);
        if (dCol < 0) candidates.add(Direction.W);
        if (dCol > 0) candidates.add(Direction.E);
        
        // If no direction info (blocking at goal), try all directions
        if (candidates.isEmpty()) {
            for (Direction dir : Direction.values()) {
                candidates.add(dir);
            }
        }
        
        // Find first viable direction - PIBT handles agents, only check walls/boxes
        for (Direction dir : candidates) {
            Position nextPos = blockingPos.move(dir);
            if (!level.isWall(nextPos) && !state.hasBoxAt(nextPos)) {
                return dir;
            }
        }
        
        return null;
    }
    
    /**
     * Find a safe spot for an agent to move to.
     * Safe = not on any box goal, not on any agent goal, has multiple exits (freedom > 1).
     */
    private Position findSafeSpotForAgent(int agentId, Position start, State state, Level level) {
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        queue.add(start);
        visited.add(start);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            // Check if this is a safe spot (not start, not a goal, has freedom)
            if (!current.equals(start)) {
                boolean isBoxGoal = level.getBoxGoal(current) != '\0';
                boolean isAgentGoal = level.getAgentGoal(current) >= 0;
                int freedom = countFreeNeighbors(current, state, level);
                
                if (!isBoxGoal && !isAgentGoal && freedom >= 2) {
                    return current;
                }
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next) && 
                    !state.hasBoxAt(next) && !state.hasAgentAt(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }
        
        // Fallback: any non-goal position
        visited.clear();
        queue.add(start);
        visited.add(start);
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            if (!current.equals(start) && level.getBoxGoal(current) == '\0') {
                return current;
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next) && 
                    !state.hasBoxAt(next) && !state.hasAgentAt(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }
        return null;
    }
    
    /** Count free neighbors (not wall, not box, not agent). */
    private int countFreeNeighbors(Position pos, State state, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position next = pos.move(dir);
            if (!level.isWall(next) && !state.hasBoxAt(next) && !state.hasAgentAt(next)) {
                count++;
            }
        }
        return count;
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
    
    /** Move agent off box goal position to nearest non-goal position using BFS. */
    private State yieldFromBoxGoal(int agentId, State state, Level level, 
            List<Action[]> fullPlan, int numAgents) {
        Position agentPos = state.getAgentPosition(agentId);
        if (level.getBoxGoal(agentPos) == '\0') {
            return state; // Not on a box goal
        }
        
        // BFS to find nearest non-goal position
        List<Position> pathToSafe = findPathToNonGoal(agentPos, state, level);
        if (pathToSafe == null || pathToSafe.isEmpty()) {
            return state; // No safe position found
        }
        
        // Execute the path
        State tempState = state;
        Position currentPos = agentPos;
        for (Position nextPos : pathToSafe) {
            Direction dir = getDirection(currentPos, nextPos);
            if (dir == null) break;
            
            Action moveAction = Action.move(dir);
            Action[] jointAction = new Action[numAgents];
            for (int i = 0; i < numAgents; i++) {
                jointAction[i] = (i == agentId) ? moveAction : Action.noOp();
            }
            fullPlan.add(jointAction);
            tempState = tempState.apply(moveAction, agentId);
            currentPos = nextPos;
        }
        return tempState;
    }
    
    /** BFS to find path to nearest non-goal position. */
    private List<Position> findPathToNonGoal(Position start, State state, Level level) {
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> parent = new HashMap<>();
        queue.add(start);
        parent.put(start, null);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            // Found a non-goal position (not start)
            if (!current.equals(start) && level.getBoxGoal(current) == '\0') {
                // Reconstruct path
                List<Position> path = new ArrayList<>();
                Position p = current;
                while (p != null && !p.equals(start)) {
                    path.add(0, p);
                    p = parent.get(p);
                }
                return path;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!parent.containsKey(next) && !level.isWall(next) && 
                    !state.hasBoxAt(next) && !state.hasAgentAt(next)) {
                    parent.put(next, current);
                    queue.add(next);
                }
            }
        }
        return null; // No safe position reachable
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
    
    /** Plan path for a single subgoal. First tries fast 2D A*, then space-time A* if needed. */
    private List<Action> planSubgoal(Subgoal subgoal, State state, Level level) {
        if (subgoal.isAgentGoal) {
            return boxSearchPlanner.searchForAgentGoal(subgoal.agentId, subgoal.goalPos, state, level);
        } else {
            Position boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level);
            if (boxPos == null) return null;
            
            // First try fast 2D A* (no reservation table)
            List<Action> path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                    subgoal.goalPos, subgoal.boxType, state, level, new HashSet<>());
            if (path != null) return path;
            
            // Fallback to space-time A* if 2D search fails
            return boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                    subgoal.goalPos, subgoal.boxType, state, level, new HashSet<>(),
                    reservationTable, globalTimeStep);
        }
    }
    
    /** Verify that a subgoal was actually reached. */
    private boolean verifyGoalReached(Subgoal subgoal, State state, Level level) {
        if (subgoal.isAgentGoal) {
            return state.getAgentPosition(subgoal.agentId).equals(subgoal.goalPos);
        } else {
            return state.getBoxAt(subgoal.goalPos) == subgoal.boxType;
        }
    }

    /** Try CBS fallback when stuck with cyclic dependencies. */
    private List<Action[]> tryCBSFallback(State currentState, Level level, State initialState,
            List<Action[]> fullPlan, long startTime, int numAgents) {
        
        DependencyAnalyzer.AnalysisResult analysis = DependencyAnalyzer.analyze(currentState, level);
        
        if (analysis.hasCycle) {
            logNormal("[PP] Cyclic dependency detected, trying displacement...");
            
            if (displacementAttempts < MAX_DISPLACEMENT_ATTEMPTS) {
                displacementAttempts++;
                List<Action[]> displacementPlan = new ArrayList<>();
                boolean success = deadlockBreaker.attemptCycleBreaking(
                    displacementPlan, currentState, level, numAgents,
                    analysis.cycles.isEmpty() ? new ArrayList<>() : analysis.cycles.get(0),
                    pathAnalyzer, conflictResolver);
                
                if (success && !displacementPlan.isEmpty()) {
                    logNormal("[PP] Displacement succeeded with " + displacementPlan.size() + " steps");
                    fullPlan.addAll(displacementPlan);
                    return null; // Continue with PP
                }
            }
            
            // Try CBS as last resort
            logNormal("[PP] Trying CBS fallback...");
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
        criticalPositions.addAll(pathAnalyzer.findCriticalPositionsForAgentGoal(
            currentState, level, blockedGoal.agentId, blockedGoal.goalPos, satisfiedGoals));
        
        int planSizeBefore = fullPlan.size();
        AgentCoordinator.ClearingResult result = agentCoordinator.tryIdleAgentClearingWithResult(
            fullPlan, currentState, level, numAgents, blockedGoal.agentId, criticalPositions,
            pathAnalyzer, conflictResolver);
        
        if (result.success) {
            logNormal("[PP] Cleared blocking agent " + result.clearedAgentId);
            return true;
        }
        
        // Strategy 2: Try box displacement
        List<Integer> blockedAgents = new ArrayList<>();
        for (Subgoal sg : subgoals) blockedAgents.add(sg.agentId);
        
        List<DeadlockResolver.BlockingInfo> blockingInfos = 
            deadlockResolver.analyzeBlocking(currentState, level, blockedAgents, immovableBoxes);
        
        if (!blockingInfos.isEmpty()) {
            DeadlockResolver.DisplacementPlan displacement = 
                deadlockResolver.createDisplacementPlan(blockingInfos, currentState, level, displacementHistory);
            
            if (displacement != null) {
                List<Action> displacePath = boxSearchPlanner.planBoxDisplacement(
                    displacement.agentId, displacement.boxPosition,
                    displacement.tempPosition, displacement.boxType, currentState, level);
                
                if (displacePath != null && !displacePath.isEmpty()) {
                    String historyKey = displacement.boxType + "@" + displacement.boxPosition;
                    displacementHistory.add(historyKey);
                    
                    State tempState = currentState;
                    for (Action action : displacePath) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                            displacement.agentId, action, tempState, level, numAgents, false);
                        jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, displacement.agentId);
                        fullPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    }
                    logNormal("[PP] Displaced box " + displacement.boxType);
                    return true;
                }
            }
        }
        
        // Strategy 3: Random reorder and retry
        Collections.shuffle(subgoals, random);
        for (Subgoal sg : subgoals) {
            List<Action> path = planSubgoal(sg, currentState, level);
            if (path != null && !path.isEmpty()) {
                State tempState = currentState;
                for (Action action : path) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                        sg.agentId, action, tempState, level, numAgents, sg.isAgentGoal);
                    jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, sg.agentId);
                    fullPlan.add(jointAction);
                    tempState = applyJointAction(jointAction, tempState, level, numAgents);
                }
                if (verifyGoalReached(sg, tempState, level)) {
                    logNormal("[PP] Succeeded with reordered goal");
                    return true;
                }
            }
        }
        
        return false;
    }

    /** Recompute state from initial + all actions. */
    private State recomputeState(State initial, List<Action[]> plan, Level level, int numAgents) {
        State state = initial;
        for (Action[] jointAction : plan) {
            state = applyJointAction(jointAction, state, level, numAgents);
        }
        return state;
    }

    /** Apply joint action to state. */
    private State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        return planMerger.applyJointAction(jointAction, state, numAgents);
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
