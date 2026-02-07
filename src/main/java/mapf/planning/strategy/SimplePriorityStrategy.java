package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
import mapf.planning.coordination.ConflictDetector;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.goal.Goal;
import mapf.planning.goal.GoalExtractor;
import mapf.planning.pathfinding.AStarPathPlanner;
import mapf.planning.pathfinding.PathPlanner;

import java.util.*;

/**
 * REFACTORED Priority Planning Strategy.
 * 
 * Follows ARCHITECTURE.md guidelines:
 * - Single Responsibility: Orchestrates goal execution, delegates to specialists
 * - Strategy Pattern: Implements SearchStrategy interface
 * - Clean separation: Goal extraction, path planning, conflict detection are delegated
 * 
 * Key insight from domain analysis:
 * Push+Pull means NO dead-ends exist. A box can always be moved back.
 * This eliminates the need for complex dead-end-first ordering logic.
 * 
 * Algorithm:
 * 1. Extract unsatisfied goals (box goals first, then agent goals)
 * 2. For each goal, plan a path using A*
 * 3. Execute one action at a time, letting other agents act via plan merging
 * 4. Handle conflicts by simple waiting or re-planning
 */
public class SimplePriorityStrategy implements SearchStrategy {
    
    private final ConflictDetector conflictDetector = new ConflictDetector();
    private final AStarPathPlanner pathPlanner = new AStarPathPlanner();
    
    private long timeoutMs = 180_000; // 3 minutes
    private int maxActions = 20_000;
    
    // Current planning state
    private GoalExtractor goalExtractor;
    private Map<Integer, List<Action>> agentPlans = new HashMap<>();
    private Set<Position> satisfiedGoalPositions = new HashSet<>();
    
    // Logging
    private static final boolean VERBOSE = SearchConfig.isVerbose();
    
    @Override
    public String getName() {
        return "SimplePriorityStrategy";
    }
    
    @Override
    public void setTimeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    public void setMaxStates(int maxStates) {
        // Not directly used, but kept for interface compatibility
    }
    
    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        
        // Initialize
        goalExtractor = new GoalExtractor(level);
        agentPlans.clear();
        satisfiedGoalPositions.clear();
        
        List<Action[]> solution = new ArrayList<>();
        State currentState = initialState;
        int numAgents = level.getNumAgents();
        
        log("Starting SimplePriorityStrategy with " + numAgents + " agents");
        log("Box goals: " + goalExtractor.getBoxGoalCount() + ", Agent goals: " + goalExtractor.getAgentGoalCount());
        
        // Main loop: execute actions until all goals satisfied or timeout/limit
        while (!goalExtractor.areAllGoalsSatisfied(currentState)) {
            // Timeout check
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                log("TIMEOUT after " + solution.size() + " actions");
                return null;
            }
            
            // Action limit check
            if (solution.size() >= maxActions) {
                log("ACTION LIMIT reached: " + maxActions);
                return null;
            }
            
            // Update frozen positions (satisfied goals should not be disturbed)
            updateFrozenPositions(currentState, level);
            
            // Get the next goal to work on
            Goal currentGoal = selectNextGoal(currentState);
            if (currentGoal == null) {
                // No goals but not satisfied? Something is wrong
                log("ERROR: No goal selected but not all goals satisfied");
                return null;
            }
            
            // Get or plan path for this goal
            List<Action> plan = getOrPlanForGoal(currentGoal, currentState, level);
            if (plan == null || plan.isEmpty()) {
                // Can't find path - try to handle
                log("No path for " + currentGoal + ", attempting recovery");
                
                // Simple recovery: let the agent wait, maybe other agents will move
                Action[] waitAction = createWaitAction(numAgents);
                List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(currentState, waitAction, level);
                if (conflicts.isEmpty()) {
                    currentState = applyJointAction(waitAction, currentState, level);
                    solution.add(waitAction);
                    continue;
                }
                
                // If even waiting fails, we're stuck
                log("STUCK: Cannot proceed for " + currentGoal);
                return null;
            }
            
            // Execute one action from the plan
            Action agentAction = plan.remove(0);
            int agentId = currentGoal.agentId;
            
            // Create joint action with plan merging (let other agents move if possible)
            Action[] jointAction = createJointAction(agentId, agentAction, currentState, level, numAgents);
            
            // Check for conflicts
            List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(currentState, jointAction, level);
            
            if (!conflicts.isEmpty()) {
                // Handle conflicts: simplified approach - just wait
                log("Conflict detected for agent " + agentId + ", waiting");
                jointAction = createWaitAction(numAgents);
                // Clear the plan to re-plan next iteration
                agentPlans.remove(agentId);
            }
            
            // Apply and record
            currentState = applyJointAction(jointAction, currentState, level);
            solution.add(jointAction);
            
            // Update satisfied goals
            updateSatisfiedGoals(currentState, level);
        }
        
        log("SUCCESS: Found solution with " + solution.size() + " actions");
        return solution;
    }
    
    // ============ Goal Selection ============
    
    /**
     * Selects the next goal to work on.
     * Simple strategy: first unsatisfied box goal, then agent goals.
     */
    private Goal selectNextGoal(State state) {
        List<Goal> unsatisfied = goalExtractor.getUnsatisfiedGoalsSmart(state);
        if (unsatisfied.isEmpty()) return null;
        
        // Prefer box goals (they come first from GoalExtractor)
        for (Goal goal : unsatisfied) {
            if (goal.isBoxGoal()) {
                return goal;
            }
        }
        
        // All box goals done, return first agent goal
        return unsatisfied.get(0);
    }
    
    // ============ Path Planning ============
    
    private List<Action> getOrPlanForGoal(Goal goal, State state, Level level) {
        int agentId = goal.agentId;
        
        // Check if we have a valid cached plan
        List<Action> cached = agentPlans.get(agentId);
        if (cached != null && !cached.isEmpty()) {
            // Validate the first action is still applicable
            if (state.isApplicable(cached.get(0), agentId, level)) {
                return cached;
            }
            // Plan invalid, re-plan
            agentPlans.remove(agentId);
        }
        
        // Plan new path
        PathPlanner.PathResult result;
        if (goal.isBoxGoal()) {
            // Find the box to push
            Position boxPos = findBoxForGoal(goal, state, level);
            if (boxPos == null) return null;
            result = pathPlanner.findBoxPath(state, level, agentId, boxPos, goal.position);
        } else {
            result = pathPlanner.findAgentPath(state, level, agentId, goal.position);
        }
        
        if (!result.found) return null;
        
        // Cache and return
        List<Action> plan = new ArrayList<>(result.actions);
        agentPlans.put(agentId, plan);
        return plan;
    }
    
    /**
     * Finds the closest box that matches the goal's box type and can be pushed by the agent.
     */
    private Position findBoxForGoal(Goal goal, State state, Level level) {
        Position agentPos = state.getAgentPosition(goal.agentId);
        Position bestBox = null;
        int bestDist = Integer.MAX_VALUE;
        
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == goal.boxType) {
                Position boxPos = entry.getKey();
                // Skip boxes already at goals (satisfied)
                if (satisfiedGoalPositions.contains(boxPos)) continue;
                
                int dist = agentPos.manhattanDistance(boxPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestBox = boxPos;
                }
            }
        }
        
        return bestBox;
    }
    
    // ============ Action Creation ============
    
    private Action[] createWaitAction(int numAgents) {
        Action[] actions = new Action[numAgents];
        Arrays.fill(actions, Action.noOp());
        return actions;
    }
    
    private Action[] createJointAction(int primaryAgent, Action primaryAction, State state, Level level, int numAgents) {
        Action[] jointAction = new Action[numAgents];
        Arrays.fill(jointAction, Action.noOp());
        jointAction[primaryAgent] = primaryAction;
        
        // Plan merging: let other agents make progress if they don't conflict
        // (Simplified: skip for now to keep it clean)
        // TODO: Add plan merging for efficiency
        
        return jointAction;
    }
    
    private State applyJointAction(Action[] jointAction, State state, Level level) {
        return state.applyJointAction(jointAction, level);
    }
    
    // ============ Goal Status Tracking ============
    
    private void updateFrozenPositions(State state, Level level) {
        satisfiedGoalPositions.clear();
        for (Goal goal : goalExtractor.getAllBoxGoals()) {
            if (goalExtractor.isBoxGoalSatisfied(goal, state)) {
                satisfiedGoalPositions.add(goal.position);
            }
        }
        pathPlanner.setFrozenPositions(satisfiedGoalPositions);
    }
    
    private void updateSatisfiedGoals(State state, Level level) {
        // Already updated in updateFrozenPositions
    }
    
    // ============ Logging ============
    
    private void log(String msg) {
        if (VERBOSE) {
            System.err.println("[SimplePriority] " + msg);
        }
    }
}
