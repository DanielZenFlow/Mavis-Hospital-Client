package mapf.planning.pibt;

import mapf.domain.*;
import java.util.*;

/**
 * Priority Inheritance with Backtracking (PIBT).
 * Handles narrow corridor coordination by chain-moving blocking agents.
 */
public class PIBT {
    
    /** Result of a PIBT move attempt. */
    public static class PIBTResult {
        public final boolean success;
        public final List<Action[]> actions;  // Joint actions executed
        public final State finalState;
        
        public PIBTResult(boolean success, List<Action[]> actions, State finalState) {
            this.success = success;
            this.actions = actions;
            this.finalState = finalState;
        }
    }
    
    /**
     * Move an agent in a direction, chain-moving any blocking agents.
     * 
     * @param agentId The agent that wants to move
     * @param targetPos The position the agent wants to reach
     * @param state Current state
     * @param level Level data
     * @return PIBTResult with success status and actions taken
     */
    public PIBTResult moveAgentTo(int agentId, Position targetPos, State state, Level level) {
        int numAgents = state.getNumAgents();
        List<Action[]> allActions = new ArrayList<>();
        State currentState = state;
        
        Position agentPos = currentState.getAgentPosition(agentId);
        
        // BFS to find path to target
        List<Direction> path = findPath(agentPos, targetPos, currentState, level);
        if (path == null || path.isEmpty()) {
            return new PIBTResult(false, allActions, currentState);
        }
        
        // Execute each step with PIBT
        for (Direction dir : path) {
            PIBTStepResult stepResult = pibtStep(agentId, dir, currentState, level, new HashSet<>());
            if (!stepResult.success) {
                return new PIBTResult(false, allActions, currentState);
            }
            allActions.addAll(stepResult.actions);
            currentState = stepResult.finalState;
        }
        
        return new PIBTResult(true, allActions, currentState);
    }
    
    /**
     * Clear a blocking agent by moving it in the specified direction using PIBT.
     * This is the main entry point for clearing operations.
     */
    public PIBTResult clearAgent(int blockingAgent, Direction clearDir, State state, Level level) {
        return pibtStepToResult(blockingAgent, clearDir, state, level, new HashSet<>());
    }
    
    /** Result of a single PIBT step. */
    private static class PIBTStepResult {
        final boolean success;
        final List<Action[]> actions;
        final State finalState;
        
        PIBTStepResult(boolean success, List<Action[]> actions, State finalState) {
            this.success = success;
            this.actions = actions;
            this.finalState = finalState;
        }
    }
    
    /**
     * Core PIBT algorithm: move agent in direction, recursively clearing blockers.
     * 
     * @param agentId Agent to move
     * @param dir Direction to move
     * @param state Current state
     * @param level Level data
     * @param committed Agents already in the clearing chain (prevents cycles)
     */
    private PIBTStepResult pibtStep(int agentId, Direction dir, State state, Level level, Set<Integer> committed) {
        int numAgents = state.getNumAgents();
        Position agentPos = state.getAgentPosition(agentId);
        Position nextPos = agentPos.move(dir);
        
        // Wall check
        if (level.isWall(nextPos)) {
            return new PIBTStepResult(false, Collections.emptyList(), state);
        }
        
        // Box check - can't move through boxes (for now)
        if (state.hasBoxAt(nextPos)) {
            return new PIBTStepResult(false, Collections.emptyList(), state);
        }
        
        List<Action[]> actions = new ArrayList<>();
        State currentState = state;
        
        // Check if another agent is blocking
        int blockingAgent = findAgentAt(nextPos, currentState);
        if (blockingAgent != -1) {
            // Cycle detection
            if (committed.contains(blockingAgent)) {
                return new PIBTStepResult(false, Collections.emptyList(), state);
            }
            
            // Priority inheritance: blocking agent must move in same direction
            Set<Integer> newCommitted = new HashSet<>(committed);
            newCommitted.add(agentId);
            
            PIBTStepResult clearResult = pibtStep(blockingAgent, dir, currentState, level, newCommitted);
            if (!clearResult.success) {
                // Try alternative directions for the blocking agent
                clearResult = tryAlternativeDirections(blockingAgent, dir, currentState, level, newCommitted);
                if (!clearResult.success) {
                    return new PIBTStepResult(false, Collections.emptyList(), state);
                }
            }
            
            actions.addAll(clearResult.actions);
            currentState = clearResult.finalState;
        }
        
        // Now the path should be clear, move the agent
        Action moveAction = Action.move(dir);
        if (!currentState.isApplicable(moveAction, agentId, level)) {
            return new PIBTStepResult(false, actions, currentState);
        }
        
        Action[] jointAction = createJointAction(agentId, moveAction, numAgents);
        actions.add(jointAction);
        currentState = currentState.apply(moveAction, agentId);
        
        return new PIBTStepResult(true, actions, currentState);
    }
    
    /** Wrapper for pibtStep that returns PIBTResult. */
    private PIBTResult pibtStepToResult(int agentId, Direction dir, State state, Level level, Set<Integer> committed) {
        PIBTStepResult result = pibtStep(agentId, dir, state, level, committed);
        return new PIBTResult(result.success, result.actions, result.finalState);
    }
    
    /**
     * Try moving blocking agent in alternative directions (perpendicular or opposite).
     */
    private PIBTStepResult tryAlternativeDirections(int agentId, Direction preferredDir, 
            State state, Level level, Set<Integer> committed) {
        
        // Try perpendicular directions first, then opposite
        Direction[] alternatives = getAlternativeDirections(preferredDir);
        
        for (Direction altDir : alternatives) {
            PIBTStepResult result = pibtStep(agentId, altDir, state, level, committed);
            if (result.success) {
                return result;
            }
        }
        
        return new PIBTStepResult(false, Collections.emptyList(), state);
    }
    
    /** Get alternative directions: perpendicular first, then opposite. */
    private Direction[] getAlternativeDirections(Direction dir) {
        switch (dir) {
            case N: return new Direction[] { Direction.E, Direction.W, Direction.S };
            case S: return new Direction[] { Direction.E, Direction.W, Direction.N };
            case E: return new Direction[] { Direction.N, Direction.S, Direction.W };
            case W: return new Direction[] { Direction.N, Direction.S, Direction.E };
            default: return new Direction[] {};
        }
    }
    
    /** Find agent at position, return -1 if none. */
    private int findAgentAt(Position pos, State state) {
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (state.getAgentPosition(i).equals(pos)) {
                return i;
            }
        }
        return -1;
    }
    
    /** Create joint action with one agent moving, others NoOp. */
    private Action[] createJointAction(int agentId, Action action, int numAgents) {
        Action[] jointAction = new Action[numAgents];
        for (int i = 0; i < numAgents; i++) {
            jointAction[i] = (i == agentId) ? action : Action.noOp();
        }
        return jointAction;
    }
    
    /** BFS to find path from start to goal, avoiding walls and boxes (not agents). */
    private List<Direction> findPath(Position start, Position goal, State state, Level level) {
        if (start.equals(goal)) {
            return Collections.emptyList();
        }
        
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Position> parent = new HashMap<>();
        Map<Position, Direction> directionTo = new HashMap<>();
        
        queue.add(start);
        parent.put(start, null);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            if (current.equals(goal)) {
                // Reconstruct path as directions
                List<Direction> path = new ArrayList<>();
                Position p = goal;
                while (!p.equals(start)) {
                    path.add(0, directionTo.get(p));
                    p = parent.get(p);
                }
                return path;
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!parent.containsKey(next) && !level.isWall(next) && !state.hasBoxAt(next)) {
                    parent.put(next, current);
                    directionTo.put(next, dir);
                    queue.add(next);
                }
            }
        }
        
        return null;  // No path found
    }
}
