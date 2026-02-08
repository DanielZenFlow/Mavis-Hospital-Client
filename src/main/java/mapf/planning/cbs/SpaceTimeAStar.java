package mapf.planning.cbs;

import mapf.domain.*;
import mapf.planning.*;
import mapf.planning.heuristic.Heuristic;

import java.util.*;

/**
 * Space-Time A* search for CBS low-level planning.
 * 
 * Unlike standard A*, this search operates in space-time coordinates (x, y, t)
 * and respects time-indexed constraints.
 * 
 * The search finds a path for a single agent that:
 * 1. Reaches the goal state
 * 2. Does not violate any constraints
 */
public class SpaceTimeAStar {
    
    private final Level level;
    private final Heuristic heuristic;
    private final int agentId;
    private final Set<CBSStrategy.Constraint> constraints;
    private final Position boxGoalPos;
    private final Character boxGoalType;
    
    /**
     * Node in space-time search.
     */
    private static class STNode implements Comparable<STNode> {
        final State state;
        final STNode parent;
        final Action action;
        final int time;      // Timestep
        final int g;         // Cost from start
        final int f;         // f = g + h
        
        STNode(State state, STNode parent, Action action, int time, int g, int h) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.time = time;
            this.g = g;
            this.f = g + h;
        }
        
        @Override
        public int compareTo(STNode other) {
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0) return fCompare;
            return Integer.compare(other.g, this.g);
        }
        
        /**
         * Key for closed list: (state, time) pair.
         */
        public String getKey() {
            return state.hashCode() + "_" + time;
        }
    }
    
    public SpaceTimeAStar(Level level, Heuristic heuristic, int agentId, 
                          Set<CBSStrategy.Constraint> constraints) {
        this(level, heuristic, agentId, constraints, null, null);
    }

    public SpaceTimeAStar(Level level, Heuristic heuristic, int agentId, 
                          Set<CBSStrategy.Constraint> constraints, Position boxGoalPos, Character boxGoalType) {
        this.level = level;
        this.heuristic = heuristic;
        this.agentId = agentId;
        this.constraints = constraints;
        this.boxGoalPos = boxGoalPos;
        this.boxGoalType = boxGoalType;
    }
    
    /**
     * Searches for a path that satisfies all constraints.
     * 
     * @param initialState the starting state
     * @param maxTime maximum timestep to search to (prevents infinite loops)
     * @return list of states representing the path, or null if no solution
     */
    public List<State> search(State initialState, int maxTime) {
        PriorityQueue<STNode> openList = new PriorityQueue<>();
        Set<String> closedList = new HashSet<>();
        
        int h = heuristic.estimate(initialState, level);
        STNode startNode = new STNode(initialState, null, null, 0, 0, h);
        openList.add(startNode);
        
        int exploredCount = 0;
        int maxExplored = SearchConfig.DEFAULT_MAX_STATES / 10; // Less budget for low-level
        
        while (!openList.isEmpty() && exploredCount < maxExplored) {
            STNode current = openList.poll();
            
            // Check if goal reached
            if (isGoalForAgent(current.state)) {
                return reconstructPath(current);
            }
            
            String key = current.getKey();
            if (closedList.contains(key)) {
                continue;
            }
            closedList.add(key);
            exploredCount++;
            
            // Don't explore beyond max time
            if (current.time >= maxTime) {
                continue;
            }
            
            // Generate successors
            for (Map.Entry<Action, State> successor : current.state.getSuccessors(agentId, level)) {
                Action action = successor.getKey();
                State newState = successor.getValue();
                int newTime = current.time + 1;
                
                // Check constraints
                if (violatesConstraint(newState, newTime)) {
                    continue;
                }
                
                String newKey = newState.hashCode() + "_" + newTime;
                if (closedList.contains(newKey)) {
                    continue;
                }
                
                int newG = current.g + 1;
                int newH = heuristic.estimate(newState, level);
                
                STNode newNode = new STNode(newState, current, action, newTime, newG, newH);
                openList.add(newNode);
            }
            
            // Also consider waiting (NoOp) - agent stays in place
            State waitState = current.state; // Same state
            int waitTime = current.time + 1;
            
            if (!violatesConstraint(waitState, waitTime)) {
                String waitKey = waitState.hashCode() + "_" + waitTime;
                if (!closedList.contains(waitKey)) {
                    int waitG = current.g + 1;
                    int waitH = heuristic.estimate(waitState, level);
                    STNode waitNode = new STNode(waitState, current, Action.noOp(), waitTime, waitG, waitH);
                    openList.add(waitNode);
                }
            }
        }
        
        return null; // No path found
    }
    
    /**
     * Checks if the agent has reached its goals in this state.
     * Supports both Box Goals (pushing box to target) and Agent Goals (position).
     */
    private boolean isGoalForAgent(State state) {
        // 1. If assigned a Box Goal, check if satisfied
        if (boxGoalPos != null && boxGoalType != null) {
            return boxGoalType.equals(state.getBoxes().get(boxGoalPos));
        }

        // 2. Fallback: Check Agent Position Goal
        Position agentPos = state.getAgentPosition(agentId);
        
        // Scan for this agent's goal position
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    Position goalPos = Position.of(row, col);
                    return agentPos.equals(goalPos);
                }
            }
        }
        
        // No goals at all for this agent -> assumed done at start
        return true;
    }
    
    /**
     * Checks if a state at a given time violates any constraint.
     */
    private boolean violatesConstraint(State state, int time) {
        Position agentPos = state.getAgentPosition(agentId);
        
        // Check constraints against Agent Body
        if (isConstrained(agentPos, time)) return true;

        // Check constraints against Boxes
        // If the agent places a box at a position that is constrained for this agent, it's a violation.
        for (Position boxPos : state.getBoxes().keySet()) {
             if (isConstrained(boxPos, time)) return true;
        }

        return false;
    }

    private boolean isConstrained(Position pos, int time) {
        for (CBSStrategy.Constraint c : constraints) {
            if (c.agentId == agentId && c.time == time && c.position.equals(pos)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Reconstructs the path from the goal node.
     */
    private List<State> reconstructPath(STNode goalNode) {
        List<State> path = new ArrayList<>();
        STNode current = goalNode;
        
        while (current != null) {
            path.add(current.state);
            current = current.parent;
        }
        
        Collections.reverse(path);
        return path;
    }
}
