package mapf.planning.pathfinding;

import mapf.domain.Action;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.util.List;

/**
 * Interface for path planning algorithms.
 * 
 * Follows ARCHITECTURE.md Strategy pattern - different implementations can be swapped.
 * 
 * Path planners find action sequences to move an agent (optionally with a box) 
 * from current position to a target position.
 */
public interface PathPlanner {
    
    /**
     * Result of a path planning request.
     */
    class PathResult {
        public final boolean found;
        public final List<Action> actions;
        public final int cost;
        
        private PathResult(boolean found, List<Action> actions, int cost) {
            this.found = found;
            this.actions = actions;
            this.cost = cost;
        }
        
        public static PathResult success(List<Action> actions) {
            return new PathResult(true, actions, actions.size());
        }
        
        public static PathResult failure() {
            return new PathResult(false, List.of(), Integer.MAX_VALUE);
        }
        
        @Override
        public String toString() {
            if (found) {
                return "PathResult[cost=" + cost + ", actions=" + actions.size() + "]";
            }
            return "PathResult[NOT FOUND]";
        }
    }
    
    /**
     * Finds a path for an agent to move from current position to target.
     * 
     * @param state Current game state
     * @param level Level information
     * @param agentId Agent to plan for
     * @param target Target position for the agent
     * @return PathResult with actions, or failure
     */
    PathResult findAgentPath(State state, Level level, int agentId, Position target);
    
    /**
     * Finds a path to push/pull a box to a target position.
     * 
     * @param state Current game state
     * @param level Level information
     * @param agentId Agent doing the pushing/pulling
     * @param boxPos Current box position
     * @param boxTarget Target position for the box
     * @return PathResult with actions, or failure
     */
    PathResult findBoxPath(State state, Level level, int agentId, Position boxPos, Position boxTarget);
    
    /**
     * Estimates cost to move agent from A to B (for heuristics).
     * Default implementation uses Manhattan distance.
     */
    default int estimateCost(Position from, Position to) {
        return from.manhattanDistance(to);
    }
}
