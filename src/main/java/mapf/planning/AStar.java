package mapf.planning;

import mapf.domain.*;

import java.util.*;

/**
 * A* search algorithm implementation for single-agent pathfinding.
 * 
 * A* is a best-first search algorithm that uses f(n) = g(n) + h(n) where:
 * - g(n) is the cost from the start to node n
 * - h(n) is the heuristic estimate from n to the goal
 * 
 * With an admissible heuristic, A* guarantees finding the optimal solution.
 */
public class AStar {
    
    /** The level to search in */
    private final Level level;
    
    /** The heuristic function */
    private final Heuristic heuristic;
    
    /** Maximum number of states to explore (to prevent infinite loops) */
    private final int maxExploredStates;
    
    /**
     * Node in the search tree, containing state and path cost information.
     */
    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action action;
        final int agentId;
        final int g; // Cost from start to this node
        final int f; // f = g + h (total estimated cost)
        
        SearchNode(State state, SearchNode parent, Action action, int agentId, int g, int h) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.agentId = agentId;
            this.g = g;
            this.f = g + h;
        }
        
        @Override
        public int compareTo(SearchNode other) {
            // Primary: lower f value is better
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0) return fCompare;
            
            // Tie-breaker: higher g value (prefer nodes closer to goal)
            return Integer.compare(other.g, this.g);
        }
    }
    
    /**
     * Creates a new A* search instance.
     * 
     * @param level the level to search in
     * @param heuristic the heuristic function
     */
    public AStar(Level level, Heuristic heuristic) {
        this(level, heuristic, SearchConfig.DEFAULT_MAX_STATES);
    }
    
    /**
     * Creates a new A* search instance with custom state limit.
     * 
     * @param level the level to search in
     * @param heuristic the heuristic function
     * @param maxExploredStates maximum states to explore
     */
    public AStar(Level level, Heuristic heuristic, int maxExploredStates) {
        this.level = level;
        this.heuristic = heuristic;
        this.maxExploredStates = maxExploredStates;
    }
    
    /**
     * Searches for a plan from the initial state to the goal state.
     * 
     * @param initialState the starting state
     * @return list of actions to reach the goal, or null if no solution found
     */
    public List<Action> search(State initialState) {
        return search(initialState, 0); // Default to agent 0 for single-agent
    }
    
    /**
     * Searches for a plan for a specific agent.
     * 
     * @param initialState the starting state
     * @param agentId the agent to plan for
     * @return list of actions to reach the goal, or null if no solution found
     */
    public List<Action> search(State initialState, int agentId) {
        // Priority queue ordered by f value
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        
        // Set of explored states
        Set<State> closedList = new HashSet<>();
        
        // Map for finding nodes in open list (for potential updates)
        Map<State, SearchNode> stateToNode = new HashMap<>();
        
        // Initialize with start node
        int h = heuristic.estimate(initialState, level);
        SearchNode startNode = new SearchNode(initialState, null, null, agentId, 0, h);
        openList.add(startNode);
        stateToNode.put(initialState, startNode);
        
        int exploredCount = 0;
        
        while (!openList.isEmpty() && exploredCount < maxExploredStates) {
            SearchNode current = openList.poll();
            stateToNode.remove(current.state);
            
            // Check if goal reached
            if (current.state.isGoalState(level)) {
                return reconstructPath(current);
            }
            
            // Skip if already explored
            if (closedList.contains(current.state)) {
                continue;
            }
            
            closedList.add(current.state);
            exploredCount++;
            
            // Expand successors
            for (Map.Entry<Action, State> successor : current.state.getSuccessors(agentId, level)) {
                Action action = successor.getKey();
                State newState = successor.getValue();
                
                // Skip if already explored
                if (closedList.contains(newState)) {
                    continue;
                }
                
                int newG = current.g + 1; // Uniform cost per action
                int newH = heuristic.estimate(newState, level);
                
                // Check if we have a better path to this state
                SearchNode existingNode = stateToNode.get(newState);
                if (existingNode != null && existingNode.g <= newG) {
                    continue; // Existing path is better or equal
                }
                
                SearchNode newNode = new SearchNode(newState, current, action, agentId, newG, newH);
                openList.add(newNode);
                stateToNode.put(newState, newNode);
            }
        }
        
        // No solution found
        System.err.println("A* explored " + exploredCount + " states without finding solution");
        return null;
    }
    
    /**
     * Reconstructs the path from start to goal by following parent pointers.
     * 
     * @param goalNode the goal node
     * @return list of actions from start to goal
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
     * Gets the number of states in the closed list after the last search.
     * Useful for debugging and performance analysis.
     * 
     * @return number of explored states
     */
    public int getLastSearchExploredCount() {
        // This would require storing the count from the last search
        // For now, this is a placeholder
        throw new UnsupportedOperationException("Not yet implemented");
    }
}
