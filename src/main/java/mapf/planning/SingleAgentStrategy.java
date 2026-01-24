package mapf.planning;

import mapf.domain.*;
import java.util.*;

/**
 * Single-agent search strategy using A* with optional weighting.
 * Optimized for levels with only one agent.
 */
public class SingleAgentStrategy implements SearchStrategy {
    
    private final Heuristic heuristic;
    private final SearchConfig config;
    private long timeoutMs;
    private int maxStates;
    private double weight;
    
    public SingleAgentStrategy(Heuristic heuristic, SearchConfig config) {
        this.heuristic = heuristic;
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
        this.weight = config.getAstarWeight();
    }
    
    @Override
    public String getName() {
        return weight > 1.0 ? "Weighted A* (w=" + weight + ")" : "A*";
    }
    
    @Override
    public void setTimeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    public void setMaxStates(int maxStates) {
        this.maxStates = maxStates;
    }
    
    public void setWeight(double weight) {
        this.weight = weight;
    }
    
    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Set<State> closedList = new HashSet<>();
        Map<State, SearchNode> stateToNode = new HashMap<>();
        
        int h = (int)(weight * heuristic.estimate(initialState, level));
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h);
        openList.add(startNode);
        stateToNode.put(initialState, startNode);
        
        int exploredCount = 0;
        
        while (!openList.isEmpty() && exploredCount < maxStates) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                System.err.println(getName() + ": Timeout after " + exploredCount + " states");
                return null;
            }
            
            SearchNode current = openList.poll();
            stateToNode.remove(current.state);
            
            if (current.state.isGoalState(level)) {
                System.err.println(getName() + ": Solution found after " + exploredCount + " states");
                return reconstructPath(current);
            }
            
            if (closedList.contains(current.state)) {
                continue;
            }
            
            closedList.add(current.state);
            exploredCount++;
            
            // Expand for agent 0
            for (Map.Entry<Action, State> successor : current.state.getSuccessors(0, level)) {
                Action action = successor.getKey();
                State newState = successor.getValue();
                
                if (closedList.contains(newState)) {
                    continue;
                }
                
                int newG = current.g + 1;
                int newH = (int)(weight * heuristic.estimate(newState, level));
                
                SearchNode existingNode = stateToNode.get(newState);
                if (existingNode != null && existingNode.g <= newG) {
                    continue;
                }
                
                SearchNode newNode = new SearchNode(newState, current, 
                        new Action[] { action }, newG, newH);
                openList.add(newNode);
                stateToNode.put(newState, newNode);
            }
        }
        
        System.err.println(getName() + ": No solution found after " + exploredCount + " states");
        return null;
    }
    
    private List<Action[]> reconstructPath(SearchNode goalNode) {
        List<Action[]> path = new ArrayList<>();
        SearchNode current = goalNode;
        
        while (current.parent != null) {
            path.add(current.actions);
            current = current.parent;
        }
        
        Collections.reverse(path);
        return path;
    }
    
    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action[] actions;
        final int g;
        final int f;
        
        SearchNode(State state, SearchNode parent, Action[] actions, int g, int h) {
            this.state = state;
            this.parent = parent;
            this.actions = actions;
            this.g = g;
            this.f = g + h;
        }
        
        @Override
        public int compareTo(SearchNode other) {
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0) return fCompare;
            return Integer.compare(other.g, this.g);
        }
    }
}
