package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
import mapf.planning.coordination.ConflictDetector;
import mapf.planning.heuristic.Heuristic;

import java.util.*;

/**
 * Joint-state A* search strategy for multi-agent path finding.
 * Searches in joint configuration space where all agents move simultaneously.
 * 
 * Optimal and complete, but has exponential complexity O(b^(d*n)) where:
 * - b = branching factor per agent
 * - d = solution depth
 * - n = number of agents
 * 
 * Best for 2-3 agents. For more agents, use PriorityPlanningStrategy.
 */
public class JointAStarStrategy implements SearchStrategy {
    
    private final Heuristic heuristic;
    private final SearchConfig config;
    private final ConflictDetector conflictDetector;
    private long timeoutMs;
    private int maxStates;
    private double weight;
    
    public JointAStarStrategy(Heuristic heuristic, SearchConfig config) {
        this.heuristic = heuristic;
        this.config = config;
        this.conflictDetector = new ConflictDetector();
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
        this.weight = config.getAstarWeight();
    }
    
    @Override
    public String getName() {
        return weight > 1.0 ? "Joint Weighted A* (w=" + weight + ")" : "Joint A*";
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
        int numAgents = initialState.getNumAgents();
        
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
                long elapsed = System.currentTimeMillis() - startTime;
                System.err.println(getName() + ": Solution found after " + exploredCount + 
                        " states in " + elapsed + "ms");
                return reconstructPath(current);
            }
            
            if (closedList.contains(current.state)) {
                continue;
            }
            
            closedList.add(current.state);
            exploredCount++;
            
            if (exploredCount % SearchConfig.PROGRESS_LOG_INTERVAL == 0) {
                System.err.println(getName() + ": Explored " + exploredCount + 
                        " states, open=" + openList.size());
            }
            
            // Generate all valid joint actions
            List<Action[]> jointActions = generateJointActions(current.state, numAgents, level);
            
            for (Action[] jointAction : jointActions) {
                State newState = applyJointAction(current.state, jointAction);
                
                if (newState == null || closedList.contains(newState)) {
                    continue;
                }
                
                int newG = current.g + 1;
                int newH = (int)(weight * heuristic.estimate(newState, level));
                
                SearchNode existingNode = stateToNode.get(newState);
                if (existingNode != null && existingNode.g <= newG) {
                    continue;
                }
                
                SearchNode newNode = new SearchNode(newState, current, jointAction, newG, newH);
                openList.add(newNode);
                stateToNode.put(newState, newNode);
            }
        }
        
        System.err.println(getName() + ": No solution found after " + exploredCount + " states");
        return null;
    }
    
    private List<Action[]> generateJointActions(State state, int numAgents, Level level) {
        // Get applicable actions for each agent
        List<List<Action>> agentActions = new ArrayList<>();
        
        for (int i = 0; i < numAgents; i++) {
            agentActions.add(getApplicableActions(state, i, level));
        }
        
        // Generate combinations with conflict filtering
        List<Action[]> jointActions = new ArrayList<>();
        generateCombinations(agentActions, 0, new Action[numAgents], jointActions, state, level);
        
        return jointActions;
    }
    
    private List<Action> getApplicableActions(State state, int agentId, Level level) {
        List<Action> applicable = new ArrayList<>();
        
        // Always include NoOp
        applicable.add(Action.noOp());
        
        // Move actions
        for (Direction dir : Direction.values()) {
            Action action = Action.move(dir);
            if (state.isApplicable(action, agentId, level)) {
                applicable.add(action);
            }
        }
        
        // Push actions
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                Action action = Action.push(agentDir, boxDir);
                if (state.isApplicable(action, agentId, level)) {
                    applicable.add(action);
                }
            }
        }
        
        // Pull actions
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                Action action = Action.pull(agentDir, boxDir);
                if (state.isApplicable(action, agentId, level)) {
                    applicable.add(action);
                }
            }
        }
        
        return applicable;
    }
    
    private void generateCombinations(List<List<Action>> agentActions, int agentIdx,
                                      Action[] current, List<Action[]> results, 
                                      State state, Level level) {
        if (agentIdx == agentActions.size()) {
            // Check for conflicts
            if (conflictDetector.hasNoConflicts(state, current, level)) {
                results.add(current.clone());
            }
            return;
        }
        
        for (Action action : agentActions.get(agentIdx)) {
            current[agentIdx] = action;
            generateCombinations(agentActions, agentIdx + 1, current, results, state, level);
        }
    }
    
    private State applyJointAction(State state, Action[] actions) {
        State current = state;
        for (int i = 0; i < actions.length; i++) {
            if (actions[i].type != Action.ActionType.NOOP) {
                current = current.apply(actions[i], i);
            }
        }
        return current;
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
