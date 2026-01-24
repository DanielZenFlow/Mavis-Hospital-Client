package mapf.planning;

import mapf.domain.*;

import java.util.*;

/**
 * Joint-state A* search for multi-agent path finding.
 * 
 * Unlike single-agent A*, this expands joint actions (one action per agent)
 * and searches in the joint state space where all agents move simultaneously.
 * 
 * This is complete and optimal but has exponential complexity in the number of agents.
 * For levels with many agents, consider using CBS or priority-based approaches.
 */
public class JointAStar {
    
    /** The level to search in */
    private final Level level;
    
    /** The heuristic function */
    private final Heuristic heuristic;
    
    /** Maximum number of states to explore */
    private final int maxExploredStates;
    
    /** Conflict detector for validating joint actions */
    private final ConflictDetector conflictDetector;
    
    /**
     * Node in the joint search tree.
     */
    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action[] actions; // Joint action that led to this state
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
    
    public JointAStar(Level level, Heuristic heuristic) {
        this(level, heuristic, SearchConfig.DEFAULT_MAX_STATES);
    }
    
    public JointAStar(Level level, Heuristic heuristic, int maxExploredStates) {
        this.level = level;
        this.heuristic = heuristic;
        this.maxExploredStates = maxExploredStates;
        this.conflictDetector = new ConflictDetector();
    }
    
    /**
     * Searches for a joint plan where all agents move simultaneously.
     * 
     * @param initialState the starting state
     * @return list of joint actions (each Action[] has one action per agent), or null if no solution
     */
    public List<Action[]> search(State initialState) {
        int numAgents = initialState.getNumAgents();
        
        // Handle single-agent case efficiently
        if (numAgents <= 1) {
            return searchSingleAgent(initialState);
        }
        
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Set<State> closedList = new HashSet<>();
        Map<State, SearchNode> stateToNode = new HashMap<>();
        
        int h = heuristic.estimate(initialState, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h);
        openList.add(startNode);
        stateToNode.put(initialState, startNode);
        
        int exploredCount = 0;
        
        while (!openList.isEmpty() && exploredCount < maxExploredStates) {
            SearchNode current = openList.poll();
            stateToNode.remove(current.state);
            
            if (current.state.isGoalState(level)) {
                return reconstructPath(current);
            }
            
            if (closedList.contains(current.state)) {
                continue;
            }
            
            closedList.add(current.state);
            exploredCount++;
            
            if (exploredCount % SearchConfig.PROGRESS_LOG_INTERVAL == 0) {
                System.err.println("JointA* explored " + exploredCount + " states, open=" + openList.size());
            }
            
            // Generate all valid joint actions
            List<Action[]> jointActions = generateJointActions(current.state, numAgents);
            
            for (Action[] jointAction : jointActions) {
                State newState = applyJointAction(current.state, jointAction);
                
                if (newState == null || closedList.contains(newState)) {
                    continue;
                }
                
                int newG = current.g + 1;
                int newH = heuristic.estimate(newState, level);
                
                SearchNode existingNode = stateToNode.get(newState);
                if (existingNode != null && existingNode.g <= newG) {
                    continue;
                }
                
                SearchNode newNode = new SearchNode(newState, current, jointAction, newG, newH);
                openList.add(newNode);
                stateToNode.put(newState, newNode);
            }
        }
        
        System.err.println("JointA* explored " + exploredCount + " states without finding solution");
        return null;
    }
    
    /**
     * Optimized search for single agent.
     */
    private List<Action[]> searchSingleAgent(State initialState) {
        AStar singleAStar = new AStar(level, heuristic, maxExploredStates);
        List<Action> path = singleAStar.search(initialState, 0);
        
        if (path == null) {
            return null;
        }
        
        // Convert to joint action format
        List<Action[]> jointPath = new ArrayList<>();
        for (Action action : path) {
            jointPath.add(new Action[] { action });
        }
        return jointPath;
    }
    
    /**
     * Generates all valid joint actions for all agents.
     * Uses pruning to reduce the exponential branching factor.
     */
    private List<Action[]> generateJointActions(State state, int numAgents) {
        // Get applicable actions for each agent
        List<List<Action>> agentActions = new ArrayList<>();
        
        for (int i = 0; i < numAgents; i++) {
            List<Action> applicable = getApplicableActions(state, i);
            agentActions.add(applicable);
        }
        
        // Generate combinations (Cartesian product)
        List<Action[]> jointActions = new ArrayList<>();
        generateCombinations(agentActions, 0, new Action[numAgents], jointActions, state);
        
        return jointActions;
    }
    
    /**
     * Gets applicable actions for a single agent.
     */
    private List<Action> getApplicableActions(State state, int agentId) {
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
    
    /**
     * Recursively generates combinations of actions and filters for conflicts.
     */
    private void generateCombinations(List<List<Action>> agentActions, int agentIdx,
                                      Action[] current, List<Action[]> results, State state) {
        if (agentIdx == agentActions.size()) {
            // Check for conflicts
            if (conflictDetector.hasNoConflicts(state, current, level)) {
                results.add(current.clone());
            }
            return;
        }
        
        for (Action action : agentActions.get(agentIdx)) {
            current[agentIdx] = action;
            generateCombinations(agentActions, agentIdx + 1, current, results, state);
        }
    }
    
    /**
     * Applies a joint action to a state, returning null if any conflicts occur.
     */
    private State applyJointAction(State state, Action[] actions) {
        State current = state;
        
        // Apply actions sequentially (order doesn't matter for valid joint actions)
        for (int i = 0; i < actions.length; i++) {
            if (actions[i].type != Action.ActionType.NOOP) {
                current = current.apply(actions[i], i);
            }
        }
        
        return current;
    }
    
    /**
     * Reconstructs the path from the goal node.
     */
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
}
