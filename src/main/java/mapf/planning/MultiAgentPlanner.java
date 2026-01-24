package mapf.planning;

import mapf.domain.*;

import java.util.*;

/**
 * Multi-agent path planner using priority-based planning.
 * 
 * Algorithm:
 * 1. Assign priorities to agents (currently: by agent ID)
 * 2. Plan for agents in priority order
 * 3. Higher priority agents' paths become constraints for lower priority agents
 * 
 * This is a simple but effective approach for many MAPF instances.
 * For more complex scenarios, consider CBS (Conflict-Based Search) or
 * other advanced MAPF algorithms.
 */
public class MultiAgentPlanner {
    
    /** The level being planned for */
    private final Level level;
    
    /** The heuristic to use for search */
    private final Heuristic heuristic;
    
    /** Conflict detector */
    private final ConflictDetector conflictDetector;
    
    /** Computed paths for each agent (action sequences) */
    private final Map<Integer, List<Action>> agentPaths;
    
    /** Current step index in the paths */
    private int currentStep;
    
    /**
     * Creates a new MultiAgentPlanner.
     * 
     * @param level the level to plan for
     * @param heuristic the heuristic to use
     */
    public MultiAgentPlanner(Level level, Heuristic heuristic) {
        this.level = level;
        this.heuristic = heuristic;
        this.conflictDetector = new ConflictDetector();
        this.agentPaths = new HashMap<>();
        this.currentStep = 0;
    }
    
    /**
     * Plans the next actions for all agents.
     * 
     * @param currentState the current state
     * @return array of actions, one per agent
     */
    public Action[] planNextActions(State currentState) {
        int numAgents = level.getNumAgents();
        Action[] actions = new Action[numAgents];
        
        // If we don't have paths or need to replan
        if (agentPaths.isEmpty() || needsReplanning(currentState)) {
            replan(currentState);
            currentStep = 0;
        }
        
        // Get actions from pre-computed paths
        for (int i = 0; i < numAgents; i++) {
            List<Action> path = agentPaths.get(i);
            
            if (path == null || currentStep >= path.size()) {
                actions[i] = Action.noOp();
            } else {
                actions[i] = path.get(currentStep);
            }
        }
        
        // Detect and resolve conflicts
        List<ConflictDetector.Conflict> conflicts = 
            conflictDetector.detectConflicts(currentState, actions, level);
        
        if (!conflicts.isEmpty()) {
            resolveConflicts(conflicts, actions, currentState);
        }
        
        currentStep++;
        return actions;
    }
    
    /**
     * Checks if replanning is needed.
     * 
     * @param currentState the current state
     * @return true if we should replan
     */
    private boolean needsReplanning(State currentState) {
        // Replan if all agents have completed their paths
        boolean allDone = true;
        for (int i = 0; i < level.getNumAgents(); i++) {
            List<Action> path = agentPaths.get(i);
            if (path != null && currentStep < path.size()) {
                allDone = false;
                break;
            }
        }
        
        return allDone && !currentState.isGoalState(level);
    }
    
    /**
     * Replans paths for all agents using priority-based planning.
     * 
     * @param currentState the current state
     */
    private void replan(State currentState) {
        agentPaths.clear();
        
        // Get agent priorities (lower ID = higher priority)
        int[] priorities = getAgentPriorities();
        
        // Plan for each agent in priority order
        for (int priority : priorities) {
            planForAgent(priority, currentState);
        }
    }
    
    /**
     * Gets the planning order for agents.
     * Currently uses agent ID order; override for different priority schemes.
     * 
     * @return array of agent IDs in planning order
     */
    protected int[] getAgentPriorities() {
        int[] priorities = new int[level.getNumAgents()];
        for (int i = 0; i < priorities.length; i++) {
            priorities[i] = i;
        }
        return priorities;
    }
    
    /**
     * Plans a path for a single agent.
     * 
     * @param agentId the agent to plan for
     * @param currentState the current state
     */
    private void planForAgent(int agentId, State currentState) {
        // Create single-agent state focusing on this agent
        AStar astar = new AStar(level, heuristic);
        
        // Search for path
        List<Action> path = astar.search(currentState, agentId);
        
        if (path != null) {
            agentPaths.put(agentId, path);
        } else {
            // If no path found, use empty path (will result in NoOp)
            agentPaths.put(agentId, new ArrayList<>());
            System.err.println("Warning: No path found for agent " + agentId);
        }
    }
    
    /**
     * Resolves conflicts by making lower priority agents wait.
     * 
     * @param conflicts the conflicts to resolve
     * @param actions the action array to modify
     * @param currentState the current state
     */
    private void resolveConflicts(List<ConflictDetector.Conflict> conflicts, 
                                  Action[] actions, State currentState) {
        for (ConflictDetector.Conflict conflict : conflicts) {
            // Lower priority agent (higher ID) waits
            int waitingAgent = Math.max(conflict.agent1, conflict.agent2);
            actions[waitingAgent] = Action.noOp();
            
            System.err.println("Resolved conflict: Agent " + waitingAgent + " waits");
        }
    }
    
    /**
     * Resets the planner, clearing all computed paths.
     */
    public void reset() {
        agentPaths.clear();
        currentStep = 0;
    }
    
    /**
     * Gets the planned path for an agent.
     * 
     * @param agentId the agent ID
     * @return the planned path, or null if not planned
     */
    public List<Action> getAgentPath(int agentId) {
        List<Action> path = agentPaths.get(agentId);
        return path != null ? new ArrayList<>(path) : null;
    }
}
