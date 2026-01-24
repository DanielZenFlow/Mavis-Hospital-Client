package mapf.planning;

import mapf.domain.*;
import java.util.*;

/**
 * Priority-based planning strategy for multi-agent path finding.
 * 
 * Algorithm (as described in ARCHITECTURE.md Option A):
 * 1. Assign priorities to agents
 * 2. Plan for each agent independently using A*
 * 3. Higher priority agents' paths become constraints for lower priority agents
 * 4. Detect and resolve conflicts at execution time
 * 
 * Scalable to many agents O(n * b^d), but may produce suboptimal solutions.
 * May fail on tightly-coupled scenarios where agents must coordinate closely.
 */
public class PriorityPlanningStrategy implements SearchStrategy {
    
    private final Heuristic heuristic;
    private final SearchConfig config;
    private final ConflictDetector conflictDetector;
    private long timeoutMs;
    private int maxStates;
    
    public PriorityPlanningStrategy(Heuristic heuristic, SearchConfig config) {
        this.heuristic = heuristic;
        this.config = config;
        this.conflictDetector = new ConflictDetector();
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
    }
    
    @Override
    public String getName() {
        return "Priority Planning";
    }
    
    @Override
    public void setTimeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    public void setMaxStates(int maxStates) {
        this.maxStates = maxStates;
    }
    
    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        int numAgents = initialState.getNumAgents();
        
        System.err.println(getName() + ": Planning for " + numAgents + " agents");
        
        // Assign priorities (can be improved with better heuristics)
        int[] priorities = assignPriorities(initialState, level);
        
        // Plan paths for each agent
        Map<Integer, List<Action>> agentPaths = new HashMap<>();
        
        for (int priority = 0; priority < numAgents; priority++) {
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                System.err.println(getName() + ": Timeout during planning");
                return null;
            }
            
            int agentId = priorities[priority];
            List<Action> path = planSingleAgentPath(agentId, initialState, level, agentPaths);
            
            if (path == null) {
                System.err.println(getName() + ": Failed to find path for agent " + agentId);
                // Use empty path (NoOp) as fallback
                path = new ArrayList<>();
            }
            
            agentPaths.put(agentId, path);
            System.err.println(getName() + ": Agent " + agentId + " path length: " + path.size());
        }
        
        // Merge paths into joint actions
        List<Action[]> jointPlan = mergePaths(agentPaths, numAgents, initialState, level);
        
        System.err.println(getName() + ": Total plan length: " + jointPlan.size());
        return jointPlan;
    }
    
    /**
     * Assigns priorities to agents. Current strategy: agents with more work to do
     * (farther from goal, more boxes to move) get higher priority.
     */
    private int[] assignPriorities(State state, Level level) {
        int numAgents = state.getNumAgents();
        
        // Calculate "work" score for each agent
        List<int[]> agentScores = new ArrayList<>();
        for (int i = 0; i < numAgents; i++) {
            int score = calculateAgentWork(i, state, level);
            agentScores.add(new int[] { i, score });
        }
        
        // Sort by work score (descending - more work = higher priority)
        agentScores.sort((a, b) -> Integer.compare(b[1], a[1]));
        
        int[] priorities = new int[numAgents];
        for (int i = 0; i < numAgents; i++) {
            priorities[i] = agentScores.get(i)[0];
        }
        
        return priorities;
    }
    
    /**
     * Estimates how much work an agent needs to do.
     */
    private int calculateAgentWork(int agentId, State state, Level level) {
        int work = 0;
        Color agentColor = level.getAgentColor(agentId);
        
        // Count boxes this agent needs to move
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            char boxType = box.getValue();
            if (level.getBoxColor(boxType) == agentColor) {
                // Check if box is at goal
                Position boxPos = box.getKey();
                if (level.getBoxGoal(boxPos) != boxType) {
                    work += SearchConfig.BOX_NOT_AT_GOAL_WORK_SCORE;
                }
            }
        }
        
        return work;
    }
    
    /**
     * Plans a path for a single agent, considering higher-priority agents' paths as constraints.
     */
    private List<Action> planSingleAgentPath(int agentId, State initialState, Level level,
                                              Map<Integer, List<Action>> existingPaths) {
        // Use single-agent A* with constraint checking
        PriorityQueue<AgentSearchNode> openList = new PriorityQueue<>();
        Map<AgentState, AgentSearchNode> visited = new HashMap<>();
        
        Position startPos = initialState.getAgentPosition(agentId);
        AgentState startAgentState = new AgentState(startPos, initialState.getBoxes(), 0);
        
        int h = estimateSingleAgentCost(agentId, initialState, level);
        AgentSearchNode startNode = new AgentSearchNode(startAgentState, initialState, null, null, 0, h);
        openList.add(startNode);
        visited.put(startAgentState, startNode);
        
        int maxSingleAgentStates = maxStates / Math.max(1, initialState.getNumAgents());
        int exploredCount = 0;
        
        while (!openList.isEmpty() && exploredCount < maxSingleAgentStates) {
            AgentSearchNode current = openList.poll();
            
            // Check if this agent's goals are satisfied
            if (isAgentGoalSatisfied(agentId, current.fullState, level)) {
                return reconstructSinglePath(current);
            }
            
            exploredCount++;
            
            // Expand
            for (Action action : getAllActions()) {
                if (!current.fullState.isApplicable(action, agentId, level)) {
                    continue;
                }
                
                State newFullState = current.fullState.apply(action, agentId);
                int newTime = current.agentState.time + 1;
                
                // Check constraints from higher-priority agents
                if (violatesConstraints(agentId, newFullState, newTime, existingPaths, level)) {
                    continue;
                }
                
                AgentState newAgentState = new AgentState(
                        newFullState.getAgentPosition(agentId),
                        newFullState.getBoxes(),
                        newTime);
                
                int newG = current.g + 1;
                int newH = estimateSingleAgentCost(agentId, newFullState, level);
                
                AgentSearchNode existingNode = visited.get(newAgentState);
                if (existingNode != null && existingNode.g <= newG) {
                    continue;
                }
                
                AgentSearchNode newNode = new AgentSearchNode(
                        newAgentState, newFullState, current, action, newG, newH);
                openList.add(newNode);
                visited.put(newAgentState, newNode);
            }
        }
        
        return null; // No path found
    }
    
    /**
     * Checks if an agent's goals are satisfied.
     */
    private boolean isAgentGoalSatisfied(int agentId, State state, Level level) {
        Color agentColor = level.getAgentColor(agentId);
        
        // Check all box goals for this agent's color
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0' && level.getBoxColor(goalType) == agentColor) {
                    Position pos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(pos);
                    if (actualBox == null || actualBox != goalType) {
                        return false;
                    }
                }
            }
        }
        
        // Check agent goal position (if any)
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    Position goalPos = new Position(row, col);
                    if (!state.getAgentPosition(agentId).equals(goalPos)) {
                        return false;
                    }
                }
            }
        }
        
        return true;
    }
    
    /**
     * Estimates cost for single agent to reach its goals.
     */
    private int estimateSingleAgentCost(int agentId, State state, Level level) {
        int cost = 0;
        Color agentColor = level.getAgentColor(agentId);
        Position agentPos = state.getAgentPosition(agentId);
        
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            char boxType = box.getValue();
            if (level.getBoxColor(boxType) == agentColor) {
                Position boxPos = box.getKey();
                // Find closest goal for this box type
                int minDist = Integer.MAX_VALUE;
                for (int r = 0; r < level.getRows(); r++) {
                    for (int c = 0; c < level.getCols(); c++) {
                        if (level.getBoxGoal(r, c) == boxType) {
                            int dist = boxPos.manhattanDistance(new Position(r, c));
                            minDist = Math.min(minDist, dist);
                        }
                    }
                }
                if (minDist < Integer.MAX_VALUE) {
                    cost += minDist;
                }
            }
        }
        
        return cost;
    }
    
    /**
     * Checks if a state violates constraints from higher-priority agents.
     */
    private boolean violatesConstraints(int agentId, State state, int time,
                                        Map<Integer, List<Action>> existingPaths, Level level) {
        Position agentPos = state.getAgentPosition(agentId);
        
        for (Map.Entry<Integer, List<Action>> entry : existingPaths.entrySet()) {
            int otherAgentId = entry.getKey();
            List<Action> otherPath = entry.getValue();
            
            // Get other agent's position at this time
            Position otherPos = getAgentPositionAtTime(otherAgentId, time, otherPath, level);
            
            // Check vertex conflict
            if (agentPos.equals(otherPos)) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * Gets an agent's position at a specific time given their path.
     */
    private Position getAgentPositionAtTime(int agentId, int time, List<Action> path, Level level) {
        // This is simplified - would need to track full state evolution
        // For now, just return a default position
        return null; // Simplified constraint checking
    }
    
    private List<Action> getAllActions() {
        List<Action> actions = new ArrayList<>();
        actions.add(Action.noOp());
        
        for (Direction dir : Direction.values()) {
            actions.add(Action.move(dir));
        }
        
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                actions.add(Action.push(agentDir, boxDir));
                actions.add(Action.pull(agentDir, boxDir));
            }
        }
        
        return actions;
    }
    
    private List<Action> reconstructSinglePath(AgentSearchNode goalNode) {
        List<Action> path = new ArrayList<>();
        AgentSearchNode current = goalNode;
        
        while (current.parent != null) {
            path.add(current.action);
            current = current.parent;
        }
        
        Collections.reverse(path);
        return path;
    }
    
    /**
     * Merges individual agent paths into joint actions.
     */
    private List<Action[]> mergePaths(Map<Integer, List<Action>> agentPaths, int numAgents,
                                       State initialState, Level level) {
        // Find maximum path length
        int maxLength = 0;
        for (List<Action> path : agentPaths.values()) {
            maxLength = Math.max(maxLength, path.size());
        }
        
        // If no paths, return empty
        if (maxLength == 0) {
            return new ArrayList<>();
        }
        
        List<Action[]> jointPlan = new ArrayList<>();
        State currentState = initialState;
        
        for (int t = 0; t < maxLength; t++) {
            Action[] jointAction = new Action[numAgents];
            
            for (int a = 0; a < numAgents; a++) {
                List<Action> path = agentPaths.get(a);
                if (path != null && t < path.size()) {
                    jointAction[a] = path.get(t);
                } else {
                    jointAction[a] = Action.noOp();
                }
            }
            
            // Resolve conflicts
            List<ConflictDetector.Conflict> conflicts = 
                    conflictDetector.detectConflicts(currentState, jointAction, level);
            
            for (ConflictDetector.Conflict conflict : conflicts) {
                // Lower priority agent (higher ID in simple case) waits
                int waitingAgent = Math.max(conflict.agent1, conflict.agent2);
                jointAction[waitingAgent] = Action.noOp();
            }
            
            jointPlan.add(jointAction);
            
            // Update state
            for (int a = 0; a < numAgents; a++) {
                if (jointAction[a].type != Action.ActionType.NOOP) {
                    if (currentState.isApplicable(jointAction[a], a, level)) {
                        currentState = currentState.apply(jointAction[a], a);
                    }
                }
            }
        }
        
        return jointPlan;
    }
    
    /**
     * State representation for single-agent search with time.
     */
    private static class AgentState {
        final Position agentPos;
        final Map<Position, Character> boxes;
        final int time;
        
        AgentState(Position agentPos, Map<Position, Character> boxes, int time) {
            this.agentPos = agentPos;
            this.boxes = boxes;
            this.time = time;
        }
        
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof AgentState)) return false;
            AgentState other = (AgentState) obj;
            return agentPos.equals(other.agentPos) && 
                   boxes.equals(other.boxes) && 
                   time == other.time;
        }
        
        @Override
        public int hashCode() {
            return Objects.hash(agentPos, boxes, time);
        }
    }
    
    private static class AgentSearchNode implements Comparable<AgentSearchNode> {
        final AgentState agentState;
        final State fullState;
        final AgentSearchNode parent;
        final Action action;
        final int g;
        final int f;
        
        AgentSearchNode(AgentState agentState, State fullState, AgentSearchNode parent,
                        Action action, int g, int h) {
            this.agentState = agentState;
            this.fullState = fullState;
            this.parent = parent;
            this.action = action;
            this.g = g;
            this.f = g + h;
        }
        
        @Override
        public int compareTo(AgentSearchNode other) {
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0) return fCompare;
            return Integer.compare(other.g, this.g);
        }
    }
}
