package mapf.planning.cbs;

import mapf.domain.*;
import mapf.planning.*;
import mapf.planning.heuristic.Heuristic;

import java.util.*;

/**
 * Conflict-Based Search (CBS) strategy for multi-agent path finding.
 * 
 * CBS is a two-level search algorithm:
 * - High level: Search over constraint tree (CT) nodes
 * - Low level: Single-agent A* with constraints
 * 
 * Reference: Sharon et al. (2015) "Conflict-Based Search for Optimal Multi-Agent Path Finding"
 * 
 * CBS guarantees optimal solutions but can be exponential in the number of conflicts.
 * Best suited for levels where agents are loosely coupled (few conflicts).
 * 
 * Constraints from PRODUCT.md:
 * - Time limit: 3 minutes (180000 ms)
 * - Action limit: 20,000 joint actions
 */
public class CBSStrategy implements SearchStrategy {
    
    private final Heuristic heuristic;
    private final SearchConfig config;
    private long timeoutMs = SearchConfig.DEFAULT_TIMEOUT_MS;
    private int maxStates = SearchConfig.DEFAULT_MAX_STATES;
    
    // Statistics
    private int highLevelNodesExpanded = 0;
    private int lowLevelSearches = 0;
    
    // Task Assignment
    private Map<Integer, Task> globalTaskAssignment;
    
    private static class Task {
        final Position goalPos;
        final Character boxType;
        Task(Position g, Character t) { goalPos = g; boxType = t; }
    }
    
    public CBSStrategy(Heuristic heuristic) {
        this(heuristic, new SearchConfig());
    }
    
    public CBSStrategy(Heuristic heuristic, SearchConfig config) {
        this.heuristic = heuristic;
        this.config = config;
    }
    
    @Override
    public String getName() {
        return "CBS (Conflict-Based Search)";
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
        
        System.err.println(getName() + ": Starting search with " + numAgents + " agents");
        
        // 1. Assign Tasks
        this.globalTaskAssignment = assignTasks(initialState, level);
        
        // Priority queue for CT nodes, ordered by solution cost (sum of path lengths)
        PriorityQueue<CTNode> openList = new PriorityQueue<>();
        
        // Create root node with no constraints
        CTNode root = createRootNode(initialState, level, numAgents);
        if (root == null) {
            System.err.println(getName() + ": Failed to find initial paths for all agents");
            return null;
        }
        
        openList.add(root);
        highLevelNodesExpanded = 0;
        
        while (!openList.isEmpty()) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                System.err.println(getName() + ": Timeout after " + highLevelNodesExpanded + " CT nodes");
                return null;
            }
            
            // Check node limit
            if (highLevelNodesExpanded >= maxStates) {
                System.err.println(getName() + ": Reached node limit");
                return null;
            }
            
            CTNode current = openList.poll();
            highLevelNodesExpanded++;
            
            if (highLevelNodesExpanded % 100 == 0) {
                System.err.println(getName() + ": Expanded " + highLevelNodesExpanded + 
                    " CT nodes, cost=" + current.cost + ", open=" + openList.size());
            }
            
            // Find first conflict in the current solution
            Conflict conflict = findFirstConflict(current, level);
            
            if (conflict == null) {
                // No conflicts!
                System.err.println(getName() + ": Solution found! Cost=" + current.cost);
                return convertToJointActions(current, numAgents);
            }
            
            // Branch on the conflict
            for (int agentId : new int[]{conflict.agent1, conflict.agent2}) {
                Constraint newConstraint = createConstraint(conflict, agentId);
                CTNode child = createChildNode(current, newConstraint, agentId, initialState, level);
                
                if (child != null) {
                    openList.add(child);
                }
            }
        }
        
        System.err.println(getName() + ": No solution found after " + highLevelNodesExpanded + " CT nodes");
        return null;
    }

    private Map<Integer, Task> assignTasks(State state, Level level) {
        Map<Integer, Task> assignments = new HashMap<>();
        Set<Integer> assignedAgents = new HashSet<>();
        
        // Find all box goals
        Map<Character, List<Position>> goalsByType = new HashMap<>();
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                char g = level.getBoxGoal(r, c);
                if (g != '\0') {
                    // Skip if already satisfied
                    Position p = new Position(r, c);
                    if (state.getBoxes().containsKey(p) && state.getBoxes().get(p) == g) continue;
                    goalsByType.computeIfAbsent(g, k -> new ArrayList<>()).add(p);
                }
            }
        }
        
        // Naive Greedy Assignment: Iterate goals, find nearest box, nearest agent OF MATCHING COLOR
        for (Map.Entry<Character, List<Position>> entry : goalsByType.entrySet()) {
            char type = entry.getKey();
            for (Position goalVal : entry.getValue()) {
                // Find nearest box of this type
                Position bestBox = null;
                int minBoxDist = Integer.MAX_VALUE;
                for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
                    if (box.getValue() == type) {
                        int d = box.getKey().manhattanDistance(goalVal);
                        if (d < minBoxDist) { minBoxDist = d; bestBox = box.getKey(); }
                    }
                }
                
                if (bestBox != null) {
                    // Find nearest agent THAT CAN MANIPULATE THIS BOX TYPE (color match)
                    int bestAgent = -1;
                    int minAgentDist = Integer.MAX_VALUE;
                    for (int i = 0; i < state.getNumAgents(); i++) {
                        if (assignedAgents.contains(i)) continue;
                        if (level.getAgentColor(i) != level.getBoxColor(type)) continue;  // COLOR CONSTRAINT
                        Position aPos = state.getAgentPosition(i);
                        if (aPos == null) continue;
                        int d = aPos.manhattanDistance(bestBox);
                        if (d < minAgentDist) { minAgentDist = d; bestAgent = i; }
                    }
                    
                    if (bestAgent != -1) {
                        assignments.put(bestAgent, new Task(goalVal, type));
                        assignedAgents.add(bestAgent);
                        System.err.println("CBS: Assigned Agent " + bestAgent + " to Box " + type + " at " + bestBox + " for Goal " + goalVal);
                    }
                }
            }
        }
        return assignments;
    }
    
    /**
     * Creates the root CT node by planning for each agent independently.
     */
    private CTNode createRootNode(State initialState, Level level, int numAgents) {
        CTNode root = new CTNode();
        root.constraints = new HashMap<>();
        root.solution = new HashMap<>();
        
        for (int i = 0; i < numAgents; i++) {
            root.constraints.put(i, new HashSet<>());
            
            List<State> path = lowLevelSearch(initialState, level, i, root.constraints.get(i));
            if (path == null) {
                System.err.println(getName() + ": No path found for agent " + i + " in root node");
                return null;
            }
            root.solution.put(i, path);
        }
        
        root.cost = calculateSolutionCost(root.solution);
        return root;
    }
    
    /**
     * Creates a child CT node by adding a constraint and replanning the affected agent.
     */
    private CTNode createChildNode(CTNode parent, Constraint constraint, int agentId, 
                                   State initialState, Level level) {
        CTNode child = new CTNode();
        
        // Copy constraints and add new one
        child.constraints = new HashMap<>();
        for (Map.Entry<Integer, Set<Constraint>> entry : parent.constraints.entrySet()) {
            child.constraints.put(entry.getKey(), new HashSet<>(entry.getValue()));
        }
        child.constraints.get(agentId).add(constraint);
        
        // Copy solution
        child.solution = new HashMap<>(parent.solution);
        
        // Replan for the constrained agent
        List<State> newPath = lowLevelSearch(initialState, level, agentId, child.constraints.get(agentId));
        if (newPath == null) {
            return null; // No valid path with this constraint
        }
        
        child.solution.put(agentId, newPath);
        child.cost = calculateSolutionCost(child.solution);
        
        return child;
    }
    
    /**
     * Low-level A* search for a single agent with time-space constraints.
     * Returns a sequence of states representing the path.
     */
    private List<State> lowLevelSearch(State initialState, Level level, int agentId, 
                                       Set<Constraint> constraints) {
        lowLevelSearches++;
        
        Task task = globalTaskAssignment.get(agentId);
        Position boxGoal = (task != null) ? task.goalPos : null;
        Character boxType = (task != null) ? task.boxType : null;

        // Use space-time A* with constraints
        SpaceTimeAStar stAStar = new SpaceTimeAStar(level, heuristic, agentId, constraints, boxGoal, boxType);
        return stAStar.search(initialState, SearchConfig.MAX_PLAN_LENGTH);
    }

    /**
     * Resolves Agent-Agent and Agent-Box conflicts.
     */
    private Conflict findFirstConflict(CTNode node, Level level) {
        int maxTime = 0;
        for (List<State> path : node.solution.values()) {
            maxTime = Math.max(maxTime, path.size());
        }
        
        // Check each timestep
        for (int t = 0; t < maxTime; t++) {
            List<Integer> agents = new ArrayList<>(node.solution.keySet());
            
            for (int i = 0; i < agents.size(); i++) {
                for (int j = i + 1; j < agents.size(); j++) {
                    int a1 = agents.get(i);
                    int a2 = agents.get(j);
                    
                    // Get occupied cells for both agents (Body + Moved Box)
                    Set<Position> occupied1 = getOccupiedCells(node.solution.get(a1), t, a1);
                    Set<Position> occupied2 = getOccupiedCells(node.solution.get(a2), t, a2);
                    
                    // Check intersection -> VERTEX Conflict
                    for (Position p1 : occupied1) {
                        if (occupied2.contains(p1)) {
                            // Detect who is at p1 to report strict type if needed, but VERTEX is enough
                            return new Conflict(a1, a2, p1, t, ConflictType.VERTEX);
                        }
                    }
                    
                    // Edge conflict: check swap
                    if (t > 0) {
                        Position pos1_prev = getStateAtTime(node.solution.get(a1), t-1).getAgentPosition(a1);
                        Position pos1_curr = getStateAtTime(node.solution.get(a1), t).getAgentPosition(a1);
                        Position pos2_prev = getStateAtTime(node.solution.get(a2), t-1).getAgentPosition(a2);
                        Position pos2_curr = getStateAtTime(node.solution.get(a2), t).getAgentPosition(a2);
                        
                        if (pos1_curr.equals(pos2_prev) && pos2_curr.equals(pos1_prev)) {
                            return new Conflict(a1, a2, pos1_curr, t, ConflictType.EDGE);
                        }
                    }
                }
            }
        }
        
        return null;
    }
    
    private Set<Position> getOccupiedCells(List<State> path, int t, int agentId) {
        Set<Position> occupied = new HashSet<>();
        if (path == null) return occupied;
        
        State current = getStateAtTime(path, t);
        if (current == null) return occupied;
        
        // 1. Agent Body
        Position agentPos = current.getAgentPosition(agentId);
        if (agentPos != null) occupied.add(agentPos);
        
        // 2. Boxes moved by this agent
        // Identifying moved boxes: compare with T=0
        State start = path.get(0);
        for (Map.Entry<Position, Character> entry : current.getBoxes().entrySet()) {
            Position pos = entry.getKey();
            Character type = entry.getValue();
            
            // If checking strict identity is impossible, we check "Is box at Pos in Start?"
            if (!start.getBoxes().containsKey(pos)) {
                // A box is here now, but wasn't at start. -> It moved here.
                occupied.add(pos);
            } else {
                // A box was here at start. Is it the same box?
                // If we assume boxes of same type are fungible, this is tricky.
                // But generally, if a box is at (x,y) and was at (x,y), it's static.
                // If it moved, it's occupied by the mover.
                // NOTE: This assumes decoupled: ONLY Agent `agentId` moves things in this `path`.
            }
        }
        return occupied;
    }

    /**
     * Gets the state at a given timestep, with clamping for paths that end early.
     */
    private State getStateAtTime(List<State> path, int time) {
        if (path == null || path.isEmpty()) {
            return null;
        }
        if (time >= path.size()) {
            return path.get(path.size() - 1); // Agent stays at final position
        }
        return path.get(time);
    }
    
    /**
     * Gets the final state after executing all agent paths.
     * Since CBS plans each agent independently, we need to get the last state
     * from the longest path.
     */
    private State getFinalState(CTNode node, int numAgents) {
        int maxTime = 0;
        for (List<State> path : node.solution.values()) {
            maxTime = Math.max(maxTime, path.size() - 1);
        }
        
        // Get state at final time from any agent's path
        for (List<State> path : node.solution.values()) {
            if (path != null && !path.isEmpty()) {
                return path.get(Math.min(maxTime, path.size() - 1));
            }
        }
        return null;
    }
    
    /**
     * Creates a constraint based on a conflict.
     */
    private Constraint createConstraint(Conflict conflict, int agentId) {
        return new Constraint(agentId, conflict.position, conflict.time);
    }
    
    /**
     * Calculates the total cost of a solution (sum of individual path lengths).
     */
    private int calculateSolutionCost(Map<Integer, List<State>> solution) {
        int cost = 0;
        for (List<State> path : solution.values()) {
            cost += path.size() - 1; // Number of actions = states - 1
        }
        return cost;
    }
    
    /**
     * Converts the CBS solution (individual agent paths) to joint actions.
     */
    private List<Action[]> convertToJointActions(CTNode node, int numAgents) {
        int maxLength = 0;
        for (List<State> path : node.solution.values()) {
            maxLength = Math.max(maxLength, path.size() - 1);
        }
        
        // Check action limit from PRODUCT.md
        if (maxLength > SearchConfig.MAX_ACTIONS) {
            System.err.println(getName() + ": Warning - plan exceeds action limit (" + 
                maxLength + " > " + SearchConfig.MAX_ACTIONS + ")");
        }
        
        List<Action[]> jointPlan = new ArrayList<>();
        
        for (int t = 0; t < maxLength; t++) {
            Action[] jointAction = new Action[numAgents];
            
            for (int a = 0; a < numAgents; a++) {
                List<State> path = node.solution.get(a);
                
                if (path == null || t >= path.size() - 1) {
                    jointAction[a] = Action.noOp();
                } else {
                    State current = path.get(t);
                    State next = path.get(t + 1);
                    jointAction[a] = extractAction(current, next, a);
                }
            }
            
            jointPlan.add(jointAction);
        }
        
        return jointPlan;
    }
    
    /**
     * Extracts the action that transforms current state to next state for an agent.
     */
    private Action extractAction(State current, State next, int agentId) {
        Position curPos = current.getAgentPosition(agentId);
        Position nextPos = next.getAgentPosition(agentId);
        
        if (curPos == null || nextPos == null) {
            return Action.noOp();
        }
        
        // Check if it's just a move
        if (curPos.equals(nextPos)) {
            return Action.noOp();
        }
        
        // Determine direction
        Direction moveDir = getDirection(curPos, nextPos);
        if (moveDir == null) {
            return Action.noOp();
        }
        
        // Check if a box was moved
        Map<Position, Character> curBoxes = current.getBoxes();
        Map<Position, Character> nextBoxes = next.getBoxes();
        
        // Find box that moved
        for (Map.Entry<Position, Character> entry : curBoxes.entrySet()) {
            Position boxPos = entry.getKey();
            Character boxType = entry.getValue();
            
            if (!nextBoxes.containsKey(boxPos) || !nextBoxes.get(boxPos).equals(boxType)) {
                // This box moved
                Position newBoxPos = findNewBoxPosition(boxType, curBoxes, nextBoxes);
                
                if (newBoxPos != null) {
                    // Determine if push or pull
                    if (boxPos.equals(nextPos)) {
                        // Push: agent moved to where box was
                        Direction boxDir = getDirection(boxPos, newBoxPos);
                        if (boxDir != null) {
                            return Action.push(moveDir, boxDir);
                        }
                    } else if (newBoxPos.equals(curPos)) {
                        // Pull: box moved to where agent was
                        Direction boxDir = getDirection(boxPos, newBoxPos);
                        if (boxDir != null) {
                            return Action.pull(moveDir, boxDir.opposite());
                        }
                    }
                }
            }
        }
        
        // Simple move
        return Action.move(moveDir);
    }
    
    private Direction getDirection(Position from, Position to) {
        int dr = to.row - from.row;
        int dc = to.col - from.col;
        
        if (dr == -1 && dc == 0) return Direction.N;
        if (dr == 1 && dc == 0) return Direction.S;
        if (dr == 0 && dc == 1) return Direction.E;
        if (dr == 0 && dc == -1) return Direction.W;
        
        return null;
    }
    
    private Position findNewBoxPosition(Character boxType, Map<Position, Character> curBoxes, 
                                        Map<Position, Character> nextBoxes) {
        for (Map.Entry<Position, Character> entry : nextBoxes.entrySet()) {
            if (entry.getValue().equals(boxType) && !curBoxes.containsKey(entry.getKey())) {
                return entry.getKey();
            }
        }
        return null;
    }
    
    // ==================== Inner Classes ====================
    
    /**
     * Constraint Tree Node - represents a node in the CBS high-level search.
     */
    static class CTNode implements Comparable<CTNode> {
        /** Constraints for each agent */
        Map<Integer, Set<Constraint>> constraints;
        
        /** Solution: path (sequence of states) for each agent */
        Map<Integer, List<State>> solution;
        
        /** Total cost (sum of path lengths) */
        int cost;
        
        @Override
        public int compareTo(CTNode other) {
            return Integer.compare(this.cost, other.cost);
        }
    }
    
    /**
     * A constraint: agent cannot be at position at time.
     */
    static class Constraint {
        final int agentId;
        final Position position;
        final int time;
        
        Constraint(int agentId, Position position, int time) {
            this.agentId = agentId;
            this.position = position;
            this.time = time;
        }
        
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Constraint)) return false;
            Constraint that = (Constraint) o;
            return agentId == that.agentId && time == that.time && 
                   Objects.equals(position, that.position);
        }
        
        @Override
        public int hashCode() {
            return Objects.hash(agentId, position, time);
        }
        
        @Override
        public String toString() {
            return String.format("Constraint{agent=%d, pos=%s, t=%d}", agentId, position, time);
        }
    }
    
    /**
     * Conflict between two agents.
     */
    static class Conflict {
        final int agent1;
        final int agent2;
        final Position position;
        final int time;
        final ConflictType type;
        
        Conflict(int agent1, int agent2, Position position, int time, ConflictType type) {
            this.agent1 = agent1;
            this.agent2 = agent2;
            this.position = position;
            this.time = time;
            this.type = type;
        }
    }
    
    enum ConflictType {
        VERTEX,  // Two agents at same position
        EDGE     // Two agents swap positions
    }
}
