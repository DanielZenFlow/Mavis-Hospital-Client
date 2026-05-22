package mapf.domain;

import java.util.*;

/**
 * Represents the dynamic state of the level at a point in time.
 * This includes the positions of all agents and boxes.
 * 
 * States are used extensively in the search algorithm, so equals() and hashCode()
 * are carefully implemented for correctness and efficiency.
 */
public class State {
    
    /**
     * Positions of all agents, indexed by agent number (0-9).
     * null entries indicate non-existent agents.
     */
    private final Position[] agentPositions;
    
    /**
     * Maps positions to box types (A-Z).
     * Only positions with boxes are in this map.
     */
    private final Map<Position, Character> boxes;
    
    /** Cached hash code for performance (states are immutable) */
    private int cachedHashCode = 0;
    private boolean hashCodeComputed = false;
    
    /** Cached unmodifiable view of boxes map */
    private Map<Position, Character> unmodifiableBoxes = null;
    
    /**
     * Creates a new State with the given agent positions and boxes.
     * 
     * @param agentPositions array of agent positions (will be copied)
     * @param boxes map of box positions to types (will be copied)
     */
    public State(Position[] agentPositions, Map<Position, Character> boxes) {
        // Defensive copy of agent positions
        this.agentPositions = Arrays.copyOf(agentPositions, agentPositions.length);
        // Defensive copy of boxes map
        this.boxes = new HashMap<>(boxes);
    }
    
    /**
     * Private constructor for internal use (avoids copying when we know the data is safe).
     */
    private State(Position[] agentPositions, Map<Position, Character> boxes, boolean noCopy) {
        this.agentPositions = agentPositions;
        this.boxes = boxes;
    }
    
    /**
     * Gets the position of an agent.
     * 
     * @param agentId the agent number (0-9)
     * @return the agent's position, or null if agent doesn't exist
     */
    public Position getAgentPosition(int agentId) {
        if (agentId < 0 || agentId >= agentPositions.length) {
            return null;
        }
        return agentPositions[agentId];
    }
    
    /**
     * @return the number of agents (length of agent positions array)
     */
    public int getNumAgents() {
        return agentPositions.length;
    }
    
    /**
     * Gets the box type at a position.
     * 
     * @param pos the position to check
     * @return the box type (A-Z) at this position, or '\0' if no box
     */
    public char getBoxAt(Position pos) {
        Character box = boxes.get(pos);
        return box != null ? box : '\0';
    }
    
    /**
     * Checks if there is a box at the specified position.
     * 
     * @param pos the position to check
     * @return true if there is a box at this position
     */
    public boolean hasBoxAt(Position pos) {
        return boxes.containsKey(pos);
    }
    
    /**
     * Checks if there is an agent at the specified position.
     * O(numAgents) linear scan; numAgents <= 10 per problem spec, so the
     * Position flyweight (interned via Position.of) lets equals() short-circuit
     * to a reference compare per probe, but the scan itself is still linear.
     * 
     * @param pos the position to check
     * @return true if there is an agent at this position
     */
    public boolean hasAgentAt(Position pos) {
        for (Position agentPos : agentPositions) {
            if (agentPos != null && agentPos.equals(pos)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Gets the agent at the specified position, if any.
     * O(numAgents) linear scan; same caveat as {@link #hasAgentAt}.
     * 
     * @param pos the position to check
     * @return the agent number at this position, or -1 if no agent
     */
    public int getAgentAt(Position pos) {
        for (int i = 0; i < agentPositions.length; i++) {
            if (agentPositions[i] != null && agentPositions[i].equals(pos)) {
                return i;
            }
        }
        return -1;
    }
    
    /**
     * Checks if a position is free (no wall, no agent, no box).
     * 
     * @param pos the position to check
     * @param level the level (for wall information)
     * @return true if the position is free
     */
    public boolean isFree(Position pos, Level level) {
        return level.isFree(pos) && !hasAgentAt(pos) && !hasBoxAt(pos);
    }
    
    /**
     * @return an unmodifiable view of the boxes map (cached)
     */
    public Map<Position, Character> getBoxes() {
        if (unmodifiableBoxes == null) {
            unmodifiableBoxes = Collections.unmodifiableMap(boxes);
        }
        return unmodifiableBoxes;
    }
    
    /**
     * Checks if this state satisfies all goal conditions.
     * 
     * @param level the level containing goal information
     * @return true if all goals are satisfied
     */
    public boolean isGoalState(Level level) {
        // Check all box goals
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position pos = Position.of(row, col);
                    Character actualBox = boxes.get(pos);
                    if (actualBox == null || actualBox != goalType) {
                        return false;
                    }
                }
                
                int agentGoal = level.getAgentGoal(row, col);
                if (agentGoal != -1) {
                    Position pos = Position.of(row, col);
                    Position agentPos = agentPositions[agentGoal];
                    if (agentPos == null || !agentPos.equals(pos)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    /**
     * Checks if an action is applicable for an agent in this state.
     * 
     * @param action the action to check
     * @param agentId the agent performing the action
     * @param level the level
     * @return true if the action can be performed
     */
    public boolean isApplicable(Action action, int agentId, Level level) {
        Position agentPos = agentPositions[agentId];
        if (agentPos == null) {
            return false;
        }
        
        switch (action.type) {
            case NOOP:
                return true;
                
            case MOVE: {
                Position newPos = agentPos.move(action.agentDir);
                return isFree(newPos, level);
            }
            
            case PUSH: {
                // Agent moves in agentDir, box is in front of agent and moves in boxDir
                Position boxPos = agentPos.move(action.agentDir);
                Position newBoxPos = boxPos.move(action.boxDir);
                
                // Check box exists and agent can move it
                if (!hasBoxAt(boxPos)) return false;
                char boxType = getBoxAt(boxPos);
                if (!level.canAgentMoveBox(agentId, boxType)) return false;
                
                // Check new box position is free
                if (!level.isFree(newBoxPos)) return false;
                if (hasBoxAt(newBoxPos)) return false;
                if (hasAgentAt(newBoxPos)) return false;
                
                return true;
            }
            
            case PULL: {
                // Agent moves in agentDir, box is behind agent and follows
                Position newAgentPos = agentPos.move(action.agentDir);
                Position boxPos = agentPos.move(action.boxDir.opposite());
                
                // Check agent can move to new position
                if (!isFree(newAgentPos, level)) return false;
                
                // Check box exists and agent can move it
                if (!hasBoxAt(boxPos)) return false;
                char boxType = getBoxAt(boxPos);
                if (!level.canAgentMoveBox(agentId, boxType)) return false;
                
                return true;
            }
            
            default:
                return false;
        }
    }

    /**
     * Returns the joint action that the server would effectively apply from this
     * state: individually inapplicable actions and mutually conflicting actions
     * are converted to NoOp. This is useful before replaying or sending a plan,
     * because planner internals may already have treated those entries as failed
     * while leaving the original action object in the returned plan.
     */
    public Action[] sanitizeJointAction(Action[] jointAction, Level level) {
        Action[] effective = Arrays.copyOf(jointAction, jointAction.length);
        boolean[] applicable = new boolean[jointAction.length];
        boolean[] conflicted = new boolean[jointAction.length];
        Position[] agentToPositions = new Position[jointAction.length];
        Position[] boxFromPositions = new Position[jointAction.length];
        Position[] boxToPositions = new Position[jointAction.length];

        for (int agentId = 0; agentId < jointAction.length; agentId++) {
            Action action = jointAction[agentId];
            if (action == null || action.type == Action.ActionType.NOOP) {
                effective[agentId] = Action.noOp();
                applicable[agentId] = true;
                continue;
            }

            Position agentPos = agentPositions[agentId];
            if (agentPos == null || !isApplicable(action, agentId, level)) {
                effective[agentId] = Action.noOp();
                continue;
            }

            applicable[agentId] = true;
            switch (action.type) {
                case MOVE:
                    agentToPositions[agentId] = agentPos.move(action.agentDir);
                    break;
                case PUSH: {
                    Position boxPos = agentPos.move(action.agentDir);
                    agentToPositions[agentId] = boxPos;
                    boxFromPositions[agentId] = boxPos;
                    boxToPositions[agentId] = boxPos.move(action.boxDir);
                    break;
                }
                case PULL:
                    agentToPositions[agentId] = agentPos.move(action.agentDir);
                    boxFromPositions[agentId] = agentPos.move(action.boxDir.opposite());
                    boxToPositions[agentId] = agentPos;
                    break;
                default:
                    break;
            }
        }

        for (int i = 0; i < jointAction.length; i++) {
            if (!applicable[i] || isNoOp(effective[i])) continue;
            for (int j = i + 1; j < jointAction.length; j++) {
                if (!applicable[j] || isNoOp(effective[j])) continue;

                if (sameNonNull(boxFromPositions[i], boxFromPositions[j])
                        || movingObjectsShareDestination(agentToPositions[i], boxToPositions[i],
                                                         agentToPositions[j], boxToPositions[j])) {
                    conflicted[i] = true;
                    conflicted[j] = true;
                }
            }
        }

        for (int agentId = 0; agentId < effective.length; agentId++) {
            if (conflicted[agentId]) {
                effective[agentId] = Action.noOp();
            }
        }

        return effective;
    }
    
    /**
     * Applies an action and returns the resulting state.
     * Assumes the action is applicable (call isApplicable first).
     * 
     * @param action the action to apply
     * @param agentId the agent performing the action
     * @return the new state after applying the action
     */
    public State apply(Action action, int agentId) {
        Position agentPos = agentPositions[agentId];
        
        switch (action.type) {
            case NOOP:
                return this; // No change — return same immutable state
                
            case MOVE: {
                // MOVE only changes agent position, boxes are untouched.
                // Share the box map (safe: State is immutable, no one modifies boxes after construction).
                Position newPos = agentPos.move(action.agentDir);
                Position[] newAgentPositions = Arrays.copyOf(agentPositions, agentPositions.length);
                newAgentPositions[agentId] = newPos;
                return new State(newAgentPositions, boxes, true);
            }
            
            case PUSH: {
                Position[] newAgentPositions = Arrays.copyOf(agentPositions, agentPositions.length);
                Map<Position, Character> newBoxes = new HashMap<>(boxes);
                Position boxPos = agentPos.move(action.agentDir);
                Position newBoxPos = boxPos.move(action.boxDir);
                
                // Move agent
                newAgentPositions[agentId] = boxPos;
                
                // Move box
                char boxType = newBoxes.remove(boxPos);
                newBoxes.put(newBoxPos, boxType);
                return new State(newAgentPositions, newBoxes, true);
            }
            
            case PULL: {
                Position[] newAgentPositions = Arrays.copyOf(agentPositions, agentPositions.length);
                Map<Position, Character> newBoxes = new HashMap<>(boxes);
                Position newAgentPos = agentPos.move(action.agentDir);
                Position boxPos = agentPos.move(action.boxDir.opposite());
                
                // Move agent
                newAgentPositions[agentId] = newAgentPos;
                
                // Move box to agent's old position
                char boxType = newBoxes.remove(boxPos);
                newBoxes.put(agentPos, boxType);
                return new State(newAgentPositions, newBoxes, true);
            }
            
            default:
                return this;
        }
    }
    
    /**
     * Applies a joint action (one action per agent) simultaneously.
     * Per CLAUDE.md: "Cell occupancy is evaluated at the START of each timestep.
     * No agent can move into a cell another is leaving in the same step."
     * 
     * All actions are evaluated against the CURRENT state, then all effects
     * are applied at once to produce the new state.
     * 
     * @param jointAction array of actions, one per agent
     * @param level the level (for applicability checks)
     * @return the new state after all actions are applied simultaneously
     */
    public State applyJointAction(Action[] jointAction, Level level) {
        if (level == null) {
            throw new IllegalArgumentException("Level is required for official joint-action semantics");
        }

        Position[] newAgentPositions = Arrays.copyOf(agentPositions, agentPositions.length);
        Map<Position, Character> newBoxes = new HashMap<>(boxes);

        boolean[] applicable = new boolean[jointAction.length];
        boolean[] conflicted = new boolean[jointAction.length];
        Position[] agentToPositions = new Position[jointAction.length];
        Position[] boxFromPositions = new Position[jointAction.length];
        Position[] boxToPositions = new Position[jointAction.length];
        Character[] boxTypes = new Character[jointAction.length];

        // Phase 1: evaluate every individual action against the START state.
        for (int agentId = 0; agentId < jointAction.length; agentId++) {
            Action action = jointAction[agentId];
            if (action == null || action.type == Action.ActionType.NOOP) {
                applicable[agentId] = true;
                continue;
            }

            Position agentPos = agentPositions[agentId];
            if (agentPos == null) {
                logInapplicableJointActionOnce(agentId, action, null);
                continue;
            }

            if (!isApplicable(action, agentId, level)) {
                if (isJointActionOutOfGrid(action, agentPos, level)) {
                    logOutOfGridJointActionOnce(agentId, action, agentPos, level);
                } else if ((action.type == Action.ActionType.PUSH || action.type == Action.ActionType.PULL)
                        && !hasBoxAt(getJointActionBoxSource(action, agentPos))) {
                    logBoxMismatchOnce(agentId, action, agentPos, getJointActionBoxSource(action, agentPos));
                } else {
                    logInapplicableJointActionOnce(agentId, action, agentPos);
                }
                continue; // Official semantics: inapplicable action is NoOp.
            }

            applicable[agentId] = true;
            switch (action.type) {
                case MOVE:
                    agentToPositions[agentId] = agentPos.move(action.agentDir);
                    break;
                case PUSH:
                    Position boxPos = agentPos.move(action.agentDir);
                    agentToPositions[agentId] = boxPos;
                    boxFromPositions[agentId] = boxPos;
                    boxToPositions[agentId] = boxPos.move(action.boxDir);
                    boxTypes[agentId] = boxes.get(boxPos);
                    break;
                case PULL:
                    agentToPositions[agentId] = agentPos.move(action.agentDir);
                    boxFromPositions[agentId] = agentPos.move(action.boxDir.opposite());
                    boxToPositions[agentId] = agentPos;
                    boxTypes[agentId] = boxes.get(boxFromPositions[agentId]);
                    break;
                default:
                    break;
            }
        }

        // Phase 2: conflicts are also evaluated over intended destinations from
        // the START state. Any involved agent fails and therefore performs NoOp.
        for (int i = 0; i < jointAction.length; i++) {
            if (!applicable[i] || isNoOp(jointAction[i])) continue;
            for (int j = i + 1; j < jointAction.length; j++) {
                if (!applicable[j] || isNoOp(jointAction[j])) continue;

                if (sameNonNull(boxFromPositions[i], boxFromPositions[j])
                        || movingObjectsShareDestination(agentToPositions[i], boxToPositions[i],
                                                         agentToPositions[j], boxToPositions[j])) {
                    conflicted[i] = true;
                    conflicted[j] = true;
                }
            }
        }

        // Phase 3: apply every individually applicable, non-conflicting action atomically.
        for (int agentId = 0; agentId < jointAction.length; agentId++) {
            if (!applicable[agentId] || conflicted[agentId] || isNoOp(jointAction[agentId])) {
                continue;
            }

            Action action = jointAction[agentId];
            switch (action.type) {
                case MOVE:
                    newAgentPositions[agentId] = agentToPositions[agentId];
                    break;
                case PUSH:
                case PULL:
                    newAgentPositions[agentId] = agentToPositions[agentId];
                    newBoxes.remove(boxFromPositions[agentId]);
                    newBoxes.put(boxToPositions[agentId], boxTypes[agentId]);
                    break;
                default:
                    break;
            }
        }

        return new State(newAgentPositions, newBoxes, true);
    }

    private static boolean isNoOp(Action action) {
        return action == null || action.type == Action.ActionType.NOOP;
    }

    private static boolean sameNonNull(Position a, Position b) {
        return a != null && a.equals(b);
    }

    private static boolean movingObjectsShareDestination(Position agentToA, Position boxToA,
                                                        Position agentToB, Position boxToB) {
        return sameNonNull(agentToA, agentToB)
                || sameNonNull(agentToA, boxToB)
                || sameNonNull(boxToA, agentToB)
                || sameNonNull(boxToA, boxToB);
    }

    private static Position getJointActionBoxSource(Action action, Position agentPos) {
        if (action.type == Action.ActionType.PUSH) {
            return agentPos.move(action.agentDir);
        }
        if (action.type == Action.ActionType.PULL) {
            return agentPos.move(action.boxDir.opposite());
        }
        return null;
    }

    /** First-time stderr warning when a joint action contains a non-applicable entry. */
    private static boolean INAPPLICABLE_LOGGED = false;

    private static void logInapplicableJointActionOnce(int agentId, Action action, Position agentPos) {
        if (INAPPLICABLE_LOGGED) return;
        INAPPLICABLE_LOGGED = true;
        System.err.println("[State.applyJointAction] Inapplicable action for agent " + agentId
                + " at " + agentPos + " ; action=" + action + " -- treating as NoOp.");
        StackTraceElement[] st = Thread.currentThread().getStackTrace();
        int n = Math.min(st.length, 12);
        for (int i = 1; i < n; i++) {
            System.err.println("    at " + st[i]);
        }
    }

    /** Returns true iff applying this joint-action entry would put an agent or box off-grid. */
    private static boolean isJointActionOutOfGrid(Action action, Position agentPos, Level level) {
        int rows = level.getRows();
        int cols = level.getCols();
        switch (action.type) {
            case MOVE: {
                int r = agentPos.row + action.agentDir.dRow;
                int c = agentPos.col + action.agentDir.dCol;
                return r < 0 || r >= rows || c < 0 || c >= cols;
            }
            case PUSH: {
                int br = agentPos.row + action.agentDir.dRow;
                int bc = agentPos.col + action.agentDir.dCol;
                if (br < 0 || br >= rows || bc < 0 || bc >= cols) return true;
                int nbr = br + action.boxDir.dRow;
                int nbc = bc + action.boxDir.dCol;
                return nbr < 0 || nbr >= rows || nbc < 0 || nbc >= cols;
            }
            case PULL: {
                int nar = agentPos.row + action.agentDir.dRow;
                int nac = agentPos.col + action.agentDir.dCol;
                if (nar < 0 || nar >= rows || nac < 0 || nac >= cols) return true;
                int br = agentPos.row - action.boxDir.dRow;
                int bc = agentPos.col - action.boxDir.dCol;
                return br < 0 || br >= rows || bc < 0 || bc >= cols;
            }
            default:
                return false;
        }
    }

    private static boolean OUT_OF_GRID_LOGGED = false;

    private static void logOutOfGridJointActionOnce(int agentId, Action action, Position agentPos, Level level) {
        if (OUT_OF_GRID_LOGGED) return;
        OUT_OF_GRID_LOGGED = true;
        System.err.println("[State.applyJointAction] Out-of-grid action for agent " + agentId
                + " at " + agentPos + " (grid " + level.getRows() + "x" + level.getCols() + ")"
                + " ; action=" + action + " -- treating as NoOp.");
        StackTraceElement[] st = Thread.currentThread().getStackTrace();
        int n = Math.min(st.length, 12);
        for (int i = 1; i < n; i++) {
            System.err.println("    at " + st[i]);
        }
    }

    /** One-time stderr warning when PUSH/PULL references a position with no box. */
    private static boolean BOX_MISMATCH_LOGGED = false;

    private static void logBoxMismatchOnce(int agentId, Action action, Position agentPos, Position boxPos) {
        if (BOX_MISMATCH_LOGGED) return;
        BOX_MISMATCH_LOGGED = true;
        System.err.println("[State.applyJointAction] Box-source mismatch: agent " + agentId
                + " at " + agentPos + " ; action=" + action
                + " ; expected box at " + boxPos + " but none found"
                + " -- treating action as NoOp.");
        StackTraceElement[] st = Thread.currentThread().getStackTrace();
        int n = Math.min(st.length, 12);
        for (int i = 1; i < n; i++) {
            System.err.println("    at " + st[i]);
        }
    }

    /**
     * Generates all successor states for a single agent.
     * 
     * @param agentId the agent to generate successors for
     * @param level the level
     * @return list of (action, resulting state) pairs
     */
    public List<Map.Entry<Action, State>> getSuccessors(int agentId, Level level) {
        List<Map.Entry<Action, State>> successors = new ArrayList<>();
        
        // Try all possible actions
        List<Action> allActions = generateAllActions();
        
        for (Action action : allActions) {
            if (isApplicable(action, agentId, level)) {
                State newState = apply(action, agentId);
                successors.add(new AbstractMap.SimpleEntry<>(action, newState));
            }
        }
        
        return successors;
    }
    
    /**
     * Generates all possible actions (for iteration).
     * 
     * @return list of all possible actions
     */
    private List<Action> generateAllActions() {
        List<Action> actions = new ArrayList<>();
        
        // NoOp
        actions.add(Action.noOp());
        
        // Move actions
        for (Direction dir : Direction.values()) {
            actions.add(Action.move(dir));
        }
        
        // Push actions
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                actions.add(Action.push(agentDir, boxDir));
            }
        }
        
        // Pull actions
        for (Direction agentDir : Direction.values()) {
            for (Direction boxDir : Direction.values()) {
                actions.add(Action.pull(agentDir, boxDir));
            }
        }
        
        return actions;
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        
        State other = (State) obj;
        
        // Compare agent positions
        if (!Arrays.equals(agentPositions, other.agentPositions)) {
            return false;
        }
        
        // Compare boxes
        return boxes.equals(other.boxes);
    }
    
    @Override
    public int hashCode() {
        if (!hashCodeComputed) {
            cachedHashCode = computeHashCode();
            hashCodeComputed = true;
        }
        return cachedHashCode;
    }
    
    /**
     * Computes the hash code for this state.
     * Uses a combination of agent positions and box positions.
     */
    private int computeHashCode() {
        int result = Arrays.hashCode(agentPositions);
        result = 31 * result + boxes.hashCode();
        return result;
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("State{agents=[");
        for (int i = 0; i < agentPositions.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(i).append(":").append(agentPositions[i]);
        }
        sb.append("], boxes=").append(boxes).append("}");
        return sb.toString();
    }
    
    /**
     * Creates a visual representation of this state on the level grid.
     * Useful for debugging.
     * 
     * @param level the level
     * @return multi-line string representation
     */
    public String toGridString(Level level) {
        StringBuilder sb = new StringBuilder();
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                Position pos = Position.of(row, col);
                
                if (level.isWall(pos)) {
                    sb.append('+');
                } else if (hasAgentAt(pos)) {
                    int agent = getAgentAt(pos);
                    sb.append((char) ('0' + agent));
                } else if (hasBoxAt(pos)) {
                    sb.append(getBoxAt(pos));
                } else {
                    sb.append(' ');
                }
            }
            sb.append('\n');
        }
        return sb.toString();
    }
}
