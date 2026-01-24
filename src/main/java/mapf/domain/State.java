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
     * @return an unmodifiable view of the boxes map
     */
    public Map<Position, Character> getBoxes() {
        return Collections.unmodifiableMap(boxes);
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
                    Position pos = new Position(row, col);
                    Character actualBox = boxes.get(pos);
                    if (actualBox == null || actualBox != goalType) {
                        return false;
                    }
                }
                
                int agentGoal = level.getAgentGoal(row, col);
                if (agentGoal != -1) {
                    Position pos = new Position(row, col);
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
     * Applies an action and returns the resulting state.
     * Assumes the action is applicable (call isApplicable first).
     * 
     * @param action the action to apply
     * @param agentId the agent performing the action
     * @return the new state after applying the action
     */
    public State apply(Action action, int agentId) {
        Position[] newAgentPositions = Arrays.copyOf(agentPositions, agentPositions.length);
        Map<Position, Character> newBoxes = new HashMap<>(boxes);
        
        Position agentPos = agentPositions[agentId];
        
        switch (action.type) {
            case NOOP:
                // No change
                break;
                
            case MOVE: {
                Position newPos = agentPos.move(action.agentDir);
                newAgentPositions[agentId] = newPos;
                break;
            }
            
            case PUSH: {
                Position boxPos = agentPos.move(action.agentDir);
                Position newBoxPos = boxPos.move(action.boxDir);
                
                // Move agent
                newAgentPositions[agentId] = boxPos;
                
                // Move box
                char boxType = newBoxes.remove(boxPos);
                newBoxes.put(newBoxPos, boxType);
                break;
            }
            
            case PULL: {
                Position newAgentPos = agentPos.move(action.agentDir);
                Position boxPos = agentPos.move(action.boxDir.opposite());
                
                // Move agent
                newAgentPositions[agentId] = newAgentPos;
                
                // Move box to agent's old position
                char boxType = newBoxes.remove(boxPos);
                newBoxes.put(agentPos, boxType);
                break;
            }
        }
        
        return new State(newAgentPositions, newBoxes, true);
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
                Position pos = new Position(row, col);
                
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
