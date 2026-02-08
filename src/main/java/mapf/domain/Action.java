package mapf.domain;

import java.util.Objects;

/**
 * Represents an action that an agent can perform.
 * 
 * Action types:
 * - Move: Agent moves in a direction (the cell must be free)
 * - Push: Agent pushes a box. Agent moves in agentDir, box moves in boxDir.
 *         The box must be adjacent to the agent in the agentDir direction.
 * - Pull: Agent pulls a box. Agent moves in agentDir, box follows from opposite direction.
 *         The box must be adjacent to the agent in the opposite of agentDir direction.
 * - NoOp: Agent does nothing (stays in place)
 */
public final class Action {
    
    /**
     * Enum representing the type of action.
     */
    public enum ActionType {
        MOVE,
        PUSH,
        PULL,
        NOOP
    }
    
    /** The type of this action */
    public final ActionType type;
    
    /** Direction the agent moves (null for NoOp) */
    public final Direction agentDir;
    
    /** Direction the box moves (null for Move and NoOp) */
    public final Direction boxDir;
    
    /** Pre-created NoOp action instance for reuse */
    public static final Action NOOP = new Action(ActionType.NOOP, null, null);
    
    // --- Static action cache: all 37 possible actions are pre-created ---
    // 4 Move + 16 Push + 16 Pull + 1 NoOp = 37 total
    private static final Action[] MOVE_ACTIONS = new Action[Direction.values().length];
    private static final Action[][] PUSH_ACTIONS = new Action[Direction.values().length][Direction.values().length];
    private static final Action[][] PULL_ACTIONS = new Action[Direction.values().length][Direction.values().length];
    
    static {
        Direction[] dirs = Direction.values();
        for (int i = 0; i < dirs.length; i++) {
            MOVE_ACTIONS[i] = new Action(ActionType.MOVE, dirs[i], null);
            for (int j = 0; j < dirs.length; j++) {
                PUSH_ACTIONS[i][j] = new Action(ActionType.PUSH, dirs[i], dirs[j]);
                PULL_ACTIONS[i][j] = new Action(ActionType.PULL, dirs[i], dirs[j]);
            }
        }
    }
    
    /**
     * Private constructor - use factory methods to create actions.
     */
    private Action(ActionType type, Direction agentDir, Direction boxDir) {
        this.type = type;
        this.agentDir = agentDir;
        this.boxDir = boxDir;
    }
    
    /**
     * Creates a Move action in the specified direction.
     * 
     * @param direction the direction to move
     * @return a Move action
     */
    public static Action move(Direction direction) {
        Objects.requireNonNull(direction, "Direction cannot be null for Move action");
        return MOVE_ACTIONS[direction.ordinal()];
    }
    
    /**
     * Creates a Push action.
     * Agent moves in agentDir, pushing a box that moves in boxDir.
     * The box must be in the cell the agent is moving into.
     * 
     * @param agentDir direction the agent moves
     * @param boxDir direction the box moves
     * @return a Push action
     */
    public static Action push(Direction agentDir, Direction boxDir) {
        Objects.requireNonNull(agentDir, "Agent direction cannot be null for Push action");
        Objects.requireNonNull(boxDir, "Box direction cannot be null for Push action");
        return PUSH_ACTIONS[agentDir.ordinal()][boxDir.ordinal()];
    }
    
    /**
     * Creates a Pull action.
     * Agent moves in agentDir, pulling a box from the opposite direction.
     * The box is in the cell opposite to agentDir from the agent.
     * 
     * @param agentDir direction the agent moves
     * @param boxDir direction the box moves (into the agent's former position)
     * @return a Pull action
     */
    public static Action pull(Direction agentDir, Direction boxDir) {
        Objects.requireNonNull(agentDir, "Agent direction cannot be null for Pull action");
        Objects.requireNonNull(boxDir, "Box direction cannot be null for Pull action");
        return PULL_ACTIONS[agentDir.ordinal()][boxDir.ordinal()];
    }
    
    /**
     * Returns the NoOp action (agent does nothing).
     * 
     * @return the NoOp action singleton
     */
    public static Action noOp() {
        return NOOP;
    }
    
    /**
     * Converts this action to the server protocol format.
     * 
     * Examples:
     * - Move(N)
     * - Push(E,S)
     * - Pull(W,N)
     * - NoOp
     * 
     * @return string representation in server protocol format
     */
    public String toServerString() {
        return switch (type) {
            case MOVE -> "Move(" + agentDir.name() + ")";
            case PUSH -> "Push(" + agentDir.name() + "," + boxDir.name() + ")";
            case PULL -> "Pull(" + agentDir.name() + "," + boxDir.name() + ")";
            case NOOP -> "NoOp";
        };
    }
    
    /**
     * Parses an action from the server protocol format.
     * 
     * @param s the string in server protocol format
     * @return the parsed Action
     * @throws IllegalArgumentException if the format is invalid
     */
    public static Action fromServerString(String s) {
        s = s.trim();
        
        if (s.equals("NoOp")) {
            return NOOP;
        }
        
        if (s.startsWith("Move(") && s.endsWith(")")) {
            String dir = s.substring(5, s.length() - 1);
            return move(Direction.fromString(dir));
        }
        
        if (s.startsWith("Push(") && s.endsWith(")")) {
            String[] parts = s.substring(5, s.length() - 1).split(",");
            if (parts.length != 2) {
                throw new IllegalArgumentException("Invalid Push action format: " + s);
            }
            return push(Direction.fromString(parts[0].trim()), 
                       Direction.fromString(parts[1].trim()));
        }
        
        if (s.startsWith("Pull(") && s.endsWith(")")) {
            String[] parts = s.substring(5, s.length() - 1).split(",");
            if (parts.length != 2) {
                throw new IllegalArgumentException("Invalid Pull action format: " + s);
            }
            return pull(Direction.fromString(parts[0].trim()), 
                       Direction.fromString(parts[1].trim()));
        }
        
        throw new IllegalArgumentException("Unknown action format: " + s);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Action action = (Action) obj;
        return type == action.type && agentDir == action.agentDir && boxDir == action.boxDir;
    }
    
    @Override
    public int hashCode() {
        return Objects.hash(type, agentDir, boxDir);
    }
    
    @Override
    public String toString() {
        return toServerString();
    }
}
