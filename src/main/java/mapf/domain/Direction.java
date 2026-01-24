package mapf.domain;

/**
 * Enum representing the four cardinal directions for movement.
 * Each direction has associated row and column deltas.
 */
public enum Direction {
    /** North - move up (decrease row) */
    N(-1, 0),
    
    /** South - move down (increase row) */
    S(1, 0),
    
    /** East - move right (increase column) */
    E(0, 1),
    
    /** West - move left (decrease column) */
    W(0, -1);
    
    /** Row delta when moving in this direction */
    public final int dRow;
    
    /** Column delta when moving in this direction */
    public final int dCol;
    
    Direction(int dRow, int dCol) {
        this.dRow = dRow;
        this.dCol = dCol;
    }
    
    /**
     * Returns the opposite direction.
     * N <-> S, E <-> W
     * 
     * @return the opposite direction
     */
    public Direction opposite() {
        return switch (this) {
            case N -> S;
            case S -> N;
            case E -> W;
            case W -> E;
        };
    }
    
    /**
     * Parses a direction from its string representation.
     * 
     * @param s the string representation (N, S, E, or W)
     * @return the corresponding Direction
     * @throws IllegalArgumentException if the string is not a valid direction
     */
    public static Direction fromString(String s) {
        return switch (s.toUpperCase()) {
            case "N" -> N;
            case "S" -> S;
            case "E" -> E;
            case "W" -> W;
            default -> throw new IllegalArgumentException("Invalid direction: " + s);
        };
    }
}
