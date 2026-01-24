package mapf.domain;

import java.util.Objects;

/**
 * Immutable class representing a position (coordinate) on the level grid.
 * Uses row and column indices where (0,0) is the top-left corner.
 * Row increases downward, column increases rightward.
 */
public final class Position {
    
    /** The row index (vertical position, 0-indexed from top) */
    public final int row;
    
    /** The column index (horizontal position, 0-indexed from left) */
    public final int col;
    
    /**
     * Creates a new Position with the specified row and column.
     * 
     * @param row the row index (0-indexed)
     * @param col the column index (0-indexed)
     */
    public Position(int row, int col) {
        this.row = row;
        this.col = col;
    }
    
    /**
     * Returns a new Position that is adjacent to this position in the given direction.
     * 
     * @param direction the direction to move (N, S, E, W)
     * @return a new Position in the specified direction
     */
    public Position move(Direction direction) {
        return new Position(row + direction.dRow, col + direction.dCol);
    }
    
    /**
     * Calculates the Manhattan distance from this position to another position.
     * Manhattan distance is |row1 - row2| + |col1 - col2|.
     * 
     * @param other the other position
     * @return the Manhattan distance
     */
    public int manhattanDistance(Position other) {
        return Math.abs(this.row - other.row) + Math.abs(this.col - other.col);
    }
    
    /**
     * Checks if this position is adjacent to another position (including diagonally).
     * 
     * @param other the other position
     * @return true if positions are adjacent
     */
    public boolean isAdjacentTo(Position other) {
        int rowDiff = Math.abs(this.row - other.row);
        int colDiff = Math.abs(this.col - other.col);
        return rowDiff <= 1 && colDiff <= 1 && (rowDiff + colDiff > 0);
    }
    
    /**
     * Checks if this position is cardinally adjacent to another position
     * (not diagonally, only N/S/E/W).
     * 
     * @param other the other position
     * @return true if positions are cardinally adjacent
     */
    public boolean isCardinallyAdjacentTo(Position other) {
        int rowDiff = Math.abs(this.row - other.row);
        int colDiff = Math.abs(this.col - other.col);
        return (rowDiff == 1 && colDiff == 0) || (rowDiff == 0 && colDiff == 1);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Position position = (Position) obj;
        return row == position.row && col == position.col;
    }
    
    @Override
    public int hashCode() {
        // Using a prime number based hash for better distribution
        return 31 * row + col;
    }
    
    @Override
    public String toString() {
        return "(" + row + "," + col + ")";
    }
}
