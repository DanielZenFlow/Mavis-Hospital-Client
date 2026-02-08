package mapf.domain;

/**
 * Immutable class representing a position (coordinate) on the level grid.
 * Uses row and column indices where (0,0) is the top-left corner.
 * Row increases downward, column increases rightward.
 * 
 * Flyweight pattern: Positions within the initialized grid are cached and reused.
 * This eliminates millions of short-lived Position allocations during A* search.
 */
public final class Position {
    
    /** The row index (vertical position, 0-indexed from top) */
    public final int row;
    
    /** The column index (horizontal position, 0-indexed from left) */
    public final int col;
    
    // --- Flyweight cache ---
    private static Position[][] cache;
    private static int cachedRows = 0;
    private static int cachedCols = 0;
    
    /**
     * Initializes the position cache for a grid of the given size.
     * Must be called once at level load time, before any search begins.
     * 
     * @param rows number of rows in the grid
     * @param cols number of columns in the grid
     */
    public static void initCache(int rows, int cols) {
        if (rows == cachedRows && cols == cachedCols) return; // Already initialized
        cachedRows = rows;
        cachedCols = cols;
        cache = new Position[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                cache[r][c] = new Position(r, c);
            }
        }
    }
    
    /**
     * Returns a cached Position if within grid bounds, otherwise creates a new one.
     * Prefer this over `new Position(row, col)` in hot paths.
     */
    public static Position of(int row, int col) {
        if (cache != null && row >= 0 && row < cachedRows && col >= 0 && col < cachedCols) {
            return cache[row][col];
        }
        return new Position(row, col);
    }
    
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
     * Returns the Position adjacent to this position in the given direction.
     * Uses the flyweight cache when available (O(1) lookup, zero allocation).
     * 
     * @param direction the direction to move (N, S, E, W)
     * @return the Position in the specified direction
     */
    public Position move(Direction direction) {
        return Position.of(row + direction.dRow, col + direction.dCol);
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
