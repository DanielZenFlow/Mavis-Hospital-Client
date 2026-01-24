package mapf.domain;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Represents the static (unchanging) information about a level.
 * This includes walls, goal positions, colors, and level metadata.
 * 
 * The level uses a coordinate system where:
 * - Row 0 is the top
 * - Column 0 is the left
 * - Positions are accessed as (row, col)
 */
public class Level {
    
    /** Level name from the level file */
    private final String name;
    
    /** Number of rows in the level */
    private final int rows;
    
    /** Number of columns in the level */
    private final int cols;
    
    /** 
     * Wall positions. walls[row][col] is true if there's a wall at (row, col).
     * Walls are impassable for both agents and boxes.
     */
    private final boolean[][] walls;
    
    /**
     * Box goal positions. boxGoals[row][col] contains the box type (A-Z) that
     * should be at this position in the goal state, or '\0' (null char) if no goal.
     */
    private final char[][] boxGoals;
    
    /**
     * Agent goal positions. agentGoals[row][col] contains the agent number (0-9)
     * that should be at this position in the goal state, or -1 if no goal.
     */
    private final int[][] agentGoals;
    
    /**
     * Maps box types (A-Z) to their colors.
     * An agent can only interact with boxes of the same color.
     */
    private final Map<Character, Color> boxColors;
    
    /**
     * Maps agent numbers (0-9) to their colors.
     * An agent can only interact with boxes of the same color.
     */
    private final Map<Integer, Color> agentColors;
    
    /** Number of agents in this level */
    private final int numAgents;
    
    /**
     * Creates a new Level with the specified parameters.
     * 
     * @param name level name
     * @param rows number of rows
     * @param cols number of columns
     * @param walls wall positions (will be copied)
     * @param boxGoals box goal positions (will be copied)
     * @param agentGoals agent goal positions (will be copied)
     * @param boxColors box type to color mapping (will be copied)
     * @param agentColors agent number to color mapping (will be copied)
     */
    public Level(String name, int rows, int cols, 
                 boolean[][] walls, char[][] boxGoals, int[][] agentGoals,
                 Map<Character, Color> boxColors, Map<Integer, Color> agentColors) {
        this.name = name;
        this.rows = rows;
        this.cols = cols;
        
        // Deep copy walls
        this.walls = new boolean[rows][cols];
        for (int r = 0; r < rows; r++) {
            System.arraycopy(walls[r], 0, this.walls[r], 0, cols);
        }
        
        // Deep copy boxGoals
        this.boxGoals = new char[rows][cols];
        for (int r = 0; r < rows; r++) {
            System.arraycopy(boxGoals[r], 0, this.boxGoals[r], 0, cols);
        }
        
        // Deep copy agentGoals
        this.agentGoals = new int[rows][cols];
        for (int r = 0; r < rows; r++) {
            System.arraycopy(agentGoals[r], 0, this.agentGoals[r], 0, cols);
        }
        
        // Copy color maps
        this.boxColors = new HashMap<>(boxColors);
        this.agentColors = new HashMap<>(agentColors);
        this.numAgents = agentColors.size();
    }
    
    /**
     * @return the level name
     */
    public String getName() {
        return name;
    }
    
    /**
     * @return number of rows in the level
     */
    public int getRows() {
        return rows;
    }
    
    /**
     * @return number of columns in the level
     */
    public int getCols() {
        return cols;
    }
    
    /**
     * @return number of agents in this level
     */
    public int getNumAgents() {
        return numAgents;
    }
    
    /**
     * Checks if there is a wall at the specified position.
     * 
     * @param row the row index
     * @param col the column index
     * @return true if there is a wall at (row, col)
     */
    public boolean isWall(int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            return true; // Out of bounds is treated as wall
        }
        return walls[row][col];
    }
    
    /**
     * Checks if there is a wall at the specified position.
     * 
     * @param pos the position to check
     * @return true if there is a wall at the position
     */
    public boolean isWall(Position pos) {
        return isWall(pos.row, pos.col);
    }
    
    /**
     * Checks if a position is within bounds and not a wall.
     * 
     * @param row the row index
     * @param col the column index
     * @return true if the position is passable
     */
    public boolean isFree(int row, int col) {
        return row >= 0 && row < rows && col >= 0 && col < cols && !walls[row][col];
    }
    
    /**
     * Checks if a position is within bounds and not a wall.
     * 
     * @param pos the position to check
     * @return true if the position is passable
     */
    public boolean isFree(Position pos) {
        return isFree(pos.row, pos.col);
    }
    
    /**
     * Gets the box goal type at the specified position.
     * 
     * @param row the row index
     * @param col the column index
     * @return the box type (A-Z) required at this position, or '\0' if no goal
     */
    public char getBoxGoal(int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            return '\0';
        }
        return boxGoals[row][col];
    }
    
    /**
     * Gets the box goal type at the specified position.
     * 
     * @param pos the position to check
     * @return the box type (A-Z) required at this position, or '\0' if no goal
     */
    public char getBoxGoal(Position pos) {
        return getBoxGoal(pos.row, pos.col);
    }
    
    /**
     * Gets the agent goal at the specified position.
     * 
     * @param row the row index
     * @param col the column index
     * @return the agent number (0-9) required at this position, or -1 if no goal
     */
    public int getAgentGoal(int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            return -1;
        }
        return agentGoals[row][col];
    }
    
    /**
     * Gets the agent goal at the specified position.
     * 
     * @param pos the position to check
     * @return the agent number (0-9) required at this position, or -1 if no goal
     */
    public int getAgentGoal(Position pos) {
        return getAgentGoal(pos.row, pos.col);
    }
    
    /**
     * Checks if there is a box goal at the specified position.
     * 
     * @param pos the position to check
     * @return true if there is a box goal at this position
     */
    public boolean hasBoxGoal(Position pos) {
        return getBoxGoal(pos) != '\0';
    }
    
    /**
     * Checks if there is an agent goal at the specified position.
     * 
     * @param pos the position to check
     * @return true if there is an agent goal at this position
     */
    public boolean hasAgentGoal(Position pos) {
        return getAgentGoal(pos) != -1;
    }
    
    /**
     * Gets the color of a box type.
     * 
     * @param boxType the box type (A-Z)
     * @return the color of this box type
     */
    public Color getBoxColor(char boxType) {
        return boxColors.get(boxType);
    }
    
    /**
     * Gets the color of an agent.
     * 
     * @param agentId the agent number (0-9)
     * @return the color of this agent
     */
    public Color getAgentColor(int agentId) {
        return agentColors.get(agentId);
    }
    
    /**
     * Checks if an agent can interact with a box (same color).
     * 
     * @param agentId the agent number
     * @param boxType the box type
     * @return true if the agent can push/pull the box
     */
    public boolean canAgentMoveBox(int agentId, char boxType) {
        Color agentColor = agentColors.get(agentId);
        Color boxColor = boxColors.get(boxType);
        return agentColor != null && agentColor.equals(boxColor);
    }
    
    /**
     * @return an unmodifiable view of the box colors map
     */
    public Map<Character, Color> getBoxColors() {
        return Collections.unmodifiableMap(boxColors);
    }
    
    /**
     * @return an unmodifiable view of the agent colors map
     */
    public Map<Integer, Color> getAgentColors() {
        return Collections.unmodifiableMap(agentColors);
    }
    
    @Override
    public String toString() {
        return "Level{name='" + name + "', rows=" + rows + ", cols=" + cols + 
               ", agents=" + numAgents + "}";
    }
}
