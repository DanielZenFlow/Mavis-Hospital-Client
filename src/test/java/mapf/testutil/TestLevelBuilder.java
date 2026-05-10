package mapf.testutil;

import mapf.domain.Color;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.util.HashMap;
import java.util.Map;

public final class TestLevelBuilder {
    private final String name;
    private final int rows;
    private final int cols;
    private final boolean[][] walls;
    private final char[][] boxGoals;
    private final int[][] agentGoals;
    private final Map<Character, Color> boxColors = new HashMap<>();
    private final Map<Integer, Color> agentColors = new HashMap<>();
    private final Position[] agents = new Position[10];
    private final Map<Position, Character> boxes = new HashMap<>();

    private TestLevelBuilder(String name, int rows, int cols) {
        this.name = name;
        this.rows = rows;
        this.cols = cols;
        this.walls = new boolean[rows][cols];
        this.boxGoals = new char[rows][cols];
        this.agentGoals = new int[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                agentGoals[r][c] = -1;
            }
        }
    }

    public static TestLevelBuilder level(String name, int rows, int cols) {
        return new TestLevelBuilder(name, rows, cols);
    }

    public TestLevelBuilder borderWalls() {
        for (int r = 0; r < rows; r++) {
            wall(r, 0);
            wall(r, cols - 1);
        }
        for (int c = 0; c < cols; c++) {
            wall(0, c);
            wall(rows - 1, c);
        }
        return this;
    }

    public TestLevelBuilder fillInteriorWalls() {
        for (int r = 1; r < rows - 1; r++) {
            for (int c = 1; c < cols - 1; c++) {
                wall(r, c);
            }
        }
        return this;
    }

    public TestLevelBuilder open(int row, int col) {
        walls[row][col] = false;
        return this;
    }

    public TestLevelBuilder wall(int row, int col) {
        walls[row][col] = true;
        return this;
    }

    public TestLevelBuilder agent(int id, Color color, int row, int col) {
        agents[id] = new Position(row, col);
        agentColors.put(id, color);
        return this;
    }

    public TestLevelBuilder box(char type, Color color, int row, int col) {
        boxes.put(new Position(row, col), type);
        boxColors.put(type, color);
        return this;
    }

    public TestLevelBuilder boxGoal(char type, int row, int col) {
        boxGoals[row][col] = type;
        return this;
    }

    public TestLevelBuilder agentGoal(int id, int row, int col) {
        agentGoals[row][col] = id;
        return this;
    }

    public BuiltLevel build() {
        int maxAgent = agentColors.keySet().stream().mapToInt(Integer::intValue).max().orElse(-1);
        Position[] compactAgents = new Position[maxAgent + 1];
        for (int i = 0; i <= maxAgent; i++) {
            compactAgents[i] = agents[i];
        }
        Level level = new Level(name, rows, cols, walls, boxGoals, agentGoals, boxColors, agentColors);
        State state = new State(compactAgents, boxes);
        return new BuiltLevel(level, state);
    }

    public record BuiltLevel(Level level, State state) {}
}
