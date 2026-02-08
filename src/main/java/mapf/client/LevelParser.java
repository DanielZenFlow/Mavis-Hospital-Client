package mapf.client;

import mapf.domain.*;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.*;

/**
 * Parses level files from the server's text format.
 * 
 * Level format:
 * <pre>
 * #domain
 * hospital
 * #levelname
 * ExampleLevel
 * #colors
 * red: 0, A, B
 * blue: 1, C
 * #initial
 * +++++
 * +0A +
 * + B1+
 * +++++
 * #goal
 * +++++
 * +  A+
 * + B +
 * +++++
 * #end
 * </pre>
 * 
 * Grid symbols:
 * - '+' : Wall
 * - '0'-'9' : Agent
 * - 'A'-'Z' : Box
 * - ' ' : Empty space
 */
public class LevelParser {
    
    /**
     * Result of parsing a level, containing both the static Level
     * and the initial dynamic State.
     */
    public static class ParseResult {
        public final Level level;
        public final State initialState;
        
        public ParseResult(Level level, State initialState) {
            this.level = level;
            this.initialState = initialState;
        }
    }
    
    /**
     * Parses a level from a BufferedReader.
     * Reads until #end marker or end of stream.
     * 
     * @param reader the reader to read from
     * @return ParseResult containing Level and initial State
     * @throws IOException if reading fails
     * @throws IllegalArgumentException if the level format is invalid
     */
    public ParseResult parse(BufferedReader reader) throws IOException {
        String line;
        String levelName = "Unknown";
        Map<Character, Color> boxColors = new HashMap<>();
        Map<Integer, Color> agentColors = new HashMap<>();
        List<String> initialGrid = new ArrayList<>();
        List<String> goalGrid = new ArrayList<>();
        
        String currentSection = null;
        
        while ((line = reader.readLine()) != null) {
            // Check for section headers
            if (line.startsWith("#")) {
                currentSection = line.substring(1).trim().toLowerCase();
                if (currentSection.equals("end")) {
                    break;
                }
                continue;
            }
            
            // Process based on current section
            if (currentSection == null) {
                continue;
            }
            
            switch (currentSection) {
                case "domain":
                    // We expect "hospital" but don't enforce it
                    break;
                    
                case "levelname":
                    levelName = line.trim();
                    break;
                    
                case "colors":
                    parseColorLine(line, boxColors, agentColors);
                    break;
                    
                case "initial":
                    initialGrid.add(line);
                    break;
                    
                case "goal":
                    goalGrid.add(line);
                    break;
            }
        }
        
        // Validate we have the necessary data
        if (initialGrid.isEmpty()) {
            throw new IllegalArgumentException("No initial grid found in level");
        }
        if (goalGrid.isEmpty()) {
            throw new IllegalArgumentException("No goal grid found in level");
        }
        
        // Determine grid dimensions
        int rows = initialGrid.size();
        int cols = 0;
        for (String gridLine : initialGrid) {
            cols = Math.max(cols, gridLine.length());
        }
        
        // Parse walls from initial grid (walls should be same in both)
        boolean[][] walls = new boolean[rows][cols];
        Position[] agentPositions = new Position[mapf.planning.SearchConfig.MAX_AGENTS];
        Map<Position, Character> boxes = new HashMap<>();
        
        for (int r = 0; r < rows; r++) {
            String gridLine = initialGrid.get(r);
            for (int c = 0; c < cols; c++) {
                char ch = c < gridLine.length() ? gridLine.charAt(c) : ' ';
                Position pos = Position.of(r, c);
                
                if (ch == '+') {
                    walls[r][c] = true;
                } else if (ch >= '0' && ch <= '9') {
                    int agentId = ch - '0';
                    agentPositions[agentId] = pos;
                } else if (ch >= 'A' && ch <= 'Z') {
                    boxes.put(pos, ch);
                }
            }
        }
        
        // Find max agent ID and validate contiguity (PRODUCT.md: "consecutively numbered starting from 0")
        int maxAgentId = -1;
        for (int i = 0; i < agentPositions.length; i++) {
            if (agentPositions[i] != null) {
                maxAgentId = i;
            }
        }
        
        // Validate agent IDs are contiguous 0..maxAgentId
        for (int i = 0; i <= maxAgentId; i++) {
            if (agentPositions[i] == null) {
                throw new IllegalArgumentException("Agent IDs must be contiguous: missing Agent " + i);
            }
        }
        
        // Validate all agents have colors defined (PRODUCT.md: critical constraint)
        for (int i = 0; i <= maxAgentId; i++) {
            if (!agentColors.containsKey(i)) {
                throw new IllegalArgumentException("Agent " + i + " has no color defined in #colors section");
            }
        }
        
        // Validate all box types have colors defined
        Set<Character> boxTypes = new HashSet<>();
        for (char boxType : boxes.values()) {
            boxTypes.add(boxType);
        }
        for (char boxType : boxTypes) {
            if (!boxColors.containsKey(boxType)) {
                throw new IllegalArgumentException("Box type '" + boxType + "' has no color defined in #colors section");
            }
        }
        
        // Validate grid dimensions match (PRODUCT.md: "Walls must match exactly")
        if (goalGrid.size() != rows) {
            throw new IllegalArgumentException("Goal grid rows (" + goalGrid.size() + 
                ") must match initial grid rows (" + rows + ")");
        }
        
        // Parse goal grid
        char[][] boxGoals = new char[rows][cols];
        int[][] agentGoals = new int[rows][cols];
        
        // Initialize agentGoals to -1
        for (int r = 0; r < rows; r++) {
            Arrays.fill(agentGoals[r], -1);
        }
        
        for (int r = 0; r < goalGrid.size(); r++) {
            String gridLine = goalGrid.get(r);
            for (int c = 0; c < cols; c++) {
                char ch = c < gridLine.length() ? gridLine.charAt(c) : ' ';
                
                // Validate walls match between initial and goal (PRODUCT.md requirement)
                char initialCh = c < initialGrid.get(r).length() ? initialGrid.get(r).charAt(c) : ' ';
                boolean isWallInInitial = (initialCh == '+');
                boolean isWallInGoal = (ch == '+');
                if (isWallInInitial != isWallInGoal) {
                    throw new IllegalArgumentException("Wall mismatch at (" + r + "," + c + 
                        "): initial=" + isWallInInitial + ", goal=" + isWallInGoal);
                }
                
                if (ch >= 'A' && ch <= 'Z') {
                    boxGoals[r][c] = ch;
                } else if (ch >= '0' && ch <= '9') {
                    agentGoals[r][c] = ch - '0';
                }
            }
        }
        
        // Create Level and State
        Level level = new Level(levelName, rows, cols, walls, boxGoals, agentGoals, 
                               boxColors, agentColors);
        
        // Create properly sized array (maxAgentId + 1 elements)
        // This preserves agent ID as array index
        Position[] correctAgentPositions;
        if (maxAgentId >= 0) {
            correctAgentPositions = new Position[maxAgentId + 1];
            System.arraycopy(agentPositions, 0, correctAgentPositions, 0, maxAgentId + 1);
        } else {
            correctAgentPositions = new Position[0];
        }
        
        State initialState = new State(correctAgentPositions, boxes);
        
        return new ParseResult(level, initialState);
    }
    
    /**
     * Parses a color line in the format "color: object, object, ..."
     * Objects can be agent numbers (0-9) or box types (A-Z).
     * 
     * @param line the line to parse
     * @param boxColors map to add box colors to
     * @param agentColors map to add agent colors to
     */
    private void parseColorLine(String line, Map<Character, Color> boxColors, 
                                Map<Integer, Color> agentColors) {
        line = line.trim();
        if (line.isEmpty()) return;
        
        int colonIndex = line.indexOf(':');
        if (colonIndex < 0) return;
        
        String colorName = line.substring(0, colonIndex).trim();
        String objectsStr = line.substring(colonIndex + 1).trim();
        
        Color color;
        try {
            color = Color.fromString(colorName);
        } catch (IllegalArgumentException e) {
            System.err.println("Warning: Unknown color '" + colorName + "', skipping");
            return;
        }
        
        String[] objects = objectsStr.split(",");
        for (String obj : objects) {
            obj = obj.trim();
            if (obj.isEmpty()) continue;
            
            char ch = obj.charAt(0);
            if (ch >= '0' && ch <= '9') {
                agentColors.put(ch - '0', color);
            } else if (ch >= 'A' && ch <= 'Z') {
                boxColors.put(ch, color);
            }
        }
    }
    
    /**
     * Parses a level from a list of strings (for testing).
     * 
     * @param lines the level content as a list of strings
     * @return ParseResult containing Level and initial State
     */
    public ParseResult parseFromStrings(List<String> lines) throws IOException {
        StringBuilder sb = new StringBuilder();
        for (String line : lines) {
            sb.append(line).append("\n");
        }
        
        BufferedReader reader = new BufferedReader(new java.io.StringReader(sb.toString()));
        return parse(reader);
    }
}
