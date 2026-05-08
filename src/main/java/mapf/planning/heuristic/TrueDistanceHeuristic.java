package mapf.planning.heuristic;

import mapf.domain.*;

import java.util.*;

/**
 * True distance heuristic using BFS-precomputed distances.
 * 
 * On construction, performs BFS from each goal position to compute
 * the true shortest path distance (considering walls) to every reachable cell.
 * 
 * The estimate() method then looks up these precomputed distances in O(1) time.
 * 
 * This heuristic is more accurate than Manhattan distance because it accounts
 * for walls and obstacles, making it particularly valuable in maze-like levels.
 */
public class TrueDistanceHeuristic implements Heuristic {
    
    /**
     * Distance maps from each goal position.
     * Key: goal position, Value: 2D array of distances from that goal
     */
    private final Map<Position, int[][]> distanceMaps;
    
    /** Maps box types to their goal positions */
    private final Map<Character, List<Position>> boxGoalPositions;
    
    /** Maps agent IDs to their goal positions */
    private final Map<Integer, Position> agentGoalPositions;
    
    /** The level this heuristic was built for */
    private final Level level;
    
    /** Value representing unreachable cells */
    private static final int UNREACHABLE = Integer.MAX_VALUE;
    
    /**
     * Creates a new TrueDistanceHeuristic by precomputing distances from all goals.
     * 
     * @param level the level to compute distances for
     */
    public TrueDistanceHeuristic(Level level) {
        this.level = level;
        this.distanceMaps = new HashMap<>();
        this.boxGoalPositions = new HashMap<>();
        this.agentGoalPositions = new HashMap<>();
        
        // Find all goal positions
        findGoalPositions();
        
        // Precompute distances from each goal
        Set<Position> allGoals = new HashSet<>();
        for (List<Position> goals : boxGoalPositions.values()) {
            allGoals.addAll(goals);
        }
        allGoals.addAll(agentGoalPositions.values());
        
        for (Position goal : allGoals) {
            distanceMaps.put(goal, computeDistances(goal));
        }
    }
    
    /**
     * Finds all goal positions in the level.
     */
    private void findGoalPositions() {
        boxGoalPositions.putAll(level.getBoxGoalsByType());
        agentGoalPositions.putAll(level.getAgentGoalPositionMap());
    }
    
    /**
     * Computes shortest path distances from a goal position to all cells using BFS.
     * 
     * @param goal the goal position to compute distances from
     * @return 2D array of distances (UNREACHABLE for walls and unreachable cells)
     */
    private int[][] computeDistances(Position goal) {
        int rows = level.getRows();
        int cols = level.getCols();
        int[][] distances = new int[rows][cols];
        
        // Initialize all distances to unreachable
        for (int[] row : distances) {
            Arrays.fill(row, UNREACHABLE);
        }
        
        // BFS from goal
        Queue<Position> queue = new LinkedList<>();
        queue.add(goal);
        distances[goal.row][goal.col] = 0;
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int currentDist = distances[current.row][current.col];
            
            // Explore neighbors
            for (Direction dir : Direction.values()) {
                Position neighbor = current.move(dir);
                
                // Skip if wall or out of bounds
                if (!level.isFree(neighbor)) {
                    continue;
                }
                
                // Skip if already visited with shorter distance
                if (distances[neighbor.row][neighbor.col] <= currentDist + 1) {
                    continue;
                }
                
                distances[neighbor.row][neighbor.col] = currentDist + 1;
                queue.add(neighbor);
            }
        }
        
        return distances;
    }
    
    @Override
    public int estimate(State state, Level level) {
        int totalDistance = 0;
        
        // Calculate distance for each box to its nearest goal
        // Also track which boxes are NOT yet at their goals (per agent color)
        Map<Color, List<Position>> unsatisfiedBoxesByColor = new HashMap<>();
        
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position boxPos = entry.getKey();
            char boxType = entry.getValue();
            
            int minDist = getMinDistanceToGoal(boxPos, boxType);
            if (minDist < UNREACHABLE) {
                totalDistance += minDist;
                
                // Track unsatisfied boxes (distance > 0 means not at goal)
                if (minDist > 0) {
                    Color boxColor = level.getBoxColor(boxType);
                    unsatisfiedBoxesByColor.computeIfAbsent(boxColor, k -> new ArrayList<>()).add(boxPos);
                }
            }
        }
        
        // Calculate distance for each agent to its goal position (if any)
        for (Map.Entry<Integer, Position> entry : agentGoalPositions.entrySet()) {
            int agentId = entry.getKey();
            Position goalPos = entry.getValue();
            Position agentPos = state.getAgentPosition(agentId);
            
            if (agentPos != null) {
                int[][] distMap = distanceMaps.get(goalPos);
                if (distMap != null && inBounds(agentPos)) {
                    int dist = distMap[agentPos.row][agentPos.col];
                    if (dist < UNREACHABLE) {
                        totalDistance += dist;
                    }
                }
            }
        }
        
        // NEW: Add distance from each agent to its nearest unsatisfied box
        // This encourages agents to stay near their work and discourages unnecessary movement
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            Position agentPos = state.getAgentPosition(agentId);
            if (agentPos == null) continue;
            
            Color agentColor = level.getAgentColor(agentId);
            List<Position> unsatisfiedBoxes = unsatisfiedBoxesByColor.get(agentColor);
            
            if (unsatisfiedBoxes != null && !unsatisfiedBoxes.isEmpty()) {
                // Find minimum distance to any unsatisfied box this agent can move
                int minDistToBox = UNREACHABLE;
                for (Position boxPos : unsatisfiedBoxes) {
                    int dist = agentPos.manhattanDistance(boxPos) - 1; // -1 because agent needs to be adjacent
                    if (dist < 0) dist = 0;
                    minDistToBox = Math.min(minDistToBox, dist);
                }
                
                if (minDistToBox < UNREACHABLE) {
                    totalDistance += minDistToBox;
                }
            }
        }
        
        // Conflict penalty: boxes occupying wrong-type goals must be displaced
        // Each misplaced box on a goal needs at least 2 extra moves to clear
        // (move box off goal + surrounding rearrangement). This is admissible.
        for (Map.Entry<Character, List<Position>> goalEntry : boxGoalPositions.entrySet()) {
            char goalType = goalEntry.getKey();
            for (Position goalPos : goalEntry.getValue()) {
                Character boxAtGoal = state.getBoxes().get(goalPos);
                if (boxAtGoal != null && boxAtGoal != goalType) {
                    // Wrong box type blocking this goal — penalty of 2 (admissible lower bound)
                    totalDistance += 2;
                }
            }
        }
        
        return totalDistance;
    }
    
    /**
     * Gets the minimum distance from a position to any matching goal.
     * 
     * @param pos the position
     * @param boxType the box type
     * @return minimum distance, or UNREACHABLE if no path exists
     */
    private int getMinDistanceToGoal(Position pos, char boxType) {
        List<Position> goals = boxGoalPositions.get(boxType);
        if (goals == null || goals.isEmpty()) {
            return 0; // No goal for this type, consider it at goal
        }
        
        int minDist = UNREACHABLE;
        if (!inBounds(pos)) {
            return minDist;
        }
        for (Position goal : goals) {
            int[][] distMap = distanceMaps.get(goal);
            if (distMap != null) {
                int dist = distMap[pos.row][pos.col];
                minDist = Math.min(minDist, dist);
            }
        }
        
        return minDist;
    }
    
    /**
     * Gets the precomputed distance from a position to a specific goal.
     * 
     * @param from the starting position
     * @param goal the goal position
     * @return the distance, or UNREACHABLE if no path exists
     */
    public int getDistance(Position from, Position goal) {
        int[][] distMap = distanceMaps.get(goal);
        if (distMap == null) {
            return UNREACHABLE;
        }
        if (!inBounds(from)) {
            // Diagnostic: log first occurrence per session so we can find the source.
            logOutOfBoundsOnce("getDistance", from, goal);
            return UNREACHABLE;
        }
        return distMap[from.row][from.col];
    }
    
    /**
     * Checks if a position is reachable from any goal.
     * 
     * @param pos the position to check
     * @return true if the position is reachable
     */
    public boolean isReachable(Position pos) {
        if (!inBounds(pos)) {
            return false;
        }
        for (int[][] distMap : distanceMaps.values()) {
            if (distMap[pos.row][pos.col] < UNREACHABLE) {
                return true;
            }
        }
        return false;
    }

    /** Bounds check for the level grid. */
    private boolean inBounds(Position p) {
        return p != null
                && p.row >= 0 && p.row < level.getRows()
                && p.col >= 0 && p.col < level.getCols();
    }

    private static boolean OOB_LOGGED = false;

    /**
     * Log the first out-of-bounds Position seen in a session, with a short stack trace,
     * so we can pinpoint which BSP / planner code path is producing it.
     * Subsequent occurrences are silent to keep stderr clean.
     */
    private static void logOutOfBoundsOnce(String where, Position from, Position goal) {
        if (OOB_LOGGED) return;
        OOB_LOGGED = true;
        System.err.println("[TrueDistanceHeuristic] OOB lookup at " + where +
                " from=(" + from.row + "," + from.col + ") goal=(" +
                goal.row + "," + goal.col + ") -- returning UNREACHABLE.");
        StackTraceElement[] st = Thread.currentThread().getStackTrace();
        int n = Math.min(st.length, 12);
        for (int i = 1; i < n; i++) {
            System.err.println("    at " + st[i]);
        }
    }
}
