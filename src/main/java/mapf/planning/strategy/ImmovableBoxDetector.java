package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import java.util.*;

/**
 * Detects immovable boxes and computes distances considering them as walls.
 * 
 * Performance optimization: Precomputes BFS distance maps from all goal positions
 * at initialization time. Since immovable boxes never change position (no agent can
 * push them), these distance maps are valid for the entire solve. Subsequent distance
 * queries use O(1) lookup instead of O(N) BFS.
 * 
 * Uses BFS symmetry: d(A,B) = d(B,A) on undirected grids, so a distance map
 * precomputed FROM a goal also answers queries TO that goal.
 * 
 * For non-precomputed source positions (e.g., agent→box), a lazy cache stores
 * the BFS result on first computation for O(1) reuse.
 */
public class ImmovableBoxDetector {
    
    private Set<Position> immovableBoxPositions = null;
    private State cachedStateForImmovable = null;
    
    private Set<Position> preSatisfiedStaticGoals = null;
    private State cachedStateForStaticGoals = null;
    
    // --- Distance cache ---
    // Key: source position of BFS. Value: int[row][col] distance array.
    // Includes precomputed goal positions + lazily cached on-demand positions.
    private final Map<Position, int[][]> distanceMapCache = new HashMap<>();
    private boolean cacheInitialized = false;
    private int cachedRows = 0;
    private int cachedCols = 0;
    
    private static final int UNREACHABLE = Integer.MAX_VALUE;
    
    /**
     * Gets immovable box positions (cached per state).
     */
    public Set<Position> getImmovableBoxes(State state, Level level) {
        if (immovableBoxPositions != null && cachedStateForImmovable == state) {
            return immovableBoxPositions;
        }
        
        cachedStateForImmovable = state;
        immovableBoxPositions = new HashSet<>();
        
        // Find which colors have agents
        Set<Color> pushableColors = new HashSet<>();
        for (int i = 0; i < state.getNumAgents(); i++) {
            pushableColors.add(level.getAgentColor(i));
        }
        
        // Mark boxes with no matching agent color as immovable
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            char boxType = entry.getValue();
            Color boxColor = level.getBoxColor(boxType);
            if (!pushableColors.contains(boxColor)) {
                immovableBoxPositions.add(entry.getKey());
            }
        }
        
        return immovableBoxPositions;
    }
    
    /**
     * Finds pre-satisfied static goals (boxes at goals that cannot be moved).
     */
    public Set<Position> findPreSatisfiedStaticGoals(State state, Level level) {
        if (preSatisfiedStaticGoals != null && cachedStateForStaticGoals == state) {
            return preSatisfiedStaticGoals;
        }
        
        cachedStateForStaticGoals = state;
        preSatisfiedStaticGoals = new HashSet<>();
        
        Set<Position> immovableBoxes = getImmovableBoxes(state, level);
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType == '\0') continue;
                
                Position goalPos = new Position(row, col);
                Character actualBox = state.getBoxes().get(goalPos);
                
                if (actualBox != null && actualBox == goalType && immovableBoxes.contains(goalPos)) {
                    preSatisfiedStaticGoals.add(goalPos);
                }
            }
        }
        
        return preSatisfiedStaticGoals;
    }
    
    /**
     * Precomputes BFS distance maps from all goal positions, treating immovable boxes as walls.
     * 
     * Since immovable boxes cannot be moved (no agent of matching color exists),
     * their positions are constant throughout the solve. This means the distance maps
     * are valid forever and never need invalidation.
     * 
     * @param state initial state (used to determine immovable box positions)
     * @param level the level (used to find goal positions and walls)
     */
    public void initializeDistanceCache(State state, Level level) {
        if (cacheInitialized) return;
        
        this.cachedRows = level.getRows();
        this.cachedCols = level.getCols();
        
        // Ensure immovable boxes are computed
        getImmovableBoxes(state, level);
        
        // Precompute BFS from all goal positions (box goals + agent goals)
        int goalCount = 0;
        for (int row = 0; row < cachedRows; row++) {
            for (int col = 0; col < cachedCols; col++) {
                Position pos = new Position(row, col);
                
                // Box goals
                if (level.getBoxGoal(row, col) != '\0' && !distanceMapCache.containsKey(pos)) {
                    distanceMapCache.put(pos, computeFullBFS(pos, level));
                    goalCount++;
                }
                
                // Agent goals
                if (level.getAgentGoal(row, col) >= 0 && !distanceMapCache.containsKey(pos)) {
                    distanceMapCache.put(pos, computeFullBFS(pos, level));
                    goalCount++;
                }
            }
        }
        
        cacheInitialized = true;
        if (SearchConfig.isNormal()) {
            System.err.println("[ImmovableBoxDetector] Precomputed " + goalCount 
                + " distance maps (" + cachedRows + "×" + cachedCols + "), "
                + immovableBoxPositions.size() + " immovable boxes as walls");
        }
    }
    
    /**
     * Computes distance treating immovable boxes as walls.
     * 
     * Uses cached distance maps when available (O(1) lookup).
     * Falls back to on-demand BFS with lazy caching for uncached positions.
     * Exploits BFS symmetry: d(from, to) = d(to, from).
     */
    public int getDistanceWithImmovableBoxes(Position from, Position to, State state, Level level) {
        if (from.equals(to)) return 0;
        
        // Ensure immovable boxes are computed (needed for BFS)
        getImmovableBoxes(state, level);
        
        // Try cache: BFS from 'from' → lookup to
        int[][] mapFrom = distanceMapCache.get(from);
        if (mapFrom != null) {
            return mapFrom[to.row][to.col];
        }
        
        // Try cache with symmetry: BFS from 'to' → lookup from (d(A,B) = d(B,A))
        int[][] mapTo = distanceMapCache.get(to);
        if (mapTo != null) {
            return mapTo[from.row][from.col];
        }
        
        // Cache miss: compute BFS from 'from', cache for future reuse
        if (cacheInitialized) {
            int[][] newMap = computeFullBFS(from, level);
            distanceMapCache.put(from, newMap);
            return newMap[to.row][to.col];
        }
        
        // Fallback: cache not initialized, use single-pair BFS (backward compatible)
        return computeSinglePairBFS(from, to, level);
    }
    
    /**
     * Computes full BFS distance map from a source position.
     * Returns int[rows][cols] with distances to all reachable cells.
     * Unreachable cells have value UNREACHABLE (Integer.MAX_VALUE).
     */
    private int[][] computeFullBFS(Position source, Level level) {
        int[][] distances = new int[cachedRows][cachedCols];
        for (int[] row : distances) {
            Arrays.fill(row, UNREACHABLE);
        }
        
        Queue<Position> queue = new LinkedList<>();
        queue.add(source);
        distances[source.row][source.col] = 0;
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int dist = distances[current.row][current.col];
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (level.isWall(next)) continue;
                if (distances[next.row][next.col] <= dist + 1) continue;
                if (immovableBoxPositions.contains(next)) continue;
                
                distances[next.row][next.col] = dist + 1;
                queue.add(next);
            }
        }
        
        return distances;
    }
    
    /**
     * Single-pair BFS for backward compatibility when cache is not initialized.
     * Only used as fallback — should not be called in normal operation.
     */
    private int computeSinglePairBFS(Position from, Position to, Level level) {
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> visited = new HashMap<>();
        
        queue.add(from);
        visited.put(from, 0);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int dist = visited.get(current);
            
            if (current.equals(to)) return dist;
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (visited.containsKey(next)) continue;
                if (level.isWall(next)) continue;
                if (immovableBoxPositions != null && immovableBoxPositions.contains(next)) continue;
                
                visited.put(next, dist + 1);
                queue.add(next);
            }
        }
        
        return UNREACHABLE;
    }
}
