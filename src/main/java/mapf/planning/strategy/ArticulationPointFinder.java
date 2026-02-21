package mapf.planning.strategy;

import mapf.domain.*;

import java.util.*;

/**
 * Computes articulation points (cut vertices) of the free-space graph using Tarjan's algorithm.
 * 
 * An articulation point is a cell whose removal would disconnect the free-space graph.
 * Parking a box or agent on an articulation point can split the map, trapping agents
 * on one side. Used by PathAnalyzer to avoid selecting such positions for parking.
 * 
 * The free-space graph is static (depends only on walls), so this is computed once per level.
 * Time complexity: O(V + E) where V = free cells, E = adjacent cell pairs.
 */
public class ArticulationPointFinder {
    
    /**
     * Finds all articulation points in the free-space graph of the given level.
     * Free space = all cells that are not walls.
     * Adjacency = 4-directional (N/S/E/W).
     * 
     * @param level The level defining walls
     * @return Set of positions that are articulation points
     */
    public static Set<Position> findArticulationPoints(Level level) {
        int rows = level.getRows();
        int cols = level.getCols();
        
        // Build position → index mapping for free cells
        List<Position> freeCells = new ArrayList<>();
        Map<Position, Integer> posToIndex = new HashMap<>();
        
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (!level.isWall(r, c)) {
                    Position p = new Position(r, c);
                    posToIndex.put(p, freeCells.size());
                    freeCells.add(p);
                }
            }
        }
        
        int n = freeCells.size();
        if (n <= 2) return Collections.emptySet();
        
        // Build adjacency list
        @SuppressWarnings("unchecked")
        List<Integer>[] adj = new List[n];
        for (int i = 0; i < n; i++) {
            adj[i] = new ArrayList<>(4);
        }
        
        for (int i = 0; i < n; i++) {
            Position p = freeCells.get(i);
            for (Direction dir : Direction.values()) {
                Position neighbor = p.move(dir);
                Integer neighborIdx = posToIndex.get(neighbor);
                if (neighborIdx != null) {
                    adj[i].add(neighborIdx);
                }
            }
        }
        
        // Tarjan's algorithm for articulation points (iterative to avoid stack overflow)
        Set<Position> articulationPoints = new HashSet<>();
        int[] disc = new int[n];
        int[] low = new int[n];
        boolean[] visited = new boolean[n];
        int[] parent = new int[n];
        Arrays.fill(disc, -1);
        Arrays.fill(low, -1);
        Arrays.fill(parent, -1);
        
        int timer = 0;
        
        // Process each connected component
        for (int start = 0; start < n; start++) {
            if (visited[start]) continue;
            
            // Iterative DFS using explicit stack
            // Stack frame: (node, neighborIndex, isReturning)
            Deque<int[]> stack = new ArrayDeque<>();
            
            visited[start] = true;
            disc[start] = timer;
            low[start] = timer;
            timer++;
            int rootChildren = 0;
            
            stack.push(new int[]{start, 0}); // node, next neighbor index
            
            while (!stack.isEmpty()) {
                int[] frame = stack.peek();
                int u = frame[0];
                int nextIdx = frame[1];
                
                if (nextIdx < adj[u].size()) {
                    frame[1]++; // advance to next neighbor
                    int v = adj[u].get(nextIdx);
                    
                    if (!visited[v]) {
                        visited[v] = true;
                        parent[v] = u;
                        disc[v] = timer;
                        low[v] = timer;
                        timer++;
                        
                        if (u == start) rootChildren++;
                        
                        stack.push(new int[]{v, 0});
                    } else if (v != parent[u]) {
                        // Back edge: update low
                        low[u] = Math.min(low[u], disc[v]);
                    }
                } else {
                    // Done with all neighbors of u
                    stack.pop();
                    
                    if (!stack.isEmpty()) {
                        int p = parent[u];
                        low[p] = Math.min(low[p], low[u]);
                        
                        // Articulation point check for non-root
                        if (parent[p] != -1 && low[u] >= disc[p]) {
                            articulationPoints.add(freeCells.get(p));
                        }
                    }
                }
            }
            
            // Root is an articulation point if it has 2+ children in DFS tree
            if (rootChildren >= 2) {
                articulationPoints.add(freeCells.get(start));
            }
        }
        
        return articulationPoints;
    }
    
    /**
     * Finds articulation points considering both walls and a set of extra obstacles
     * (e.g., immovable boxes). This is the dynamic version for state-dependent analysis.
     * 
     * @param level The level defining walls
     * @param extraWalls Additional positions to treat as walls
     * @return Set of positions that are articulation points
     */
    public static Set<Position> findArticulationPoints(Level level, Set<Position> extraWalls) {
        if (extraWalls == null || extraWalls.isEmpty()) {
            return findArticulationPoints(level);
        }
        
        int rows = level.getRows();
        int cols = level.getCols();
        
        List<Position> freeCells = new ArrayList<>();
        Map<Position, Integer> posToIndex = new HashMap<>();
        
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                Position p = new Position(r, c);
                if (!level.isWall(r, c) && !extraWalls.contains(p)) {
                    posToIndex.put(p, freeCells.size());
                    freeCells.add(p);
                }
            }
        }
        
        int n = freeCells.size();
        if (n <= 2) return Collections.emptySet();
        
        @SuppressWarnings("unchecked")
        List<Integer>[] adj = new List[n];
        for (int i = 0; i < n; i++) {
            adj[i] = new ArrayList<>(4);
        }
        
        for (int i = 0; i < n; i++) {
            Position p = freeCells.get(i);
            for (Direction dir : Direction.values()) {
                Position neighbor = p.move(dir);
                Integer neighborIdx = posToIndex.get(neighbor);
                if (neighborIdx != null) {
                    adj[i].add(neighborIdx);
                }
            }
        }
        
        // Same Tarjan's algorithm
        Set<Position> articulationPoints = new HashSet<>();
        int[] disc = new int[n];
        int[] low = new int[n];
        boolean[] visited = new boolean[n];
        int[] parentArr = new int[n];
        Arrays.fill(disc, -1);
        Arrays.fill(low, -1);
        Arrays.fill(parentArr, -1);
        
        int timer = 0;
        
        for (int start = 0; start < n; start++) {
            if (visited[start]) continue;
            
            Deque<int[]> stack = new ArrayDeque<>();
            visited[start] = true;
            disc[start] = timer;
            low[start] = timer;
            timer++;
            int rootChildren = 0;
            
            stack.push(new int[]{start, 0});
            
            while (!stack.isEmpty()) {
                int[] frame = stack.peek();
                int u = frame[0];
                int nextIdx = frame[1];
                
                if (nextIdx < adj[u].size()) {
                    frame[1]++;
                    int v = adj[u].get(nextIdx);
                    
                    if (!visited[v]) {
                        visited[v] = true;
                        parentArr[v] = u;
                        disc[v] = timer;
                        low[v] = timer;
                        timer++;
                        
                        if (u == start) rootChildren++;
                        stack.push(new int[]{v, 0});
                    } else if (v != parentArr[u]) {
                        low[u] = Math.min(low[u], disc[v]);
                    }
                } else {
                    stack.pop();
                    if (!stack.isEmpty()) {
                        int p = parentArr[u];
                        low[p] = Math.min(low[p], low[u]);
                        if (parentArr[p] != -1 && low[u] >= disc[p]) {
                            articulationPoints.add(freeCells.get(p));
                        }
                    }
                }
            }
            
            if (rootChildren >= 2) {
                articulationPoints.add(freeCells.get(start));
            }
        }
        
        return articulationPoints;
    }
}
