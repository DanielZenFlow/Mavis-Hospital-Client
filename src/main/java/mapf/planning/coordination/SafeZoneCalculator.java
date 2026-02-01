package mapf.planning.coordination;

import mapf.domain.*;
import mapf.planning.SearchConfig;

import java.util.*;

/**
 * Calculates safe parking zones for agents in MAPF.
 * 
 * A "safe zone" is a position where an agent can park without blocking
 * any other agent's current or future work.
 * 
 * Key concepts from MAPF literature:
 * 1. Working Area: All positions an agent might need to visit to complete its task
 * 2. Bottleneck: A position whose removal disconnects parts of the graph (articulation point)
 * 3. Siding/Bypass: A position off the main corridors where agents can park safely
 * 
 * The algorithm:
 * 1. Compute global working area = union of all other agents' working areas
 * 2. Find positions NOT in the global working area
 * 3. Prefer: dead-ends > open areas > positions far from working area
 * 4. Avoid: bottlenecks, corridor entrances, any position in working area
 */
public class SafeZoneCalculator {

    // Cache for computed working areas (invalidated when state changes significantly)
    private Map<Integer, Set<Position>> agentWorkingAreaCache = new HashMap<>();
    private State cachedState = null;

    /**
     * Computes the "working area" for an agent - all positions it might need.
     * This includes:
     * 1. Path from agent to its assigned box
     * 2. Path from box to box goal
     * 3. Path from agent to agent goal (if exists)
     * 4. Buffer zones around these paths
     */
    public Set<Position> computeAgentWorkingArea(int agentId, State state, Level level) {
        Set<Position> workingArea = new HashSet<>();
        
        Position agentPos = state.getAgentPosition(agentId);
        Color agentColor = level.getAgentColor(agentId);
        
        // Find all box goals this agent needs to handle
        List<Position> boxGoals = new ArrayList<>();
        List<Character> boxTypes = new ArrayList<>();
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Color boxColor = level.getBoxColor(goalType);
                    if (boxColor == agentColor) {
                        Position goalPos = new Position(row, col);
                        // Check if goal not yet satisfied
                        char currentBox = state.getBoxAt(goalPos);
                        if (currentBox != goalType) {
                            boxGoals.add(goalPos);
                            boxTypes.add(goalType);
                        }
                    }
                }
            }
        }
        
        // For each unsatisfied box goal, compute working area
        for (int i = 0; i < boxGoals.size(); i++) {
            Position goalPos = boxGoals.get(i);
            char boxType = boxTypes.get(i);
            
            // Find the box
            Position boxPos = findNearestBox(boxType, goalPos, state);
            if (boxPos == null) continue;
            
            // 1. Agent to box path
            Set<Position> agentToBox = findAllPathPositions(agentPos, boxPos, level);
            workingArea.addAll(agentToBox);
            
            // 2. Box to goal path (wider because box needs to be pushed)
            Set<Position> boxToGoal = findAllPathPositions(boxPos, goalPos, level);
            workingArea.addAll(boxToGoal);
            
            // 3. Add buffer around box path (agent needs space to push)
            for (Position p : new HashSet<>(boxToGoal)) {
                for (Direction dir : Direction.values()) {
                    Position neighbor = p.move(dir);
                    if (!level.isWall(neighbor)) {
                        workingArea.add(neighbor);
                    }
                }
            }
        }
        
        // Agent goal path
        Position agentGoal = findAgentGoalPosition(agentId, level);
        if (agentGoal != null && !agentPos.equals(agentGoal)) {
            Set<Position> agentPath = findAllPathPositions(agentPos, agentGoal, level);
            workingArea.addAll(agentPath);
        }
        
        return workingArea;
    }

    /**
     * Computes the global working area - union of all other agents' working areas.
     * This represents all positions that should be avoided when parking.
     */
    public Set<Position> computeGlobalWorkingArea(int excludeAgentId, State state, Level level) {
        Set<Position> globalArea = new HashSet<>();
        
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            if (agentId == excludeAgentId) continue;
            
            Set<Position> agentArea = computeAgentWorkingArea(agentId, state, level);
            globalArea.addAll(agentArea);
        }
        
        return globalArea;
    }

    /**
     * Finds a safe parking position for an agent.
     * 
     * Priority:
     * 1. Dead-ends (degree 1) - best parking spots
     * 2. Open areas (degree 3+) NOT in global working area
     * 3. Positions far from global working area
     * 
     * Avoid:
     * - Any position in global working area
     * - Bottleneck positions (articulation points)
     * - Corridor positions (degree 2)
     */
    public Position findSafePosition(int agentId, State state, Level level) {
        Position currentPos = state.getAgentPosition(agentId);
        
        // Compute global working area
        Set<Position> globalWorkingArea = computeGlobalWorkingArea(agentId, state, level);
        
        // BFS to find positions, scoring each
        Queue<Position> queue = new LinkedList<>();
        Map<Position, Integer> distances = new HashMap<>();
        
        queue.add(currentPos);
        distances.put(currentPos, 0);
        
        Position bestPosition = null;
        int bestScore = Integer.MIN_VALUE;
        
        int maxSearch = level.getRows() * level.getCols();
        int searched = 0;
        
        while (!queue.isEmpty() && searched < maxSearch) {
            Position pos = queue.poll();
            int distance = distances.get(pos);
            searched++;
            
            // Skip current position and occupied positions
            if (!pos.equals(currentPos) && 
                state.getBoxAt(pos) == '\0' && 
                !isOccupiedByOtherAgent(pos, agentId, state)) {
                
                int score = scorePosition(pos, distance, globalWorkingArea, state, level, agentId);
                
                if (score > bestScore) {
                    bestScore = score;
                    bestPosition = pos;
                }
            }
            
            // Explore neighbors
            for (Direction dir : Direction.values()) {
                Position next = pos.move(dir);
                if (!distances.containsKey(next) && !level.isWall(next)) {
                    distances.put(next, distance + 1);
                    queue.add(next);
                }
            }
        }
        
        if (bestPosition != null && bestScore > -500) {
            logNormal("[SAFE-ZONE] Found safe position " + bestPosition + 
                    " for Agent " + agentId + " (score=" + bestScore + ")");
        } else {
            logNormal("[SAFE-ZONE] No good safe position found for Agent " + agentId);
        }
        
        return bestPosition;
    }

    /**
     * Scores a potential parking position.
     * Higher score = better parking spot.
     */
    private int scorePosition(Position pos, int distance, Set<Position> globalWorkingArea,
                             State state, Level level, int agentId) {
        int score = 0;
        int freeNeighbors = countFreeNeighbors(pos, level);
        int passableNeighbors = countPassableNeighbors(pos, state, level, agentId);
        
        // === CRITICAL: Position in global working area = very bad ===
        if (globalWorkingArea.contains(pos)) {
            score -= 1000;
        }
        
        // === Check if position is a bottleneck ===
        if (isBottleneck(pos, level, globalWorkingArea)) {
            score -= 500;
        }
        
        // === Check if position blocks entry to safe areas ===
        if (blocksAccessToSafeArea(pos, level, globalWorkingArea)) {
            score -= 300;
        }
        
        // === Corridor (degree 2) = bad ===
        if (freeNeighbors == 2) {
            score -= 200;
        }
        
        // === Dead-end (degree 1) = excellent parking ===
        if (freeNeighbors == 1) {
            score += 200;
        }
        
        // === Open area (degree 3+) = good ===
        if (freeNeighbors >= 3 && passableNeighbors >= 3) {
            score += 50 + passableNeighbors * 10;
        }
        
        // === Distance penalty (prefer closer positions) ===
        score -= distance * 2;
        
        // === Distance FROM working area = bonus ===
        int minDistToWorkingArea = computeMinDistanceToSet(pos, globalWorkingArea, level);
        score += minDistToWorkingArea * 5;
        
        return score;
    }

    /**
     * Checks if a position is a bottleneck (articulation point).
     * Simplified check: if removing this position would disconnect
     * important areas from each other.
     */
    private boolean isBottleneck(Position pos, Level level, Set<Position> workingArea) {
        int freeNeighbors = countFreeNeighbors(pos, level);
        
        // Degree 1 or 2 positions are not bottlenecks (they're endpoints or corridors)
        if (freeNeighbors <= 2) {
            return false;
        }
        
        // For degree 3+ positions, check if they connect distinct regions
        // Simple heuristic: check if free neighbors can reach each other without this position
        List<Position> neighbors = new ArrayList<>();
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (!level.isWall(neighbor)) {
                neighbors.add(neighbor);
            }
        }
        
        if (neighbors.size() < 2) return false;
        
        // Check if first neighbor can reach others without passing through pos
        Position start = neighbors.get(0);
        Set<Position> reachable = bfsWithout(start, pos, level, 50);
        
        for (int i = 1; i < neighbors.size(); i++) {
            if (!reachable.contains(neighbors.get(i))) {
                // This neighbor is disconnected - pos is a bottleneck
                return true;
            }
        }
        
        return false;
    }

    /**
     * Checks if parking at this position would block access to safe areas.
     */
    private boolean blocksAccessToSafeArea(Position pos, Level level, Set<Position> workingArea) {
        // Find dead-ends and open areas near this position
        int freeNeighbors = countFreeNeighbors(pos, level);
        
        // Only positions with degree 2-3 might block access
        if (freeNeighbors < 2 || freeNeighbors > 3) {
            return false;
        }
        
        // Check each neighbor - if any neighbor is a safe area entry point, we'd be blocking it
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (level.isWall(neighbor)) continue;
            
            int neighborDegree = countFreeNeighbors(neighbor, level);
            
            // If neighbor is a dead-end, we'd be blocking the only way in
            if (neighborDegree == 1) {
                return true;
            }
            
            // If neighbor leads to a cluster of safe positions not in working area
            if (neighborDegree >= 3 && !workingArea.contains(neighbor)) {
                // Check if this is the only way to reach that area
                Set<Position> reachableWithout = bfsWithout(neighbor, pos, level, 20);
                int safeCount = 0;
                for (Position p : reachableWithout) {
                    if (!workingArea.contains(p) && countFreeNeighbors(p, level) >= 3) {
                        safeCount++;
                    }
                }
                if (safeCount >= 3) {
                    // There's a safe zone through this neighbor, check if we're the only entry
                    boolean hasAlternativeEntry = false;
                    for (Direction d2 : Direction.values()) {
                        Position alt = neighbor.move(d2);
                        if (!level.isWall(alt) && !alt.equals(pos)) {
                            hasAlternativeEntry = true;
                            break;
                        }
                    }
                    if (!hasAlternativeEntry) {
                        return true;
                    }
                }
            }
        }
        
        return false;
    }

    /**
     * BFS that avoids a specific position.
     */
    private Set<Position> bfsWithout(Position start, Position avoid, Level level, int maxSteps) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        
        queue.add(start);
        visited.add(start);
        visited.add(avoid); // Mark as visited to avoid it
        
        int steps = 0;
        while (!queue.isEmpty() && steps < maxSteps) {
            Position pos = queue.poll();
            steps++;
            
            for (Direction dir : Direction.values()) {
                Position next = pos.move(dir);
                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }
        
        visited.remove(avoid);
        return visited;
    }

    /**
     * Finds all positions on shortest paths between two points.
     * Returns the union of all shortest paths (not just one).
     */
    private Set<Position> findAllPathPositions(Position start, Position goal, Level level) {
        Set<Position> pathPositions = new HashSet<>();
        
        // BFS from start to find distances
        Map<Position, Integer> distFromStart = bfsDistances(start, level, level.getRows() * level.getCols());
        
        // BFS from goal to find distances
        Map<Position, Integer> distFromGoal = bfsDistances(goal, level, level.getRows() * level.getCols());
        
        if (!distFromStart.containsKey(goal)) {
            // No path exists
            return pathPositions;
        }
        
        int shortestDist = distFromStart.get(goal);
        
        // A position is on a shortest path if distFromStart[p] + distFromGoal[p] == shortestDist
        for (Map.Entry<Position, Integer> entry : distFromStart.entrySet()) {
            Position pos = entry.getKey();
            int dStart = entry.getValue();
            Integer dGoal = distFromGoal.get(pos);
            
            if (dGoal != null && dStart + dGoal == shortestDist) {
                pathPositions.add(pos);
            }
        }
        
        return pathPositions;
    }

    /**
     * BFS to compute distances from a position.
     */
    private Map<Position, Integer> bfsDistances(Position start, Level level, int maxSteps) {
        Map<Position, Integer> distances = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        
        queue.add(start);
        distances.put(start, 0);
        
        while (!queue.isEmpty() && distances.size() < maxSteps) {
            Position pos = queue.poll();
            int dist = distances.get(pos);
            
            for (Direction dir : Direction.values()) {
                Position next = pos.move(dir);
                if (!distances.containsKey(next) && !level.isWall(next)) {
                    distances.put(next, dist + 1);
                    queue.add(next);
                }
            }
        }
        
        return distances;
    }

    /**
     * Computes minimum distance from a position to any position in a set.
     */
    private int computeMinDistanceToSet(Position pos, Set<Position> targetSet, Level level) {
        if (targetSet.isEmpty()) return 100; // No targets = very far
        if (targetSet.contains(pos)) return 0;
        
        // BFS from pos
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Integer> dist = new HashMap<>();
        
        queue.add(pos);
        visited.add(pos);
        dist.put(pos, 0);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int d = dist.get(current);
            
            if (targetSet.contains(current)) {
                return d;
            }
            
            if (d > 20) break; // Don't search too far
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    dist.put(next, d + 1);
                    queue.add(next);
                }
            }
        }
        
        return 20; // Far away
    }

    /**
     * Finds the nearest box of a given type to a goal position.
     */
    private Position findNearestBox(char boxType, Position goalPos, State state) {
        Position nearest = null;
        int minDist = Integer.MAX_VALUE;
        
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                int dist = manhattanDistance(entry.getKey(), goalPos);
                if (dist < minDist) {
                    minDist = dist;
                    nearest = entry.getKey();
                }
            }
        }
        
        return nearest;
    }

    private int manhattanDistance(Position a, Position b) {
        return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
    }

    private Position findAgentGoalPosition(int agentId, Level level) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    return new Position(row, col);
                }
            }
        }
        return null;
    }

    private int countFreeNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            if (!level.isWall(pos.move(dir))) {
                count++;
            }
        }
        return count;
    }

    private int countPassableNeighbors(Position pos, State state, Level level, int excludeAgentId) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (level.isWall(neighbor)) continue;
            if (state.getBoxAt(neighbor) != '\0') continue;
            if (isOccupiedByOtherAgent(neighbor, excludeAgentId, state)) continue;
            count++;
        }
        return count;
    }

    private boolean isOccupiedByOtherAgent(Position pos, int excludeAgentId, State state) {
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (i != excludeAgentId && state.getAgentPosition(i).equals(pos)) {
                return true;
            }
        }
        return false;
    }

    private void logNormal(String msg) {
        if (SearchConfig.isNormal()) {
            System.err.println(msg);
        }
    }
}
