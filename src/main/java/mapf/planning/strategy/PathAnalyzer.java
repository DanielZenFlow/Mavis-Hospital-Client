package mapf.planning.strategy;

import mapf.domain.*;

import java.util.*;

/**
 * Handles path finding and position analysis.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Responsibilities:
 * - findParkingPosition: Find valid parking positions for agents
 * - planAgentPath: Plan path for agent to a position
 * - findPathIgnoringDynamicObstacles: Path finding ignoring dynamic obstacles
 * - findCriticalPositions: Find positions critical to a goal
 */
public class PathAnalyzer {

    /** Articulation points of the free-space graph. Parking on these can split the map. */
    private Set<Position> articulationPoints = Collections.emptySet();

    /** Set the pre-computed articulation points for parking avoidance. */
    public void setArticulationPoints(Set<Position> ap) {
        this.articulationPoints = (ap != null) ? ap : Collections.emptySet();
    }

    /** Get the articulation points set. */
    public Set<Position> getArticulationPoints() {
        return articulationPoints;
    }

    /**
     * Plans a path for an agent to a target position.
     * Returns list of MOVE actions only.
     */
    public List<Action> planAgentPath(int agentId, Position targetPos, State state, Level level, int numAgents) {
        Position startPos = state.getAgentPosition(agentId);
        if (startPos.equals(targetPos)) {
            return Collections.emptyList();
        }

        Map<Position, Position> cameFrom = new HashMap<>();
        Map<Position, Direction> moveDir = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();

        queue.add(startPos);
        visited.add(startPos);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (current.equals(targetPos)) {
                return reconstructMovePath(startPos, targetPos, cameFrom, moveDir);
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (visited.contains(next))
                    continue;
                if (level.isWall(next))
                    continue;
                if (state.getBoxes().containsKey(next))
                    continue;

                boolean blocked = false;
                for (int i = 0; i < numAgents; i++) {
                    if (i != agentId && state.getAgentPosition(i).equals(next)) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked)
                    continue;

                visited.add(next);
                cameFrom.put(next, current);
                moveDir.put(next, dir);
                queue.add(next);
            }
        }

        return null;
    }

    /**
     * Finds a path ignoring dynamic obstacles (boxes and agents).
     * Uses A* with Manhattan distance heuristic.
     */
    public List<Position> findPathIgnoringDynamicObstacles(Position start, Position goal, Level level) {
        if (start.equals(goal)) {
            return Collections.singletonList(start);
        }

        Map<Position, Position> cameFrom = new HashMap<>();
        Map<Position, Integer> gScore = new HashMap<>();
        PriorityQueue<Position> openSet = new PriorityQueue<>(
                Comparator.comparingInt(p -> gScore.getOrDefault(p, Integer.MAX_VALUE) + manhattan(p, goal)));

        gScore.put(start, 0);
        openSet.add(start);

        while (!openSet.isEmpty()) {
            Position current = openSet.poll();

            if (current.equals(goal)) {
                return reconstructPositionPath(start, goal, cameFrom);
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (level.isWall(next))
                    continue;

                int tentativeG = gScore.get(current) + 1;

                if (tentativeG < gScore.getOrDefault(next, Integer.MAX_VALUE)) {
                    cameFrom.put(next, current);
                    gScore.put(next, tentativeG);
                    openSet.remove(next);
                    openSet.add(next);
                }
            }
        }

        return null;
    }

    /**
     * Finds a parking position for an agent that doesn't block critical positions.
     * Uses a scoring system that balances distance with position quality:
     * - Dead-end positions (1 passable neighbor): +30 score bonus (out of the way)
     * - Corridor positions (2 passable neighbors): -10 score penalty (may block)
     * - Articulation points: -20 score penalty (may split map)
     * - Distance penalty: -dist (closer is better)
     * 
     * Higher score = better parking position.
     */
    public Position findParkingPosition(int agentId, State state, Level level, int numAgents,
            Set<Position> criticalPositions, Set<Position> satisfiedGoalPositions) {
        
        Position agentPos = state.getAgentPosition(agentId);
        Position bestPosition = null;
        int bestScore = Integer.MIN_VALUE;

        Set<Position> allAgentPositions = new HashSet<>();
        for (int i = 0; i < numAgents; i++) {
            allAgentPositions.add(state.getAgentPosition(i));
        }

        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Integer> distances = new HashMap<>();

        queue.add(agentPos);
        visited.add(agentPos);
        distances.put(agentPos, 0);

        int maxSearchDistance = 30;

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int dist = distances.get(current);

            if (dist > maxSearchDistance)
                continue;

            if (isValidParkingPosition(current, agentPos, state, level, criticalPositions,
                    satisfiedGoalPositions, allAgentPositions)) {
                int score = -dist * 3; // base: closer is much better (distance weighted 3x)
                
                int passableNeighbors = countPassableNeighbors(current, level);
                if (passableNeighbors == 1) {
                    score += 8; // dead-end: good parking, but not worth going far
                } else if (passableNeighbors == 2) {
                    score -= 3; // corridor: mild penalty
                }
                // 3+ neighbors: junction, neutral (score += 0)
                
                if (articulationPoints.contains(current)) {
                    score -= 5; // avoid splitting the map
                }
                
                if (score > bestScore) {
                    bestScore = score;
                    bestPosition = current;
                }
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);

                if (visited.contains(next))
                    continue;
                if (level.isWall(next))
                    continue;
                if (state.getBoxes().containsKey(next))
                    continue;

                boolean occupied = false;
                for (int i = 0; i < numAgents; i++) {
                    if (i != agentId && state.getAgentPosition(i).equals(next)) {
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                    continue;

                visited.add(next);
                distances.put(next, dist + 1);
                queue.add(next);
            }
        }

        return bestPosition;
    }

    /**
     * Checks if a position is valid for parking.
     */
    public boolean isValidParkingPosition(Position pos, Position agentCurrentPos, State state,
            Level level, Set<Position> criticalPositions, Set<Position> satisfiedGoalPositions,
            Set<Position> allAgentPositions) {
        
        if (pos.equals(agentCurrentPos))
            return false;
        if (level.isWall(pos))
            return false;
        if (state.getBoxes().containsKey(pos))
            return false;
        if (criticalPositions.contains(pos))
            return false;
        if (satisfiedGoalPositions.contains(pos))
            return false;
        if (allAgentPositions.contains(pos))
            return false;

        int freeNeighbors = countFreeNeighbors(pos, state, level);
        // In a pull-supporting domain, agents can always walk out of dead-ends.
        // Only reject completely blocked positions (0 free neighbors).
        if (freeNeighbors < 1)
            return false;

        char boxGoal = level.getBoxGoal(pos);
        if (boxGoal != '\0') {
            // Avoid parking on box goal positions
            return false;
        }

        return true;
    }

    /**
     * Finds critical positions for reaching a goal with a specific box.
     */
    public Set<Position> findCriticalPositions(State state, Level level, int agentId,
            Position goalPos, Position boxPos, Set<Position> satisfiedGoalPositions) {
        
        Set<Position> critical = new HashSet<>();

        List<Position> path = findPathIgnoringDynamicObstacles(boxPos, goalPos, level);
        if (path != null) {
            for (Position p : path) {
                critical.add(p);
                for (Direction dir : Direction.values()) {
                    Position neighbor = p.move(dir);
                    if (!level.isWall(neighbor)) {
                        critical.add(neighbor);
                    }
                }
            }
        }

        Position agentPos = state.getAgentPosition(agentId);
        List<Position> agentToBox = findPathIgnoringDynamicObstacles(agentPos, boxPos, level);
        if (agentToBox != null) {
            for (Position p : agentToBox) {
                critical.add(p);
            }
        }

        critical.removeAll(satisfiedGoalPositions);

        return critical;
    }

    /**
     * Finds critical positions for an agent goal (without box).
     */
    public Set<Position> findCriticalPositionsForAgentGoal(State state, Level level, int agentId,
            Position goalPos, Set<Position> satisfiedGoalPositions) {
        
        Set<Position> critical = new HashSet<>();
        Position agentPos = state.getAgentPosition(agentId);

        List<Position> path = findPathIgnoringDynamicObstacles(agentPos, goalPos, level);
        if (path != null) {
            for (Position p : path) {
                critical.add(p);
            }
        }

        critical.removeAll(satisfiedGoalPositions);

        return critical;
    }

    /**
     * Counts free neighbors around a position.
     */
    public int countFreeNeighbors(Position pos, State state, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (!level.isWall(neighbor) && !state.getBoxes().containsKey(neighbor)) {
                count++;
            }
        }
        return count;
    }

    /**
     * Counts passable neighbors (walls only).
     */
    public int countPassableNeighbors(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position neighbor = pos.move(dir);
            if (!level.isWall(neighbor)) {
                count++;
            }
        }
        return count;
    }

    /**
     * Checks if a position is in a corridor (exactly 2 passable neighbors).
     */
    public boolean isInCorridor(Position pos, Level level) {
        return countPassableNeighbors(pos, level) == 2;
    }

    /**
     * Gets the corridor depth from a position.
     */
    public int getCorridorDepth(Position pos, Level level) {
        if (!isInCorridor(pos, level)) {
            return 0;
        }

        int depth = 0;
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(pos);
        visited.add(pos);

        while (!queue.isEmpty()) {
            Position current = queue.poll();

            if (!isInCorridor(current, level)) {
                return depth;
            }

            depth++;

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (!visited.contains(next) && !level.isWall(next)) {
                    visited.add(next);
                    queue.add(next);
                }
            }
        }

        return depth;
    }

    private List<Action> reconstructMovePath(Position start, Position goal,
            Map<Position, Position> cameFrom, Map<Position, Direction> moveDir) {
        List<Action> path = new ArrayList<>();
        Position current = goal;

        while (!current.equals(start)) {
            Direction dir = moveDir.get(current);
            path.add(Action.move(dir));
            current = cameFrom.get(current);
        }

        Collections.reverse(path);
        return path;
    }

    private List<Position> reconstructPositionPath(Position start, Position goal,
            Map<Position, Position> cameFrom) {
        List<Position> path = new ArrayList<>();
        Position current = goal;

        while (!current.equals(start)) {
            path.add(current);
            current = cameFrom.get(current);
        }
        path.add(start);

        Collections.reverse(path);
        return path;
    }

    private int manhattan(Position a, Position b) {
        return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
    }
}
