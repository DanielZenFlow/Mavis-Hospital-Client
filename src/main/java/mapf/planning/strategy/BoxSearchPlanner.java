package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;
import mapf.planning.spacetime.ReservationTable;

import java.util.*;

/**
 * Handles A* search algorithms for box and agent goal planning.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 * 
 * Responsibilities:
 * - searchForSubgoal: A* search to move a box to a goal position
 * - searchForAgentGoal: A* search to move an agent to its goal position
 * - searchForDisplacement: Search for temporary box displacement
 * - planBoxDisplacement: Plan path to displace a blocking box
 */
public class BoxSearchPlanner {

    private final Heuristic heuristic;

    public BoxSearchPlanner(Heuristic heuristic) {
        this.heuristic = heuristic;
    }

    /**
     * A* search to move a specific box to a goal position.
     * Uses optimized StateKey that only tracks agent and target box positions.
     * 
     * LAYER 2 ENHANCEMENT: Protected goals are treated as immovable.
     */
    public List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, Set<Position> frozenGoals) {
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        int h = getDistance(boxStart, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart);
        StateKey startKey = new StateKey(initialState, agentId, boxStart);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;

        while (!openList.isEmpty() && exploredCount < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            SearchNode current = openList.poll();
            exploredCount++;

            // Check if goal is achieved
            Character boxAtGoal = current.state.getBoxes().get(goalPos);
            if (boxAtGoal != null && boxAtGoal == boxType) {
                return reconstructPath(current);
            }

            // Expand node
            for (Action action : PlanningUtils.getAllActions()) {
                if (action.type == Action.ActionType.NOOP) {
                    continue;
                }

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                // Skip actions that would disturb satisfied goals
                if (wouldDisturbSatisfiedGoal(action, agentId, current.state, frozenGoals)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);

                Position newTargetBoxPos = findTargetBoxPosition(newState, boxType, current.targetBoxPos);
                StateKey newKey = new StateKey(newState, agentId, newTargetBoxPos);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                int newH = (newTargetBoxPos != null) ? getDistance(newTargetBoxPos, goalPos, level) : 0;
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newTargetBoxPos);
                openList.add(newNode);
            }
        }

        return null;
    }

    /**
     * Space-Time A* search for subgoal with reservation table.
     * Avoids positions reserved by higher-priority agents.
     */
    public List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, Set<Position> frozenGoals,
            ReservationTable reservations, int startTime) {
        
        if (reservations == null) {
            return searchForSubgoal(agentId, boxStart, goalPos, boxType, initialState, level, frozenGoals);
        }
        
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKeyWithTime, Integer> bestG = new HashMap<>();

        int h = getDistance(boxStart, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart, startTime);
        StateKeyWithTime startKey = new StateKeyWithTime(initialState, agentId, boxStart, startTime);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;

        while (!openList.isEmpty() && exploredCount < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            SearchNode current = openList.poll();
            exploredCount++;

            // Check if goal is achieved
            Character boxAtGoal = current.state.getBoxes().get(goalPos);
            if (boxAtGoal != null && boxAtGoal == boxType) {
                return reconstructPath(current);
            }

            int nextTime = current.time + 1;

            // Expand node (including Wait action)
            for (Action action : PlanningUtils.getAllActions()) {
                State newState;
                Position newAgentPos;
                Position newBoxPos = current.targetBoxPos;
                
                if (action.type == Action.ActionType.NOOP) {
                    // Wait action - stay in place
                    newState = current.state;
                    newAgentPos = current.state.getAgentPosition(agentId);
                } else {
                    if (!current.state.isApplicable(action, agentId, level)) {
                        continue;
                    }
                    if (wouldDisturbSatisfiedGoal(action, agentId, current.state, frozenGoals)) {
                        continue;
                    }
                    newState = current.state.apply(action, agentId);
                    newAgentPos = newState.getAgentPosition(agentId);
                    newBoxPos = findTargetBoxPosition(newState, boxType, current.targetBoxPos);
                }

                // Space-Time collision check: is agent's next position reserved?
                if (reservations.isReserved(newAgentPos, nextTime, agentId)) {
                    continue;
                }
                
                // Also check box position if pushing
                if (newBoxPos != null && !newBoxPos.equals(current.targetBoxPos)) {
                    if (reservations.isReserved(newBoxPos, nextTime, agentId)) {
                        continue;
                    }
                }

                StateKeyWithTime newKey = new StateKeyWithTime(newState, agentId, newBoxPos, nextTime);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                int newH = (newBoxPos != null) ? getDistance(newBoxPos, goalPos, level) : 0;
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newBoxPos, nextTime);
                openList.add(newNode);
            }
        }

        return null;
    }

    /**
     * A* search to move an agent to its goal position (no box involved).
     */
    public List<Action> searchForAgentGoal(int agentId, Position goalPos,
            State initialState, Level level) {
        Position startPos = initialState.getAgentPosition(agentId);
        if (startPos.equals(goalPos)) {
            return Collections.emptyList();
        }

        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<Position, Integer> bestG = new HashMap<>();

        int h = getDistance(startPos, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, null);
        openList.add(startNode);
        bestG.put(startPos, 0);

        int exploredCount = 0;

        while (!openList.isEmpty() && exploredCount < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            SearchNode current = openList.poll();
            exploredCount++;

            Position currentAgentPos = current.state.getAgentPosition(agentId);
            if (currentAgentPos.equals(goalPos)) {
                return reconstructPath(current);
            }

            for (Direction dir : Direction.values()) {
                Action action = Action.move(dir);

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);
                Position newAgentPos = newState.getAgentPosition(agentId);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newAgentPos);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newAgentPos, newG);

                int newH = getDistance(newAgentPos, goalPos, level);
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, null);
                openList.add(newNode);
            }
        }

        return null;
    }

    /**
     * Search for a path to displace a box to a temporary position.
     */
    public List<Action> searchForDisplacement(int agentId, Position boxStart, Position targetPos,
            char boxType, State initialState, Level level, long timeLimitMs) {
        
        long startTime = System.currentTimeMillis();
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        int h = getDistance(boxStart, targetPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart);
        StateKey startKey = new StateKey(initialState, agentId, boxStart);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;
        int maxExplore = 2000;

        while (!openList.isEmpty() && exploredCount < maxExplore) {
            if (exploredCount % 100 == 0 && System.currentTimeMillis() - startTime > timeLimitMs) {
                return null;
            }
            
            SearchNode current = openList.poll();
            exploredCount++;

            Character boxAtTarget = current.state.getBoxes().get(targetPos);
            if (boxAtTarget != null && boxAtTarget == boxType) {
                return reconstructPath(current);
            }

            for (Action action : PlanningUtils.getAllActions()) {
                if (action.type == Action.ActionType.NOOP) continue;

                if (!current.state.isApplicable(action, agentId, level)) continue;

                State newState = current.state.apply(action, agentId);

                Position newBoxPos = current.targetBoxPos;
                if (action.type == Action.ActionType.PUSH || action.type == Action.ActionType.PULL) {
                    Position agentPos = current.state.getAgentPosition(agentId);
                    Position oldBoxPos;
                    if (action.type == Action.ActionType.PUSH) {
                        oldBoxPos = agentPos.move(action.agentDir);
                        newBoxPos = oldBoxPos.move(action.boxDir);
                    } else {
                        oldBoxPos = agentPos.move(action.boxDir.opposite());
                        newBoxPos = agentPos;
                    }
                    
                    Character movedBox = current.state.getBoxes().get(oldBoxPos);
                    if (movedBox == null || movedBox != boxType) {
                        newBoxPos = current.targetBoxPos;
                    }
                }

                StateKey newKey = new StateKey(newState, agentId, newBoxPos);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) continue;

                bestG.put(newKey, newG);

                int newH = getDistance(newBoxPos, targetPos, level);
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newBoxPos);
                openList.add(newNode);
            }
        }

        return null;
    }

    /**
     * Plans a path to displace a blocking box to a temporary parking position.
     */
    public List<Action> planBoxDisplacement(int agentId, Position boxPos, Position targetPos,
            char boxType, State initialState, Level level) {
        
        Character actualBox = initialState.getBoxAt(boxPos);
        if (actualBox == null || actualBox != boxType) {
            return null;
        }

        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        // Less strict goal protection
        Set<Position> frozenGoals = new HashSet<>();
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    Position goalPosition = new Position(row, col);
                    char currentBox = initialState.getBoxAt(goalPosition);
                    if (currentBox == goalType) {
                        frozenGoals.add(goalPosition);
                    }
                }
            }
        }
        
        frozenGoals.remove(targetPos);
        frozenGoals.remove(boxPos);

        int h = getDistance(boxPos, targetPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxPos);
        StateKey startKey = new StateKey(initialState, agentId, boxPos);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;
        int maxStates = SearchConfig.MAX_STATES_PER_SUBGOAL / 2;

        while (!openList.isEmpty() && exploredCount < maxStates) {
            SearchNode current = openList.poll();
            exploredCount++;

            Position currentBoxPos = findBoxPosition(current.state, boxType, current.targetBoxPos);
            if (currentBoxPos != null && currentBoxPos.equals(targetPos)) {
                return reconstructPath(current);
            }

            for (Action action : PlanningUtils.getAllActions()) {
                if (action.type == Action.ActionType.NOOP) {
                    continue;
                }

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                if (wouldDisturbSatisfiedGoal(action, agentId, current.state, frozenGoals)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);

                Position newTargetBoxPos = findTargetBoxPosition(newState, boxType, current.targetBoxPos);
                StateKey newKey = new StateKey(newState, agentId, newTargetBoxPos);
                int newG = current.g + 1;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                int newH = (newTargetBoxPos != null) ? getDistance(newTargetBoxPos, targetPos, level) : 0;
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newTargetBoxPos);
                openList.add(newNode);
            }
        }

        return null;
    }

    /**
     * Computes heuristic for subgoal.
     */
    public int computeSubgoalHeuristic(State state, Position goalPos, char boxType, Level level) {
        Character atGoal = state.getBoxes().get(goalPos);
        if (atGoal != null && atGoal == boxType) {
            return 0;
        }

        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                int dist = getDistance(entry.getKey(), goalPos, level);
                minDist = Math.min(minDist, dist);
            }
        }

        return minDist == Integer.MAX_VALUE ? 0 : minDist;
    }

    // ========== Private Helper Methods ==========

    private int getDistance(Position from, Position to, Level level) {
        if (heuristic instanceof TrueDistanceHeuristic) {
            int dist = ((TrueDistanceHeuristic) heuristic).getDistance(from, to);
            if (dist < Integer.MAX_VALUE) {
                return dist;
            }
        }
        return from.manhattanDistance(to);
    }

    private List<Action> reconstructPath(SearchNode goalNode) {
        List<Action> path = new ArrayList<>();
        SearchNode current = goalNode;

        while (current.parent != null) {
            path.add(current.action);
            current = current.parent;
        }

        Collections.reverse(path);
        return path;
    }

    private Position findTargetBoxPosition(State state, char boxType, Position lastKnownPos) {
        if (lastKnownPos != null) {
            Character boxAtLast = state.getBoxes().get(lastKnownPos);
            if (boxAtLast != null && boxAtLast == boxType) {
                return lastKnownPos;
            }
        }

        for (Direction dir : Direction.values()) {
            Position newPos = lastKnownPos.move(dir);
            Character boxAtNew = state.getBoxes().get(newPos);
            if (boxAtNew != null && boxAtNew == boxType) {
                return newPos;
            }
        }

        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                return entry.getKey();
            }
        }

        return null;
    }

    private Position findBoxPosition(State state, char boxType, Position hint) {
        if (hint != null) {
            Character boxAtHint = state.getBoxAt(hint);
            if (boxAtHint != null && boxAtHint == boxType) {
                return hint;
            }
        }
        
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                return entry.getKey();
            }
        }
        return null;
    }

    private boolean wouldDisturbSatisfiedGoal(Action action, int agentId, State state, Set<Position> satisfiedGoals) {
        if (satisfiedGoals.isEmpty()) {
            return false;
        }

        Position agentPos = state.getAgentPosition(agentId);

        if (action.type == Action.ActionType.PUSH) {
            Position boxPos = agentPos.move(action.agentDir);
            return satisfiedGoals.contains(boxPos);
        }

        if (action.type == Action.ActionType.PULL) {
            Position boxPos = agentPos.move(action.boxDir.opposite());
            return satisfiedGoals.contains(boxPos);
        }

        return false;
    }

    // ========== Inner Classes ==========

    private static class StateKey {
        final Position agentPos;
        final Position targetBoxPos;

        StateKey(State state, int agentId, Position targetBoxPos) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof StateKey)) return false;
            StateKey other = (StateKey) obj;
            return agentPos.equals(other.agentPos) &&
                    Objects.equals(targetBoxPos, other.targetBoxPos);
        }

        @Override
        public int hashCode() {
            return Objects.hash(agentPos, targetBoxPos);
        }
    }
    
    /** State key with time dimension for space-time A*. */
    private static class StateKeyWithTime {
        final Position agentPos;
        final Position targetBoxPos;
        final int time;

        StateKeyWithTime(State state, int agentId, Position targetBoxPos, int time) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
            this.time = time;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof StateKeyWithTime)) return false;
            StateKeyWithTime other = (StateKeyWithTime) obj;
            return time == other.time && agentPos.equals(other.agentPos) &&
                    Objects.equals(targetBoxPos, other.targetBoxPos);
        }

        @Override
        public int hashCode() {
            return Objects.hash(agentPos, targetBoxPos, time);
        }
    }

    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action action;
        final int g;
        final int f;
        final Position targetBoxPos;
        final int time;

        SearchNode(State state, SearchNode parent, Action action, int g, int h, Position targetBoxPos) {
            this(state, parent, action, g, h, targetBoxPos, g); // time = g for backward compat
        }
        
        SearchNode(State state, SearchNode parent, Action action, int g, int h, Position targetBoxPos, int time) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.g = g;
            this.f = g + h;
            this.targetBoxPos = targetBoxPos;
            this.time = time;
        }

        @Override
        public int compareTo(SearchNode other) {
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0) return fCompare;
            return Integer.compare(other.g, this.g);
        }
    }
}
