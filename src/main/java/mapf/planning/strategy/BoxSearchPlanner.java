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
     * Smart frozen mechanism:
     * - hardFrozenGoals: goals with pending dependents, MUST NOT be disturbed
     * - softFrozenGoals: goals with no pending dependents, can be displaced with penalty
     * 
     * 3-tier fallback:
     * 1. Hard-block hard, penalty soft
     * 2. Penalty both (all frozen become soft)
     */
    public List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, 
            Set<Position> hardFrozenGoals, Set<Position> softFrozenGoals) {
        
        // Tier 1: hard-block hardFrozen, penalty softFrozen
        Set<Position> allFrozen = new HashSet<>(hardFrozenGoals);
        allFrozen.addAll(softFrozenGoals);
        List<Action> result = searchForSubgoalInternal(agentId, boxStart, goalPos, boxType, 
                                                       initialState, level, hardFrozenGoals, softFrozenGoals, true);
        if (result != null) return result;
        
        // Tier 2: penalty-only for ALL frozen goals (hard becomes soft)
        if (!allFrozen.isEmpty()) {
            System.err.println("[BSP] Tier 1 failed, retrying with all-soft-freeze for " + boxType + " -> " + goalPos);
            result = searchForSubgoalInternal(agentId, boxStart, goalPos, boxType,
                                              initialState, level, Collections.emptySet(), allFrozen, false);
        }
        return result;
    }
    
    /** Legacy overload: treats all frozen as hard. */
    public List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, Set<Position> frozenGoals) {
        return searchForSubgoal(agentId, boxStart, goalPos, boxType, initialState, level, 
                               frozenGoals, Collections.emptySet());
    }
    
    private List<Action> searchForSubgoalInternal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, 
            Set<Position> hardFrozenGoals, Set<Position> softFrozenGoals, boolean hardFreeze) {
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();

        int h = computeBoxSubgoalHeuristic(initialState, agentId, boxStart, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart);
        StateKey startKey = new StateKey(initialState, agentId, boxStart, boxType);
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

                // Goal protection: hard-frozen goals are blocked, soft-frozen get penalty
                if (hardFreeze && wouldDisturbSatisfiedGoal(action, agentId, current.state, hardFrozenGoals)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);

                Position newTargetBoxPos = findTargetBoxPosition(newState, boxType, current.targetBoxPos);
                StateKey newKey = new StateKey(newState, agentId, newTargetBoxPos, boxType);
                int penalty = disturbancePenalty(action, agentId, current.state, softFrozenGoals);
                int newG = current.g + 1 + penalty;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                int newH = (newTargetBoxPos != null) ? computeBoxSubgoalHeuristic(newState, agentId, newTargetBoxPos, goalPos, level) : 0;
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newTargetBoxPos);
                openList.add(newNode);
            }
        }

        System.err.println("[BSP-DEBUG] 2D-A* FAILED (" + (hardFreeze ? "tier1" : "tier2") + "): box=" + boxType + " " + boxStart + " -> " + goalPos 
            + " explored=" + exploredCount + "/" + SearchConfig.MAX_STATES_PER_SUBGOAL 
            + " openListRemaining=" + openList.size() + " hardFrozen=" + hardFrozenGoals.size() + " softFrozen=" + softFrozenGoals.size());
        return null;
    }

    /**
     * Space-Time A* search for subgoal with reservation table.
     * Avoids positions reserved by higher-priority agents.
     * Uses smart frozen: hard-frozen goals are blocked, soft-frozen get penalty.
     */
    public List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, 
            Set<Position> hardFrozenGoals, Set<Position> softFrozenGoals,
            ReservationTable reservations, int startTime) {
        
        if (reservations == null) {
            return searchForSubgoal(agentId, boxStart, goalPos, boxType, initialState, level, 
                                   hardFrozenGoals, softFrozenGoals);
        }
        
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKeyWithTime, Integer> bestG = new HashMap<>();

        int h = computeBoxSubgoalHeuristic(initialState, agentId, boxStart, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart, startTime);
        StateKeyWithTime startKey = new StateKeyWithTime(initialState, agentId, boxStart, startTime, boxType);
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
                    // Hard-frozen: block completely
                    if (wouldDisturbSatisfiedGoal(action, agentId, current.state, hardFrozenGoals)) {
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

                StateKeyWithTime newKey = new StateKeyWithTime(newState, agentId, newBoxPos, nextTime, boxType);
                // Soft-frozen: penalty-based
                int penalty = disturbancePenalty(action, agentId, current.state, softFrozenGoals);
                int newG = current.g + 1 + penalty;

                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

                int newH = (newBoxPos != null) ? computeBoxSubgoalHeuristic(newState, agentId, newBoxPos, goalPos, level) : 0;
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newBoxPos, nextTime);
                openList.add(newNode);
            }
        }

        System.err.println("[BSP-DEBUG] ST-A* FAILED: box=" + boxType + " " + boxStart + " -> " + goalPos 
            + " explored=" + exploredCount + "/" + SearchConfig.MAX_STATES_PER_SUBGOAL 
            + " openListRemaining=" + openList.size() + " hardFrozen=" + hardFrozenGoals.size() + " softFrozen=" + softFrozenGoals.size());
        return null;
    }

    /** Legacy ST-A* overload: treats all frozen as hard. */
    public List<Action> searchForSubgoal(int agentId, Position boxStart, Position goalPos,
            char boxType, State initialState, Level level, Set<Position> frozenGoals,
            ReservationTable reservations, int startTime) {
        return searchForSubgoal(agentId, boxStart, goalPos, boxType, initialState, level,
                               frozenGoals, Collections.emptySet(), reservations, startTime);
    }

    /**
     * A* search to move an agent to its goal position.
     * Uses all action types (Move/Push/Pull) so the agent can push/pull
     * same-color boxes out of the way if they block the path.
     */
    public List<Action> searchForAgentGoal(int agentId, Position goalPos,
            State initialState, Level level) {
        Position startPos = initialState.getAgentPosition(agentId);
        if (startPos.equals(goalPos)) {
            return Collections.emptyList();
        }

        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        // Use StateKey tracking agent position + all box positions for full-state dedup
        Map<StateKey, Integer> bestG = new HashMap<>();

        int h = getDistance(startPos, goalPos, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, null);
        // Use a key that captures agent pos (boxes may change if pushed)
        StateKey startKey = new AgentGoalStateKey(initialState, agentId);
        openList.add(startNode);
        bestG.put(startKey, 0);

        int exploredCount = 0;

        while (!openList.isEmpty() && exploredCount < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            SearchNode current = openList.poll();
            exploredCount++;

            Position currentAgentPos = current.state.getAgentPosition(agentId);
            if (currentAgentPos.equals(goalPos)) {
                return reconstructPath(current);
            }

            // Try all actions: Move, Push, Pull (agent can push/pull same-color boxes blocking path)
            for (Action action : PlanningUtils.getAllActions()) {
                if (action.type == Action.ActionType.NOOP) {
                    continue;
                }

                if (!current.state.isApplicable(action, agentId, level)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);
                Position newAgentPos = newState.getAgentPosition(agentId);
                int newG = current.g + 1;

                StateKey newKey = new AgentGoalStateKey(newState, agentId);
                Integer existingG = bestG.get(newKey);
                if (existingG != null && existingG <= newG) {
                    continue;
                }

                bestG.put(newKey, newG);

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
        StateKey startKey = new StateKey(initialState, agentId, boxStart, boxType);
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

                StateKey newKey = new StateKey(newState, agentId, newBoxPos, boxType);
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
        StateKey startKey = new StateKey(initialState, agentId, boxPos, boxType);
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
                StateKey newKey = new StateKey(newState, agentId, newTargetBoxPos, boxType);
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

    /**
     * Computes heuristic for box subgoal search.
     * Includes both box-to-goal distance AND agent-to-box distance.
     * 
     * For Push: agent must be adjacent to box (from push side) -> min 1 step to reach box
     * For Pull: agent must reach box side to pull -> same adjacency requirement
     * 
     * h = boxToGoal + max(0, agentToBox - 1)
     * The -1 accounts for the fact that when agent is adjacent, it's already in position.
     */
    private int computeBoxSubgoalHeuristic(State state, int agentId, Position boxPos, Position goalPos, Level level) {
        int boxToGoal = getDistance(boxPos, goalPos, level);
        Position agentPos = state.getAgentPosition(agentId);
        
        // Check if agent is already adjacent to the box
        if (agentPos.manhattanDistance(boxPos) <= 1) {
            return boxToGoal;
        }
        
        // Agent needs to reach the box first
        int agentToBox = getDistance(agentPos, boxPos, level);
        if (agentToBox >= Integer.MAX_VALUE) {
            return boxToGoal; // Fallback if no path found
        }
        
        // Agent-to-box contribution (subtract 1 since agent needs to be adjacent, not on box)
        return boxToGoal + Math.max(0, agentToBox - 1);
    }

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

        // Check adjacent positions (only if we have a last known position)
        if (lastKnownPos != null) {
            for (Direction dir : Direction.values()) {
                Position newPos = lastKnownPos.move(dir);
                Character boxAtNew = state.getBoxes().get(newPos);
                if (boxAtNew != null && boxAtNew == boxType) {
                    return newPos;
                }
            }
        }

        // Fallback: scan all boxes
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
    
    /**
     * Returns a cost penalty for disturbing satisfied goals.
     * Instead of hard-blocking, returns 0 if no disturbance, or a high penalty if disturbing.
     * This allows the search to disturb goals as a last resort.
     */
    private int disturbancePenalty(Action action, int agentId, State state, Set<Position> satisfiedGoals) {
        if (satisfiedGoals.isEmpty()) {
            return 0;
        }

        Position agentPos = state.getAgentPosition(agentId);

        if (action.type == Action.ActionType.PUSH) {
            Position boxPos = agentPos.move(action.agentDir);
            if (satisfiedGoals.contains(boxPos)) return 50;
        }

        if (action.type == Action.ActionType.PULL) {
            Position boxPos = agentPos.move(action.boxDir.opposite());
            if (satisfiedGoals.contains(boxPos)) return 50;
        }

        return 0;
    }

    // ========== Inner Classes ==========

    private static class StateKey {
        final Position agentPos;
        final Position targetBoxPos;
        /** Hash of ALL same-type box positions. When agent pushes/pulls other boxes
         *  of the same type to make way, the layout change must be captured. */
        final int sameTypeBoxHash;

        StateKey(State state, int agentId, Position targetBoxPos) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
            this.sameTypeBoxHash = 0; // legacy: no box type tracking
        }
        
        StateKey(State state, int agentId, Position targetBoxPos, char boxType) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
            // Hash all positions of boxes with the same type
            // This ensures different layouts of same-color boxes are distinguished
            int hash = 0;
            for (Map.Entry<Position, Character> e : state.getBoxes().entrySet()) {
                if (e.getValue() == boxType) {
                    hash ^= e.getKey().hashCode() * 31;
                }
            }
            this.sameTypeBoxHash = hash;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof StateKey)) return false;
            StateKey other = (StateKey) obj;
            return agentPos.equals(other.agentPos) &&
                    Objects.equals(targetBoxPos, other.targetBoxPos) &&
                    sameTypeBoxHash == other.sameTypeBoxHash;
        }

        @Override
        public int hashCode() {
            return Objects.hash(agentPos, targetBoxPos, sameTypeBoxHash);
        }
    }
    
    /** State key with time dimension for space-time A*. */
    private static class StateKeyWithTime {
        final Position agentPos;
        final Position targetBoxPos;
        final int time;
        final int sameTypeBoxHash;

        StateKeyWithTime(State state, int agentId, Position targetBoxPos, int time) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
            this.time = time;
            this.sameTypeBoxHash = 0;
        }
        
        StateKeyWithTime(State state, int agentId, Position targetBoxPos, int time, char boxType) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
            this.time = time;
            int hash = 0;
            for (Map.Entry<Position, Character> e : state.getBoxes().entrySet()) {
                if (e.getValue() == boxType) {
                    hash ^= e.getKey().hashCode() * 31;
                }
            }
            this.sameTypeBoxHash = hash;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof StateKeyWithTime)) return false;
            StateKeyWithTime other = (StateKeyWithTime) obj;
            return time == other.time && agentPos.equals(other.agentPos) &&
                    Objects.equals(targetBoxPos, other.targetBoxPos) &&
                    sameTypeBoxHash == other.sameTypeBoxHash;
        }

        @Override
        public int hashCode() {
            return Objects.hash(agentPos, targetBoxPos, time, sameTypeBoxHash);
        }
    }
    
    /** State key for agent goal search: tracks agent position + all box positions
     *  since Push/Pull changes box layout. */
    private static class AgentGoalStateKey extends StateKey {
        private final int boxHash;
        
        AgentGoalStateKey(State state, int agentId) {
            super(state, agentId, null);
            this.boxHash = state.getBoxes().hashCode();
        }
        
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof AgentGoalStateKey)) return false;
            AgentGoalStateKey other = (AgentGoalStateKey) obj;
            return agentPos.equals(other.agentPos) && boxHash == other.boxHash;
        }
        
        @Override
        public int hashCode() {
            return Objects.hash(agentPos, boxHash);
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
