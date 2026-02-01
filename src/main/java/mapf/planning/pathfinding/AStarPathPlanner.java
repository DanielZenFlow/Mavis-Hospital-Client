package mapf.planning.pathfinding;

import mapf.domain.*;

import java.util.*;

/**
 * A* pathfinder for single-agent path planning.
 * 
 * Clean implementation following ARCHITECTURE.md guidelines:
 * - Single responsibility: Only does A* search
 * - No goal protection logic (that's handled by the caller)
 * - No yielding/coordination (that's handled by coordinator)
 */
public class AStarPathPlanner implements PathPlanner {
    
    /** Maximum states to explore before giving up */
    private static final int MAX_STATES = 10000;
    
    /** All possible actions */
    private static final List<Action> ALL_ACTIONS = createAllActions();
    
    private static List<Action> createAllActions() {
        List<Action> actions = new ArrayList<>();
        actions.add(Action.noOp());
        for (Direction dir : Direction.values()) {
            actions.add(Action.move(dir));
            for (Direction boxDir : Direction.values()) {
                actions.add(Action.push(dir, boxDir));
                actions.add(Action.pull(dir, boxDir));
            }
        }
        return Collections.unmodifiableList(actions);
    }
    
    // Optional: positions that should be avoided (frozen boxes, etc.)
    private Set<Position> frozenPositions = Collections.emptySet();
    
    /**
     * Sets positions that cannot have boxes moved from them.
     */
    public void setFrozenPositions(Set<Position> frozen) {
        this.frozenPositions = frozen != null ? frozen : Collections.emptySet();
    }
    
    @Override
    public PathResult findAgentPath(State state, Level level, int agentId, Position target) {
        Position start = state.getAgentPosition(agentId);
        if (start == null) return PathResult.failure();
        if (start.equals(target)) return PathResult.success(Collections.emptyList());
        
        PriorityQueue<SearchNode> open = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();
        
        int h = start.manhattanDistance(target);
        SearchNode startNode = new SearchNode(state, null, null, 0, h, null);
        open.add(startNode);
        bestG.put(new StateKey(state, agentId, null), 0);
        
        int explored = 0;
        
        while (!open.isEmpty() && explored < MAX_STATES) {
            SearchNode current = open.poll();
            explored++;
            
            // Goal check: agent at target
            Position agentPos = current.state.getAgentPosition(agentId);
            if (agentPos.equals(target)) {
                return PathResult.success(reconstructPath(current));
            }
            
            // Expand
            for (Action action : ALL_ACTIONS) {
                if (action.type == Action.ActionType.NOOP) continue;
                if (!current.state.isApplicable(action, agentId, level)) continue;
                
                // Skip actions that would move frozen boxes
                if (wouldMoveFrozenBox(action, agentId, current.state)) continue;
                
                State newState = current.state.apply(action, agentId);
                StateKey key = new StateKey(newState, agentId, null);
                int newG = current.g + 1;
                
                Integer existingG = bestG.get(key);
                if (existingG != null && existingG <= newG) continue;
                
                bestG.put(key, newG);
                
                Position newAgentPos = newState.getAgentPosition(agentId);
                int newH = newAgentPos.manhattanDistance(target);
                open.add(new SearchNode(newState, current, action, newG, newH, null));
            }
        }
        
        return PathResult.failure();
    }
    
    @Override
    public PathResult findBoxPath(State state, Level level, int agentId, Position boxPos, Position boxTarget) {
        char boxType = state.getBoxAt(boxPos);
        if (boxType == '\0') return PathResult.failure();
        if (boxPos.equals(boxTarget)) return PathResult.success(Collections.emptyList());
        
        // Agent must be able to push this box
        if (!level.canAgentMoveBox(agentId, boxType)) return PathResult.failure();
        
        PriorityQueue<SearchNode> open = new PriorityQueue<>();
        Map<StateKey, Integer> bestG = new HashMap<>();
        
        int h = boxPos.manhattanDistance(boxTarget);
        SearchNode startNode = new SearchNode(state, null, null, 0, h, boxPos);
        open.add(startNode);
        bestG.put(new StateKey(state, agentId, boxPos), 0);
        
        int explored = 0;
        
        while (!open.isEmpty() && explored < MAX_STATES) {
            SearchNode current = open.poll();
            explored++;
            
            // Goal check: box at target
            char boxAtTarget = current.state.getBoxAt(boxTarget);
            if (boxAtTarget == boxType) {
                return PathResult.success(reconstructPath(current));
            }
            
            // Expand
            for (Action action : ALL_ACTIONS) {
                if (action.type == Action.ActionType.NOOP) continue;
                if (!current.state.isApplicable(action, agentId, level)) continue;
                
                // Skip actions that would move frozen boxes (except our target box)
                if (wouldMoveFrozenBox(action, agentId, current.state, current.boxPos)) continue;
                
                State newState = current.state.apply(action, agentId);
                
                // Track box position
                Position newBoxPos = trackBoxPosition(action, agentId, current.state, current.boxPos, boxType);
                
                StateKey key = new StateKey(newState, agentId, newBoxPos);
                int newG = current.g + 1;
                
                Integer existingG = bestG.get(key);
                if (existingG != null && existingG <= newG) continue;
                
                bestG.put(key, newG);
                
                int newH = newBoxPos.manhattanDistance(boxTarget);
                open.add(new SearchNode(newState, current, action, newG, newH, newBoxPos));
            }
        }
        
        return PathResult.failure();
    }
    
    // ============ Internal Helpers ============
    
    private boolean wouldMoveFrozenBox(Action action, int agentId, State state) {
        return wouldMoveFrozenBox(action, agentId, state, null);
    }
    
    private boolean wouldMoveFrozenBox(Action action, int agentId, State state, Position exceptBox) {
        if (frozenPositions.isEmpty()) return false;
        if (action.type != Action.ActionType.PUSH && action.type != Action.ActionType.PULL) return false;
        
        Position agentPos = state.getAgentPosition(agentId);
        Position boxPos;
        
        if (action.type == Action.ActionType.PUSH) {
            boxPos = agentPos.move(action.agentDir);
        } else { // PULL
            boxPos = agentPos.move(action.boxDir.opposite());
        }
        
        // Don't freeze the box we're explicitly moving
        if (exceptBox != null && boxPos.equals(exceptBox)) return false;
        
        return frozenPositions.contains(boxPos);
    }
    
    private Position trackBoxPosition(Action action, int agentId, State state, Position currentBoxPos, char boxType) {
        if (action.type != Action.ActionType.PUSH && action.type != Action.ActionType.PULL) {
            return currentBoxPos;
        }
        
        Position agentPos = state.getAgentPosition(agentId);
        Position movedFrom, movedTo;
        
        if (action.type == Action.ActionType.PUSH) {
            movedFrom = agentPos.move(action.agentDir);
            movedTo = movedFrom.move(action.boxDir);
        } else { // PULL
            movedFrom = agentPos.move(action.boxDir.opposite());
            movedTo = agentPos; // Agent's old position
        }
        
        // Only update if we moved our target box
        if (movedFrom.equals(currentBoxPos)) {
            return movedTo;
        }
        return currentBoxPos;
    }
    
    private List<Action> reconstructPath(SearchNode node) {
        List<Action> path = new ArrayList<>();
        while (node.parent != null) {
            path.add(node.action);
            node = node.parent;
        }
        Collections.reverse(path);
        return path;
    }
    
    // ============ Internal Classes ============
    
    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action action;
        final int g;
        final int f;
        final Position boxPos; // For box goals
        
        SearchNode(State state, SearchNode parent, Action action, int g, int h, Position boxPos) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.g = g;
            this.f = g + h;
            this.boxPos = boxPos;
        }
        
        @Override
        public int compareTo(SearchNode other) {
            int fCmp = Integer.compare(this.f, other.f);
            if (fCmp != 0) return fCmp;
            return Integer.compare(other.g, this.g); // Prefer deeper
        }
    }
    
    /**
     * State key for duplicate detection.
     * Uses full state for accurate duplicate detection.
     * For box goals, also tracks target box position for efficiency.
     */
    private static class StateKey {
        private final State state;
        private final Position boxPos;
        private final int hash;
        
        StateKey(State state, int agentId, Position boxPos) {
            this.state = state;
            this.boxPos = boxPos;
            // Include full state hash for correctness
            this.hash = Objects.hash(state, boxPos);
        }
        
        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof StateKey)) return false;
            StateKey other = (StateKey) obj;
            return state.equals(other.state) && Objects.equals(boxPos, other.boxPos);
        }
        
        @Override
        public int hashCode() {
            return hash;
        }
    }
}
