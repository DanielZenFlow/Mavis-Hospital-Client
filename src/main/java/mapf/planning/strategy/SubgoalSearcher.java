package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.heuristic.Heuristic;

import java.util.*;

/**
 * Single-agent A* pathfinding for subgoal execution.
 * Searches for a path for one agent to move a box to a goal or reach an agent goal.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class SubgoalSearcher {
    
    private final Heuristic heuristic;
    private final ImmovableBoxDetector immovableDetector;
    
    public SubgoalSearcher(Heuristic heuristic) {
        this.heuristic = heuristic;
        this.immovableDetector = new ImmovableBoxDetector();
    }
    
    /**
     * Searches for a plan to achieve a box subgoal (move box to goal).
     */
    public List<Action> searchForBoxSubgoal(int agentId, Position boxStart, Position goalPos,
                                           State initialState, Level level, Set<Position> satisfiedGoals,
                                           long startTime, long timeoutMs) {
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<StateKey, SearchNode> explored = new HashMap<>();
        
        StateKey startKey = new StateKey(initialState, agentId, boxStart);
        int h = computeSubgoalHeuristic(initialState, goalPos, initialState.getBoxes().get(boxStart), level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, boxStart);
        
        openList.add(startNode);
        explored.put(startKey, startNode);
        
        int statesExplored = 0;
        
        while (!openList.isEmpty() && statesExplored < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                return null; // Timeout
            }
            
            SearchNode current = openList.poll();
            Position currentBoxPos = current.targetBoxPos;
            
            // Goal check: box at goal position
            if (currentBoxPos.equals(goalPos)) {
                return reconstructPath(current);
            }
            
            statesExplored++;
            
            // Expand successors
            for (Map.Entry<Action, State> successor : current.state.getSuccessors(agentId, level)) {
                Action action = successor.getKey();
                State newState = successor.getValue();
                
                // Skip actions that disturb satisfied goals
                if (wouldDisturbSatisfiedGoal(action, agentId, current.state, satisfiedGoals)) {
                    continue;
                }
                
                // Track box position
                Position newBoxPos = computeNewBoxPosition(action, currentBoxPos, current.state, agentId);
                
                StateKey newKey = new StateKey(newState, agentId, newBoxPos);
                
                int newG = current.g + 1;
                int newH = computeSubgoalHeuristic(newState, goalPos, newState.getBoxes().get(newBoxPos), level);
                
                SearchNode existingNode = explored.get(newKey);
                if (existingNode != null && existingNode.g <= newG) {
                    continue;
                }
                
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, newBoxPos);
                openList.add(newNode);
                explored.put(newKey, newNode);
            }
        }
        
        return null; // No solution
    }
    
    /**
     * Searches for a plan to achieve an agent goal (agent reaches position).
     */
    public List<Action> searchForAgentGoal(int agentId, Position goalPos,
                                          State initialState, Level level, Set<Position> satisfiedGoals,
                                          long startTime, long timeoutMs) {
        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Set<State> explored = new HashSet<>();
        
        int h = immovableDetector.getDistanceWithImmovableBoxes(
            initialState.getAgentPosition(agentId), goalPos, initialState, level);
        SearchNode startNode = new SearchNode(initialState, null, null, 0, h, null);
        
        openList.add(startNode);
        explored.add(initialState);
        
        int statesExplored = 0;
        
        while (!openList.isEmpty() && statesExplored < SearchConfig.MAX_STATES_PER_SUBGOAL) {
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                return null;
            }
            
            SearchNode current = openList.poll();
            Position agentPos = current.state.getAgentPosition(agentId);
            
            if (agentPos.equals(goalPos)) {
                return reconstructPath(current);
            }
            
            statesExplored++;
            
            for (Map.Entry<Action, State> successor : current.state.getSuccessors(agentId, level)) {
                Action action = successor.getKey();
                State newState = successor.getValue();
                
                if (explored.contains(newState)) continue;
                
                if (wouldDisturbSatisfiedGoal(action, agentId, current.state, satisfiedGoals)) {
                    continue;
                }
                
                explored.add(newState);
                
                Position newAgentPos = newState.getAgentPosition(agentId);
                int newG = current.g + 1;
                int newH = immovableDetector.getDistanceWithImmovableBoxes(newAgentPos, goalPos, newState, level);
                
                SearchNode newNode = new SearchNode(newState, current, action, newG, newH, null);
                openList.add(newNode);
            }
        }
        
        return null;
    }
    
    private Position computeNewBoxPosition(Action action, Position currentBoxPos, State state, int agentId) {
        if (action.type == Action.ActionType.PUSH) {
            return currentBoxPos.move(action.boxDir);
        } else if (action.type == Action.ActionType.PULL) {
            return state.getAgentPosition(agentId);
        }
        return currentBoxPos;
    }
    
    private int computeSubgoalHeuristic(State state, Position goalPos, Character boxType, Level level) {
        if (boxType == null) return 0;
        
        // Find box position
        Position boxPos = null;
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) {
                boxPos = entry.getKey();
                break;
            }
        }
        
        if (boxPos == null) return 0;
        return immovableDetector.getDistanceWithImmovableBoxes(boxPos, goalPos, state, level);
    }
    
    private boolean wouldDisturbSatisfiedGoal(Action action, int agentId, State state, Set<Position> satisfiedGoals) {
        if (satisfiedGoals == null || satisfiedGoals.isEmpty()) return false;
        
        Position agentPos = state.getAgentPosition(agentId);
        
        if (action.type == Action.ActionType.PUSH) {
            Position boxPos = agentPos.move(action.agentDir);
            Position newBoxPos = boxPos.move(action.boxDir);
            return satisfiedGoals.contains(boxPos) || satisfiedGoals.contains(newBoxPos);
        } else if (action.type == Action.ActionType.PULL) {
            Position boxPos = agentPos.move(action.boxDir.opposite());
            return satisfiedGoals.contains(boxPos) || satisfiedGoals.contains(agentPos);
        }
        
        return false;
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
    
    // Inner classes
    
    private static class StateKey {
        final Position agentPos;
        final Position targetBoxPos;
        
        StateKey(State state, int agentId, Position targetBoxPos) {
            this.agentPos = state.getAgentPosition(agentId);
            this.targetBoxPos = targetBoxPos;
        }
        
        @Override
        public boolean equals(Object obj) {
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
    
    private static class SearchNode implements Comparable<SearchNode> {
        final State state;
        final SearchNode parent;
        final Action action;
        final int g;
        final int f;
        final Position targetBoxPos;
        
        SearchNode(State state, SearchNode parent, Action action, int g, int h, Position targetBoxPos) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.g = g;
            this.f = g + h;
            this.targetBoxPos = targetBoxPos;
        }
        
        @Override
        public int compareTo(SearchNode other) {
            int fCompare = Integer.compare(this.f, other.f);
            if (fCompare != 0) return fCompare;
            return Integer.compare(other.g, this.g);
        }
    }
}
