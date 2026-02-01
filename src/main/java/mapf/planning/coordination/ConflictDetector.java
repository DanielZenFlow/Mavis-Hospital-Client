package mapf.planning.coordination;

import mapf.domain.*;

import java.util.*;

/**
 * Detects conflicts between multiple agents when executing actions simultaneously.
 * 
 * Conflict types:
 * 1. Vertex conflict: Two agents want to occupy the same cell
 * 2. Edge conflict: Two agents want to swap positions (cross each other)
 * 3. Box conflict: Two agents want to move the same box
 * 
 * This class is used by the multi-agent planner to identify and resolve conflicts.
 */
public class ConflictDetector {
    
    /**
     * Represents a conflict between agents.
     */
    public static class Conflict {
        /** Type of conflict */
        public final ConflictType type;
        
        /** First agent involved */
        public final int agent1;
        
        /** Second agent involved */
        public final int agent2;
        
        /** Position(s) involved in the conflict */
        public final Position position;
        
        /** Optional second position (for edge conflicts) */
        public final Position position2;
        
        public Conflict(ConflictType type, int agent1, int agent2, Position position) {
            this(type, agent1, agent2, position, null);
        }
        
        public Conflict(ConflictType type, int agent1, int agent2, Position position, Position position2) {
            this.type = type;
            this.agent1 = agent1;
            this.agent2 = agent2;
            this.position = position;
            this.position2 = position2;
        }
        
        @Override
        public String toString() {
            return String.format("Conflict{%s, agents=[%d,%d], pos=%s}", 
                                type, agent1, agent2, position);
        }
    }
    
    /**
     * Types of conflicts.
     */
    public enum ConflictType {
        /** Two agents want to occupy the same cell */
        VERTEX,
        
        /** Two agents want to swap positions */
        EDGE,
        
        /** Two agents want to move the same box */
        BOX
    }
    
    /**
     * Detects all conflicts that would occur if the given actions were executed.
     * 
     * @param state the current state
     * @param actions actions for each agent (indexed by agent ID)
     * @param level the level
     * @return list of all detected conflicts
     */
    public List<Conflict> detectConflicts(State state, Action[] actions, Level level) {
        List<Conflict> conflicts = new ArrayList<>();
        
        // Calculate resulting positions for each agent
        Position[] currentPositions = new Position[actions.length];
        Position[] newPositions = new Position[actions.length];
        Position[] boxFromPositions = new Position[actions.length]; // Where box was
        Position[] boxToPositions = new Position[actions.length];   // Where box goes
        
        for (int i = 0; i < actions.length; i++) {
            Position agentPos = state.getAgentPosition(i);
            currentPositions[i] = agentPos;
            
            if (agentPos == null || actions[i] == null) {
                newPositions[i] = agentPos;
                continue;
            }
            
            Action action = actions[i];
            
            switch (action.type) {
                case NOOP:
                    newPositions[i] = agentPos;
                    break;
                    
                case MOVE:
                    newPositions[i] = agentPos.move(action.agentDir);
                    break;
                    
                case PUSH:
                    newPositions[i] = agentPos.move(action.agentDir);
                    boxFromPositions[i] = newPositions[i]; // Box was where agent is going
                    boxToPositions[i] = boxFromPositions[i].move(action.boxDir);
                    break;
                    
                case PULL:
                    newPositions[i] = agentPos.move(action.agentDir);
                    boxFromPositions[i] = agentPos.move(action.boxDir.opposite());
                    boxToPositions[i] = agentPos; // Box moves to agent's old position
                    break;
            }
        }
        
        // Check for vertex conflicts (two agents in same cell)
        for (int i = 0; i < actions.length; i++) {
            for (int j = i + 1; j < actions.length; j++) {
                if (newPositions[i] != null && newPositions[i].equals(newPositions[j])) {
                    conflicts.add(new Conflict(ConflictType.VERTEX, i, j, newPositions[i]));
                }
            }
        }
        
        // Check for edge conflicts (agents swapping positions)
        for (int i = 0; i < actions.length; i++) {
            for (int j = i + 1; j < actions.length; j++) {
                if (currentPositions[i] != null && currentPositions[j] != null &&
                    newPositions[i] != null && newPositions[j] != null) {
                    
                    if (currentPositions[i].equals(newPositions[j]) && 
                        currentPositions[j].equals(newPositions[i])) {
                        conflicts.add(new Conflict(ConflictType.EDGE, i, j, 
                                                  currentPositions[i], currentPositions[j]));
                    }
                }
            }
        }
        
        // Check for box conflicts (two agents moving same box)
        for (int i = 0; i < actions.length; i++) {
            for (int j = i + 1; j < actions.length; j++) {
                if (boxFromPositions[i] != null && boxFromPositions[j] != null &&
                    boxFromPositions[i].equals(boxFromPositions[j])) {
                    conflicts.add(new Conflict(ConflictType.BOX, i, j, boxFromPositions[i]));
                }
            }
        }
        
        // Check for agent-box conflicts (agent moves into box destination)
        for (int i = 0; i < actions.length; i++) {
            for (int j = 0; j < actions.length; j++) {
                if (i == j) continue;
                
                if (newPositions[i] != null && boxToPositions[j] != null &&
                    newPositions[i].equals(boxToPositions[j])) {
                    conflicts.add(new Conflict(ConflictType.VERTEX, i, j, newPositions[i]));
                }
            }
        }
        
        return conflicts;
    }
    
    /**
     * Checks if a set of actions can be executed without conflicts.
     * 
     * @param state the current state
     * @param actions actions for each agent
     * @param level the level
     * @return true if no conflicts exist
     */
    public boolean hasNoConflicts(State state, Action[] actions, Level level) {
        return detectConflicts(state, actions, level).isEmpty();
    }
    
    /**
     * Finds the first conflict (useful for conflict-based search).
     * 
     * @param state the current state
     * @param actions actions for each agent
     * @param level the level
     * @return the first conflict found, or null if none
     */
    public Conflict findFirstConflict(State state, Action[] actions, Level level) {
        List<Conflict> conflicts = detectConflicts(state, actions, level);
        return conflicts.isEmpty() ? null : conflicts.get(0);
    }
}
