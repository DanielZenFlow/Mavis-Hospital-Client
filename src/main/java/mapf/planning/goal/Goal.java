package mapf.planning.goal;

import mapf.domain.Position;

/**
 * Represents a goal to be achieved in MAPF.
 * 
 * Two types of goals:
 * 1. Box Goal: Move a box of a specific type to a position
 * 2. Agent Goal: Move an agent to a position
 * 
 * Design follows ARCHITECTURE.md: simple, immutable data class.
 */
public final class Goal {
    
    public enum Type {
        BOX_GOAL,    // Move a box to this position
        AGENT_GOAL   // Move an agent to this position
    }
    
    public final Type type;
    public final int agentId;       // Agent responsible for this goal
    public final Position position; // Target position
    public final char boxType;      // Box type (only for BOX_GOAL, '\0' for AGENT_GOAL)
    
    private Goal(Type type, int agentId, Position position, char boxType) {
        this.type = type;
        this.agentId = agentId;
        this.position = position;
        this.boxType = boxType;
    }
    
    /**
     * Creates a box goal: Agent must push box of given type to position.
     */
    public static Goal boxGoal(int agentId, char boxType, Position position) {
        return new Goal(Type.BOX_GOAL, agentId, position, boxType);
    }
    
    /**
     * Creates an agent goal: Agent must move to position.
     */
    public static Goal agentGoal(int agentId, Position position) {
        return new Goal(Type.AGENT_GOAL, agentId, position, '\0');
    }
    
    public boolean isBoxGoal() {
        return type == Type.BOX_GOAL;
    }
    
    public boolean isAgentGoal() {
        return type == Type.AGENT_GOAL;
    }
    
    @Override
    public String toString() {
        if (isBoxGoal()) {
            return "BoxGoal[Agent " + agentId + " -> Box " + boxType + " to " + position + "]";
        } else {
            return "AgentGoal[Agent " + agentId + " -> " + position + "]";
        }
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Goal)) return false;
        Goal other = (Goal) obj;
        return type == other.type && 
               agentId == other.agentId && 
               position.equals(other.position) && 
               boxType == other.boxType;
    }
    
    @Override
    public int hashCode() {
        int result = type.hashCode();
        result = 31 * result + agentId;
        result = 31 * result + position.hashCode();
        result = 31 * result + boxType;
        return result;
    }
}
