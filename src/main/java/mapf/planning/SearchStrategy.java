package mapf.planning;

import mapf.domain.*;
import java.util.List;

/**
 * Strategy interface for search algorithms.
 * Implements the Strategy Pattern as recommended in ARCHITECTURE.md.
 * 
 * Different implementations can be swapped based on level characteristics:
 * - SingleAgentAStar: For single agent levels
 * - JointAStar: For 2-3 agents (optimal but exponential)
 * - PriorityPlanning: For 4+ agents (suboptimal but scalable)
 */
public interface SearchStrategy {
    
    /**
     * Searches for a plan from the initial state to the goal state.
     * 
     * @param initialState the starting state
     * @param level the level information
     * @return list of joint actions (each Action[] has one action per agent), or null if no solution
     */
    List<Action[]> search(State initialState, Level level);
    
    /**
     * @return the name of this strategy (for logging)
     */
    String getName();
    
    /**
     * Sets the maximum time allowed for search in milliseconds.
     * 
     * @param timeoutMs maximum search time
     */
    void setTimeout(long timeoutMs);
    
    /**
     * Sets the maximum number of states to explore.
     * 
     * @param maxStates maximum states
     */
    void setMaxStates(int maxStates);
}
