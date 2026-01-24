package mapf.planning;

import mapf.domain.Level;
import mapf.domain.State;

/**
 * Interface for heuristic functions used in search algorithms.
 * 
 * A heuristic provides an estimate of the cost to reach the goal from a given state.
 * For A* search to be optimal, the heuristic must be admissible (never overestimate
 * the true cost) and ideally consistent (satisfies the triangle inequality).
 */
public interface Heuristic {
    
    /**
     * Estimates the cost to reach the goal from the given state.
     * 
     * @param state the current state
     * @param level the level (provides goal information)
     * @return estimated cost to reach goal (lower bound)
     */
    int estimate(State state, Level level);
}
