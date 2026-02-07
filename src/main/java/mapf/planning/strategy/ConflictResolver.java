package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.coordination.ConflictDetector;
import java.util.*;

/**
 * Resolves conflicts between agent actions.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class ConflictResolver {
    
    private final ConflictDetector conflictDetector = new ConflictDetector();
    
    /**
     * Resolves conflicts in a joint action by making conflicting agents wait.
     */
    public Action[] resolveConflicts(Action[] jointAction, State state, Level level, int primaryAgentId) {
        List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(state, jointAction, level);
        
        if (conflicts.isEmpty()) {
            return jointAction;
        }
        
        // Make non-primary agents wait
        Action[] resolved = jointAction.clone();
        for (ConflictDetector.Conflict conflict : conflicts) {
            if (conflict.agent1 != primaryAgentId) {
                resolved[conflict.agent1] = Action.noOp();
            }
            if (conflict.agent2 != primaryAgentId) {
                resolved[conflict.agent2] = Action.noOp();
            }
        }
        
        return resolved;
    }
    
    /**
     * Resolves conflicts by making lower priority agents wait.
     */
    public Action[] resolveConflicts(Action[] jointAction, State state, Level level) {
        List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(state, jointAction, level);
        
        if (conflicts.isEmpty()) {
            return jointAction;
        }
        
        // Make higher-numbered agents wait (lower priority)
        Action[] resolved = jointAction.clone();
        for (ConflictDetector.Conflict conflict : conflicts) {
            int higherAgent = Math.max(conflict.agent1, conflict.agent2);
            resolved[higherAgent] = Action.noOp();
        }
        
        return resolved;
    }
    
    /**
     * Applies a joint action to a state.
     * Per CLAUDE.md: actions are simultaneous — cell occupancy evaluated at START.
     */
    public State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        return state.applyJointAction(jointAction, level);
    }
}
