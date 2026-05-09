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

    // Diagnostic counters (used by PriorityPlanningStrategy.printDiagSummary).
    private int callCount = 0;
    private int conflictingCallCount = 0;

    public int getCallCount() { return callCount; }
    public int getConflictingCallCount() { return conflictingCallCount; }
    public void resetCounts() { callCount = 0; conflictingCallCount = 0; }
    
    /**
     * Resolves conflicts in a joint action by making conflicting agents wait.
     */
    public Action[] resolveConflicts(Action[] jointAction, State state, Level level, int primaryAgentId) {
        callCount++;
        List<ConflictDetector.Conflict> conflicts = conflictDetector.detectConflicts(state, jointAction, level);
        
        if (conflicts.isEmpty()) {
            return jointAction;
        }
        conflictingCallCount++;
        
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
     * Priority: agent executing PUSH/PULL > others; agent already on its own
     * agent-goal is downgraded. Tie falls back to higher agent ID yields.
     */
    public Action[] resolveConflicts(Action[] jointAction, State state, Level level) {
        callCount++;
        List<ConflictDetector.Conflict> conflicts =
                conflictDetector.detectConflicts(state, jointAction, level);
        if (conflicts.isEmpty()) return jointAction;
        conflictingCallCount++;

        Action[] resolved = jointAction.clone();
        for (ConflictDetector.Conflict conflict : conflicts) {
            int loser = pickYielder(conflict.agent1, conflict.agent2,
                                    jointAction, state, level);
            resolved[loser] = Action.noOp();
        }
        return resolved;
    }

    /** Lower returned priority => loser (yields). Tie -> higher agent ID yields. */
    private int pickYielder(int a1, int a2, Action[] joint, State state, Level level) {
        int p1 = priorityOf(a1, joint, state, level);
        int p2 = priorityOf(a2, joint, state, level);
        if (p1 != p2) return (p1 < p2) ? a1 : a2;
        return Math.max(a1, a2);
    }

    private int priorityOf(int agentId, Action[] joint, State state, Level level) {
        int p = 0;
        Action a = (agentId < joint.length) ? joint[agentId] : null;
        if (a != null && (a.type == Action.ActionType.PUSH
                       || a.type == Action.ActionType.PULL)) p += 10;  // holding a box
        Position pos = state.getAgentPosition(agentId);
        if (pos != null && level.getAgentGoal(pos.row, pos.col) == agentId) p -= 5; // already home
        return p;
    }
    
    /**
     * Applies a joint action to a state.
     * Per CLAUDE.md: actions are simultaneous — cell occupancy evaluated at START.
     */
    public State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        return state.applyJointAction(jointAction, level);
    }
}
