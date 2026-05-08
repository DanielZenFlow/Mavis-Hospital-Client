package mapf.planning.strategy;

import mapf.domain.*;

import java.util.*;

/**
 * P2: Iterated Width(1) single-agent planner.
 *
 * Per qanda.txt 3 / 75 and claudeopus47.txt 1.2 layer L2: IW(1) is the canonical
 * solver for "escape" / "relocation" subgoals \u2014 single-agent, no good heuristic
 * required, single-variable state space. It explores BFS in atom-novelty order:
 * a state is novel iff at least one atom (variable, value) has not been seen
 * in any previously expanded state. With novelty=1, the search is polynomial in
 * the number of atoms, which is ideal for sparse single-object reach problems.
 *
 * Atoms encoded:
 *   - kind 0: agent position             (one atom per visited cell)
 *   - kind 1: target-box position        (one atom per visited cell, for the box being moved)
 *
 * This planner is intentionally narrow:
 *   - Single agent (caller specifies agentId).
 *   - Single target box (caller specifies boxStart + boxType).
 *   - Goal: target box reaches goalPos (any letter match works).
 *   - No reservation table, no other-agent coordination (escape is local).
 *
 * Used as Round 0 in PriorityPlanningStrategy.planSubgoal() for subgoals marked
 * as "escape" via setEscapeSubgoals(); falls through to A*-based BoxSearchPlanner
 * if IW(1) fails or exhausts its budget.
 */
public final class IW1Planner {

    private final int maxStates;

    public IW1Planner(int maxStates) {
        this.maxStates = Math.max(100, maxStates);
    }

    /**
     * BFS with novelty=1 pruning. Returns single-agent action list, or null on failure.
     */
    public List<Action> search(int agentId, Position boxStart, Position goalPos, char boxType,
                               State initialState, Level level, Set<Position> frozenGoals) {
        if (boxStart == null || goalPos == null) return null;

        // Quick exit: already at goal.
        Character boxAtGoal = initialState.getBoxes().get(goalPos);
        if (boxAtGoal != null && boxAtGoal == boxType) {
            return Collections.emptyList();
        }

        int cols = level.getCols();
        Set<Long> seenAtoms = new HashSet<>();

        // Seed atoms with initial state.
        Position agentStart = initialState.getAgentPosition(agentId);
        seenAtoms.add(encodeAtom(0, agentStart, cols));
        seenAtoms.add(encodeAtom(1, boxStart, cols));

        Deque<Node> queue = new ArrayDeque<>();
        queue.add(new Node(initialState, null, null, boxStart));

        int explored = 0;

        while (!queue.isEmpty() && explored < maxStates) {
            Node current = queue.poll();
            explored++;

            // Goal check.
            Character at = current.state.getBoxes().get(goalPos);
            if (at != null && at == boxType) {
                return reconstructPath(current);
            }

            for (Action action : PlanningUtils.getAllActions()) {
                if (action.type == Action.ActionType.NOOP) continue;
                if (!current.state.isApplicable(action, agentId, level)) continue;

                // Frozen-goal protection: do not push/pull boxes already on satisfied goals.
                if (!frozenGoals.isEmpty()
                        && wouldDisturbFrozenGoal(action, agentId, current.state, frozenGoals)) {
                    continue;
                }

                State newState = current.state.apply(action, agentId);
                Position newBoxPos = computeNewBoxPosition(action, agentId, current.state, current.targetBoxPos);
                Position newAgentPos = newState.getAgentPosition(agentId);

                // Novelty test: at least one atom must be new.
                long agentAtom = encodeAtom(0, newAgentPos, cols);
                long boxAtom = (newBoxPos != null) ? encodeAtom(1, newBoxPos, cols) : -1L;
                boolean agentNovel = !seenAtoms.contains(agentAtom);
                boolean boxNovel = (boxAtom != -1L) && !seenAtoms.contains(boxAtom);
                if (!agentNovel && !boxNovel) continue;

                // Mark atoms seen, enqueue.
                if (agentNovel) seenAtoms.add(agentAtom);
                if (boxNovel) seenAtoms.add(boxAtom);
                queue.add(new Node(newState, current, action, newBoxPos));
            }
        }
        return null;
    }

    private static long encodeAtom(int kind, Position p, int cols) {
        // kind in upper bits, then row*cols+col.
        long cell = (long) p.row * cols + p.col;
        return ((long) kind << 32) | cell;
    }

    private boolean wouldDisturbFrozenGoal(Action action, int agentId, State state, Set<Position> frozen) {
        Position agentPos = state.getAgentPosition(agentId);
        switch (action.type) {
            case PUSH: {
                Position boxOldPos = agentPos.move(action.agentDir);
                return frozen.contains(boxOldPos);
            }
            case PULL: {
                Position boxOldPos = agentPos.move(action.boxDir.opposite());
                return frozen.contains(boxOldPos);
            }
            default:
                return false;
        }
    }

    private Position computeNewBoxPosition(Action action, int agentId,
                                            State stateBefore, Position currentBoxPos) {
        if (currentBoxPos == null) return null;
        Position agentPos = stateBefore.getAgentPosition(agentId);
        switch (action.type) {
            case PUSH: {
                Position boxOldPos = agentPos.move(action.agentDir);
                if (boxOldPos.equals(currentBoxPos)) {
                    return currentBoxPos.move(action.boxDir);
                }
                return currentBoxPos;
            }
            case PULL: {
                Position boxOldPos = agentPos.move(action.boxDir.opposite());
                if (boxOldPos.equals(currentBoxPos)) {
                    return agentPos;
                }
                return currentBoxPos;
            }
            default:
                return currentBoxPos;
        }
    }

    private List<Action> reconstructPath(Node goal) {
        List<Action> path = new ArrayList<>();
        Node cur = goal;
        while (cur.parent != null) {
            path.add(cur.action);
            cur = cur.parent;
        }
        Collections.reverse(path);
        return path;
    }

    private static final class Node {
        final State state;
        final Node parent;
        final Action action;
        final Position targetBoxPos;

        Node(State state, Node parent, Action action, Position targetBoxPos) {
            this.state = state;
            this.parent = parent;
            this.action = action;
            this.targetBoxPos = targetBoxPos;
        }
    }
}
