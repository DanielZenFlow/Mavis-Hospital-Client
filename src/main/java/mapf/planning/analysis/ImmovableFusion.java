package mapf.planning.analysis;

import mapf.domain.*;
import java.util.*;

/**
 * Fuses immovable boxes into the wall set, producing a smaller equivalent problem.
 *
 * <p>Motivation: TaskFilter identifies boxes that no agent can ever push (no matching color
 * agent exists, or the box is unreachable). Downstream search treats them as "obstacles"
 * but still pays for them: TrueDistanceHeuristic precomputes BFS over them as boxes,
 * SubgoalManager iterates them, BSP successor generation copies them. Converting them
 * to walls (and dropping them from the State's box map) shrinks the state space and
 * speeds up every BFS / A* used afterwards. Particularly impactful on pixel-art /
 * border-heavy levels (pacMAn, donkeyK, EpicfAIl, Lily, Minchia, PokeNOM, GroupEZ).
 *
 * <p>Safety: a box at position {@code P} is fusible iff
 * <ul>
 *   <li>{@code P} has no box-goal in the original Level, OR</li>
 *   <li>The goal at {@code P} requires the same letter the immovable box already has
 *       (so this immovable box "satisfies" that goal trivially).</li>
 * </ul>
 * Boxes on a wrong-typed goal are left alone (level is unsolvable anyway, and we
 * must not silently mark its goal as satisfied).
 */
public final class ImmovableFusion {

    private ImmovableFusion() { }

    /** Result of fusion: new (Level, State) plus diagnostic info. */
    public static final class FusedProblem {
        public final Level level;
        public final State state;
        public final Set<Position> fusedPositions;
        public final boolean changed;

        FusedProblem(Level level, State state, Set<Position> fusedPositions, boolean changed) {
            this.level = level;
            this.state = state;
            this.fusedPositions = Collections.unmodifiableSet(fusedPositions);
            this.changed = changed;
        }
    }

    /**
     * Produce a fused (Level, State) by treating the given immovable boxes as walls.
     * Returns the originals untouched if no boxes are fusible.
     */
    public static FusedProblem fuse(Level original, State state, Set<Position> immovableBoxes) {
        if (immovableBoxes == null || immovableBoxes.isEmpty()) {
            return new FusedProblem(original, state, Collections.emptySet(), false);
        }

        Set<Position> fusible = new HashSet<>();
        for (Position p : immovableBoxes) {
            Character box = state.getBoxAt(p);
            if (box == null) continue; // stale entry, skip
            char goalType = original.getBoxGoal(p);
            if (goalType == '\0' || goalType == box) {
                fusible.add(p);
            }
            // Otherwise: immovable box on a wrong-letter goal → unsolvable.
            // Leave it as-is so the regular planner reports failure naturally.
        }

        if (fusible.isEmpty()) {
            return new FusedProblem(original, state, Collections.emptySet(), false);
        }

        int rows = original.getRows();
        int cols = original.getCols();
        boolean[][] newWalls = new boolean[rows][cols];
        char[][] newBoxGoals = new char[rows][cols];
        int[][] newAgentGoals = new int[rows][cols];

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                newWalls[r][c] = original.isWall(r, c);
                newBoxGoals[r][c] = original.getBoxGoal(r, c);
                newAgentGoals[r][c] = original.getAgentGoal(r, c);
            }
        }
        for (Position p : fusible) {
            newWalls[p.row][p.col] = true;
            newBoxGoals[p.row][p.col] = '\0'; // satisfied-by-wall
        }

        Level newLevel = new Level(
                original.getName(), rows, cols,
                newWalls, newBoxGoals, newAgentGoals,
                original.getBoxColors(), original.getAgentColors());

        Map<Position, Character> newBoxes = new HashMap<>(state.getBoxes());
        for (Position p : fusible) {
            newBoxes.remove(p);
        }
        Position[] agentPositions = new Position[state.getNumAgents()];
        for (int i = 0; i < agentPositions.length; i++) {
            agentPositions[i] = state.getAgentPosition(i);
        }
        State newState = new State(agentPositions, newBoxes);

        return new FusedProblem(newLevel, newState, fusible, true);
    }
}
