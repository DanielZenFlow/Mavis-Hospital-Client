package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;

import java.util.*;

/**
 * Computes globally optimal box-to-goal assignments per color group using the Hungarian algorithm.
 * 
 * <p>For each box type (character), this class builds a cost matrix where:
 * <ul>
 *   <li>Rows = unsatisfied goal positions of that type</li>
 *   <li>Columns = available (movable, reachable) boxes of that type</li>
 *   <li>Cost[i][j] = BFS distance from box_j to goal_i (through immovable-box-aware pathfinding)</li>
 * </ul>
 * 
 * <p>The Hungarian algorithm then finds the assignment that minimizes total transport distance.
 * This replaces the per-goal greedy selection in {@link SubgoalManager#findBestBoxForGoal}
 * for the initial assignment, while keeping the greedy + feasibility-check as a fallback.
 * 
 * <p>Design: Stateless utility methods. The cache lives in {@link SubgoalManager}.
 * 
 * @see HungarianAlgorithm
 * @see SubgoalManager
 */
public final class HungarianBoxAssigner {

    private HungarianBoxAssigner() {} // Utility class

    /**
     * Result of a Hungarian assignment for one box type.
     * Maps each goal position to its optimally assigned box position.
     */
    public static class AssignmentResult {
        /** goal → assigned box (globally optimal). Null value means no box could be assigned. */
        private final Map<Position, Position> goalToBox;
        /** Total BFS distance of the optimal assignment. */
        private final long totalCost;

        AssignmentResult(Map<Position, Position> goalToBox, long totalCost) {
            this.goalToBox = Collections.unmodifiableMap(goalToBox);
            this.totalCost = totalCost;
        }

        public Position getAssignedBox(Position goalPos) {
            return goalToBox.get(goalPos);
        }

        public Map<Position, Position> getGoalToBoxMap() {
            return goalToBox;
        }

        public long getTotalCost() {
            return totalCost;
        }
    }

    /**
     * Computes optimal box-to-goal assignments for ALL box types in the current state.
     *
     * @param state             current world state
     * @param level             level definition
     * @param completedBoxGoals goals already satisfied (frozen)
     * @param immovableDetector for BFS distance computation through immovable-box-aware pathfinding
     * @return map from box type character to its AssignmentResult
     */
    public static Map<Character, AssignmentResult> computeAllAssignments(
            State state, Level level, Set<Position> completedBoxGoals,
            ImmovableBoxDetector immovableDetector) {

        Map<Character, AssignmentResult> results = new HashMap<>();
        Set<Position> immovableBoxes = immovableDetector.getImmovableBoxes(state, level);

        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char boxType = entry.getKey();
            List<Position> allGoals = entry.getValue();

            // 1. Collect unsatisfied goals
            List<Position> unsatisfiedGoals = new ArrayList<>();
            for (Position goalPos : allGoals) {
                if (completedBoxGoals.contains(goalPos)) continue;
                Character boxAtGoal = state.getBoxes().get(goalPos);
                if (boxAtGoal == null || boxAtGoal != boxType) {
                    unsatisfiedGoals.add(goalPos);
                }
            }
            if (unsatisfiedGoals.isEmpty()) continue;

            // 2. Collect available boxes of this type
            List<Position> availableBoxes = new ArrayList<>();
            for (Map.Entry<Position, Character> boxEntry : state.getBoxes().entrySet()) {
                if (boxEntry.getValue() != boxType) continue;
                Position boxPos = boxEntry.getKey();

                // Skip boxes already at their correct goal (self-satisfied)
                if (level.getBoxGoal(boxPos) == boxType) continue;

                // Skip immovable boxes
                if (immovableBoxes.contains(boxPos)) continue;

                availableBoxes.add(boxPos);
            }

            // 3. If fewer boxes than goals, we can't do full assignment
            //    Hungarian requires rows ≤ columns
            if (availableBoxes.isEmpty()) continue;

            // 4. Compute assignment
            AssignmentResult result = computeAssignment(
                    unsatisfiedGoals, availableBoxes, state, level, immovableDetector);
            if (result != null) {
                results.put(boxType, result);
            }
        }

        return results;
    }

    /**
     * Computes optimal assignment for a single box type group.
     *
     * @param goals            unsatisfied goal positions
     * @param boxes            available box positions
     * @param state            current state
     * @param level            level definition
     * @param immovableDetector for distance calculation
     * @return assignment result, or null if assignment is impossible
     */
    static AssignmentResult computeAssignment(
            List<Position> goals, List<Position> boxes,
            State state, Level level, ImmovableBoxDetector immovableDetector) {

        int nGoals = goals.size();
        int nBoxes = boxes.size();

        // Ensure rows (goals) ≤ columns (boxes) for Hungarian
        // If more goals than boxes, we handle the transpose
        boolean transposed = nGoals > nBoxes;
        int rows, cols;
        if (transposed) {
            // More goals than boxes — some goals will be unassigned
            // We transpose: rows=boxes, cols=goals, then invert result
            rows = nBoxes;
            cols = nGoals;
        } else {
            rows = nGoals;
            cols = nBoxes;
        }

        // Build cost matrix
        int[][] cost = new int[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                Position from, to;
                if (transposed) {
                    from = boxes.get(i);   // row = box
                    to = goals.get(j);     // col = goal
                } else {
                    from = boxes.get(j);   // col = box
                    to = goals.get(i);     // row = goal
                }
                // BFS distance (box → goal), treating immovable boxes as walls
                cost[i][j] = immovableDetector.getDistanceWithImmovableBoxes(from, to, state, level);
            }
        }

        // Solve
        int[] assignment;
        try {
            assignment = HungarianAlgorithm.solve(cost);
        } catch (IllegalArgumentException e) {
            logVerbose("[Hungarian] Failed to solve: " + e.getMessage());
            return null;
        }

        // Build result map
        Map<Position, Position> goalToBox = new HashMap<>();
        long totalCost = 0;

        if (transposed) {
            // assignment[i] = goal index assigned to box i
            for (int i = 0; i < assignment.length; i++) {
                int goalIdx = assignment[i];
                if (goalIdx >= 0 && goalIdx < nGoals) {
                    Position boxPos = boxes.get(i);
                    Position goalPos = goals.get(goalIdx);
                    goalToBox.put(goalPos, boxPos);
                    totalCost += cost[i][goalIdx];
                }
            }
        } else {
            // assignment[i] = box index assigned to goal i
            for (int i = 0; i < assignment.length; i++) {
                int boxIdx = assignment[i];
                if (boxIdx >= 0 && boxIdx < nBoxes) {
                    Position goalPos = goals.get(i);
                    Position boxPos = boxes.get(boxIdx);
                    goalToBox.put(goalPos, boxPos);
                    totalCost += cost[i][boxIdx];
                }
            }
        }

        if (goalToBox.isEmpty()) return null;

        logVerbose("[Hungarian] Assigned " + goalToBox.size() + "/" + goals.size()
                + " goals for type, total cost=" + totalCost);

        return new AssignmentResult(goalToBox, totalCost);
    }

    private static void logVerbose(String msg) {
        if (SearchConfig.isVerbose()) {
            System.err.println(msg);
        }
    }
}
