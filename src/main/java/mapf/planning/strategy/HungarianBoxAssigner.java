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

                // Skip physically stuck boxes (no free neighbor at all).
                // In Pull-Sokoban, a box needs at least 1 free neighbor to be pulled.
                // Including stuck boxes wastes assignment slots and produces
                // unrealistic cost estimates (BFS ignores dynamic obstacles).
                if (!isBoxMovable(boxPos, state, level)) continue;

                availableBoxes.add(boxPos);
            }

            // 3. If fewer boxes than goals, we can't do full assignment
            //    Hungarian requires rows ≤ columns
            if (availableBoxes.isEmpty()) continue;

            // 4. Collect same-color agent positions for agent→box distance
            Color boxColor = level.getBoxColor(boxType);
            List<Position> sameColorAgentPositions = new ArrayList<>();
            for (int a = 0; a < state.getNumAgents(); a++) {
                if (boxColor != null && boxColor.equals(level.getAgentColor(a))) {
                    sameColorAgentPositions.add(state.getAgentPosition(a));
                }
            }

            // 5. Compute assignment with full transport cost (agent→box + box→goal)
            AssignmentResult result = computeAssignment(
                    unsatisfiedGoals, availableBoxes, sameColorAgentPositions,
                    state, level, immovableDetector);
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
     * @param agentPositions   positions of same-color agents (for agent→box cost)
     * @param state            current state
     * @param level            level definition
     * @param immovableDetector for distance calculation
     * @return assignment result, or null if assignment is impossible
     */
    static AssignmentResult computeAssignment(
            List<Position> goals, List<Position> boxes, List<Position> agentPositions,
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

        // Precompute minimum agent→box distance for each box.
        // This makes the cost reflect TRUE transport cost (agent→box + box→goal)
        // instead of just box→goal, which could select a box close to goal but
        // far from all agents. Falls back to 0 if no agents found (shouldn't happen).
        int[] minAgentToBox = new int[boxes.size()];
        for (int b = 0; b < boxes.size(); b++) {
            if (agentPositions.isEmpty()) {
                minAgentToBox[b] = 0;
            } else {
                int minDist = Integer.MAX_VALUE;
                for (Position agentPos : agentPositions) {
                    int d = immovableDetector.getDistanceWithImmovableBoxes(
                            agentPos, boxes.get(b), state, level);
                    if (d < minDist) minDist = d;
                }
                minAgentToBox[b] = minDist;
            }
        }

        // Build cost matrix: cost = agent→box + box→goal
        int[][] cost = new int[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                Position boxPos, goalPos;
                int boxIdx;
                if (transposed) {
                    boxPos = boxes.get(i);    // row = box
                    goalPos = goals.get(j);   // col = goal
                    boxIdx = i;
                } else {
                    boxPos = boxes.get(j);    // col = box
                    goalPos = goals.get(i);   // row = goal
                    boxIdx = j;
                }
                int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(
                        boxPos, goalPos, state, level);
                int agentToBox = minAgentToBox[boxIdx];
                // Combine: if either is unreachable, mark as impossible
                if (boxToGoal == Integer.MAX_VALUE || agentToBox == Integer.MAX_VALUE) {
                    cost[i][j] = Integer.MAX_VALUE;
                } else {
                    cost[i][j] = agentToBox + boxToGoal;
                }
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

    /**
     * Checks if a box can be pushed or pulled in at least one direction.
     * 
     * Push: agent at neighbor, box moves in any direction except toward the agent.
     *   Push(agentDir, boxDir): agent at boxPos.move(dir) pushes box. Box can move
     *   in ANY of the 3 directions that aren't back toward the agent (lateral push
     *   is valid in this domain: agentDir ≠ boxDir is allowed).
     * 
     * Pull: agent at neighbor, agent retreats in ANY free direction, box follows.
     *   Pull requires: agent stands adjacent, retreats to any free cell (not just
     *   further from box). The agent can retreat laterally or in any direction
     *   where the destination cell is free (excluding boxPos, which still has the
     *   box when checking applicability in pre-move state).
     * 
     * Returns false if the box is completely stuck (no push or pull possible).
     * 
     * Package-private: shared by SubgoalManager to avoid duplication (DRY).
     */
    static boolean isBoxMovable(Position boxPos, State state, Level level) {
        for (Direction dir : Direction.values()) {
            Position neighbor = boxPos.move(dir);
            if (level.isWall(neighbor) || state.hasBoxAt(neighbor)) continue;
            // neighbor is free — agent could stand here
            
            // Push check: box can move in any direction except dir (where agent stands).
            // In Push(agentDir=dir.opposite(), boxDir), boxDir can differ from agentDir.
            // The box destination must be free (not wall, not box). The agent's pre-move
            // position (neighbor) is blocked by hasAgentAt in isApplicable, so boxDir ≠ dir.
            for (Direction boxMoveDir : Direction.values()) {
                if (boxMoveDir == dir) continue; // can't push box onto agent's position
                Position pushTarget = boxPos.move(boxMoveDir);
                if (!level.isWall(pushTarget) && !state.hasBoxAt(pushTarget)) {
                    return true; // Push feasible
                }
            }
            
            // Pull check: agent at neighbor retreats in ANY direction where the
            // destination is free. boxDir identifies the box (relative to agent),
            // and agentDir (retreat direction) is independent.
            // Exclude retreat to boxPos: box is still there in pre-move state,
            // so isApplicable would reject it (hasBoxAt check on newAgentPos? No —
            // isFree checks newAgentPos, but boxPos has a box).
            // Actually: retreat to boxPos means agentRetreat = boxPos. Since
            // neighbor.move(dir.opposite()) = boxPos, retreatDir = dir.opposite().
            // boxPos has a box → state.hasBoxAt(agentRetreat) = true → correctly rejected.
            for (Direction retreatDir : Direction.values()) {
                Position agentRetreat = neighbor.move(retreatDir);
                if (!level.isWall(agentRetreat) && !state.hasBoxAt(agentRetreat)) {
                    return true; // Pull feasible: agent retreats in retreatDir
                }
            }
        }
        return false;
    }

    private static void logVerbose(String msg) {
        if (SearchConfig.isVerbose()) {
            System.err.println(msg);
        }
    }
}
