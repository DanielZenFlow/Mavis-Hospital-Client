package mapf.planning.analysis;

import mapf.domain.Direction;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.util.*;

/**
 * Step 1 of the Layer-3 fix (ref: claudeopus47.txt §1.2.3, §1.3.2).
 *
 * PROBLEM: When NAMO displaces a box from position P (which happens to be a box-goal),
 * revalidateCompletedGoals removes P from completedBoxGoals, and PP immediately
 * re-plans the box back to P — creating an oscillation loop.
 *
 * ROOT CAUSE: The planner treats "box X sits at goal G" as both a *world predicate*
 * and a *subgoal*.  When NAMO uses G as a transit corridor, the world predicate
 * becomes false, which is immediately misread as "subgoal is undone → redo it".
 *
 * THIS CLASS (read-only, diagnostic only for now):
 *   Classifies every box-goal as one of:
 *     TERMINAL    – in a dead-end; filling it never blocks any other delivery.
 *     NEUTRAL     – low transit pressure; safe to treat normally.
 *     TRANSIT     – lies on ≥2 other boxes' best delivery paths; must be filled last
 *                   within its group and must not be re-instated during NAMO windows.
 *     CHOKEPOINT  – lies on ≥CHOKE_THRESHOLD paths AND has ≤2 free neighbours
 *                   (structural bottleneck like ISO's row-14 KL corridor).
 *
 * HOW IT WORKS:
 *   For every box-goal G_i, run a BFS from each other box B_j to its goal G_j,
 *   ignoring boxes (treat them as passable) to get the "ideal corridor" path.
 *   Count how many such paths pass through the *position* of G_i.
 *   crossingCount[G_i] = number of (B_j → G_j) corridors that contain G_i's cell.
 *   If crossingCount ≥ TRANSIT_THRESHOLD → TRANSIT.
 *   If crossingCount ≥ CHOKE_THRESHOLD AND degree ≤ 2 → CHOKEPOINT.
 *
 * CURRENT USE: Purely diagnostic.  Called from LevelAnalyzer.analyze() and prints
 * [TRANSIT] log lines so we can validate the classification against BOX-OSC output.
 * No decision path is altered in this step.
 */
public class GoalTransitAnalyzer {

    public enum TransitProfile {
        TERMINAL,    // safe to fill early
        NEUTRAL,     // normal
        TRANSIT,     // sits on other deliveries' corridor
        CHOKEPOINT   // structural bottleneck + high crossing count
    }

    /** A goal position together with its transit classification and crossing count. */
    public static class GoalProfile {
        public final Position goalPos;
        public final char goalType;
        public final TransitProfile profile;
        /** How many other (box → goal) BFS corridors pass through this goal's cell. */
        public final int crossingCount;
        /** Free (non-wall) neighbours of this goal cell. */
        public final int degree;
        /** Which other goals' delivery corridors cross here (for detailed diagnostics). */
        public final List<Position> crossingGoals;

        GoalProfile(Position goalPos, char goalType,
                    TransitProfile profile, int crossingCount, int degree,
                    List<Position> crossingGoals) {
            this.goalPos       = goalPos;
            this.goalType      = goalType;
            this.profile       = profile;
            this.crossingCount = crossingCount;
            this.degree        = degree;
            this.crossingGoals = Collections.unmodifiableList(crossingGoals);
        }
    }

    // ── Thresholds ──────────────────────────────────────────────────────────
    /** ≥ this many crossing corridors → TRANSIT */
    private static final int TRANSIT_THRESHOLD = 2;
    /** ≥ this many crossing corridors AND degree ≤ 2 → CHOKEPOINT */
    private static final int CHOKE_THRESHOLD   = 3;

    // ── Public entry point ───────────────────────────────────────────────────

    /**
     * Classify every active box-goal in the level.
     *
     * @param activeGoals   subset of goals that still need to be satisfied
     *                      (from TaskFilter.FilterResult.activeGoals)
     * @param level         immutable level geometry
     * @param initialState  initial world state (used only for box positions during BFS)
     * @return map from goal position → GoalProfile, one entry per activeGoal
     */
    public static Map<Position, GoalProfile> analyze(
            List<Position> activeGoals, Level level, State initialState) {

        // Collect only box-goals (agent goals are not transit targets)
        List<Position> boxGoals = new ArrayList<>();
        for (Position g : activeGoals) {
            if (level.getBoxGoal(g) != '\0') boxGoals.add(g);
        }
        if (boxGoals.isEmpty()) return Collections.emptyMap();

        // ── Step 1: For every (box → goal) pair, find BFS corridor (walls-only obstacles)
        //            Count how many corridors pass through each goal cell.
        //            crossingMatrix[i] = list of goal indices j whose corridor passes through goal[i]
        int n = boxGoals.size();
        int[] crossingCount = new int[n];
        List<List<Integer>> crossingGoalIndices = new ArrayList<>();
        for (int i = 0; i < n; i++) crossingGoalIndices.add(new ArrayList<>());

        // Build a set of box-goal positions for O(1) lookup
        Set<Position> boxGoalSet = new HashSet<>(boxGoals);

        for (int j = 0; j < n; j++) {
            Position destGoal = boxGoals.get(j);
            char goalType = level.getBoxGoal(destGoal);

            // Find the nearest box of matching type in the initial state
            // (We ignore box-blocking — want ideal geometric corridor)
            Position nearestBox = findNearestBox(destGoal, goalType, initialState);
            if (nearestBox == null) continue;

            // BFS from box to goal, treating only walls as obstacles
            Set<Position> corridor = bfsCorridor(nearestBox, destGoal, level);
            if (corridor == null) continue;

            // For every other goal position that lies on this corridor, increment crossing
            for (int i = 0; i < n; i++) {
                if (i == j) continue;   // skip self
                if (corridor.contains(boxGoals.get(i))) {
                    crossingCount[i]++;
                    crossingGoalIndices.get(i).add(j);
                }
            }
        }

        // ── Step 2: Compute free-neighbour degree for each goal cell
        int[] degree = new int[n];
        for (int i = 0; i < n; i++) {
            degree[i] = freeNeighbours(boxGoals.get(i), level);
        }

        // ── Step 3: Classify
        Map<Position, GoalProfile> result = new LinkedHashMap<>();
        for (int i = 0; i < n; i++) {
            Position gp = boxGoals.get(i);
            char gt = level.getBoxGoal(gp);
            int cc = crossingCount[i];
            int deg = degree[i];

            TransitProfile profile;
            if (deg <= 1) {
                profile = TransitProfile.TERMINAL;   // dead-end — no one passes through
            } else if (cc >= CHOKE_THRESHOLD && deg <= 2) {
                profile = TransitProfile.CHOKEPOINT;
            } else if (cc >= TRANSIT_THRESHOLD) {
                profile = TransitProfile.TRANSIT;
            } else {
                profile = TransitProfile.NEUTRAL;
            }

            List<Position> crossingPositions = new ArrayList<>();
            for (int j : crossingGoalIndices.get(i)) crossingPositions.add(boxGoals.get(j));

            result.put(gp, new GoalProfile(gp, gt, profile, cc, deg, crossingPositions));
        }

        return result;
    }

    /**
     * Print a structured diagnostic summary of the transit classification.
     * Called once at planning start — output is compact (1-3 lines per TRANSIT/CHOKEPOINT).
     */
    public static void printDiagnostic(Map<Position, GoalProfile> profiles, String levelName) {
        if (profiles.isEmpty()) return;

        long transitCount    = profiles.values().stream().filter(p -> p.profile == TransitProfile.TRANSIT).count();
        long chopCount       = profiles.values().stream().filter(p -> p.profile == TransitProfile.CHOKEPOINT).count();
        long terminalCount   = profiles.values().stream().filter(p -> p.profile == TransitProfile.TERMINAL).count();
        long neutralCount    = profiles.values().stream().filter(p -> p.profile == TransitProfile.NEUTRAL).count();

        System.err.printf("[TRANSIT] %s — goals: %d  TERMINAL=%d  NEUTRAL=%d  TRANSIT=%d  CHOKEPOINT=%d%n",
                levelName, profiles.size(), terminalCount, neutralCount, transitCount, chopCount);

        // Print each non-NEUTRAL goal in ascending crossing-count order
        profiles.values().stream()
                .filter(p -> p.profile != TransitProfile.NEUTRAL)
                .sorted(Comparator.comparing((GoalProfile p) -> p.profile.ordinal())
                        .reversed()   // CHOKEPOINT first, then TRANSIT, then TERMINAL
                        .thenComparing(p -> -p.crossingCount))
                .forEach(p -> {
                    StringBuilder sb = new StringBuilder();
                    sb.append(String.format("[TRANSIT] %s  goal=%c@%s  crossings=%d  degree=%d",
                            p.profile, p.goalType, p.goalPos, p.crossingCount, p.degree));
                    if (!p.crossingGoals.isEmpty()) {
                        sb.append("  via=[");
                        for (int k = 0; k < Math.min(p.crossingGoals.size(), 4); k++) {
                            if (k > 0) sb.append(",");
                            sb.append(p.crossingGoals.get(k));
                        }
                        if (p.crossingGoals.size() > 4) sb.append(",...");
                        sb.append("]");
                    }
                    System.err.println(sb);
                });
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /** Find the nearest box of given type in state (Manhattan distance). */
    private static Position findNearestBox(Position goal, char goalType, State state) {
        Position best = null;
        int bestDist  = Integer.MAX_VALUE;
        for (Map.Entry<Position, Character> e : state.getBoxes().entrySet()) {
            if (e.getValue() == goalType) {
                int d = Math.abs(e.getKey().row - goal.row) + Math.abs(e.getKey().col - goal.col);
                if (d < bestDist) { bestDist = d; best = e.getKey(); }
            }
        }
        return best;
    }

    /**
     * BFS from {@code start} to {@code end} treating only walls as obstacles.
     * Returns the SET of all cells on the shortest path, or null if unreachable.
     */
    private static Set<Position> bfsCorridor(Position start, Position end, Level level) {
        if (start.equals(end)) return Collections.singleton(start);

        Map<Position, Position> parent = new HashMap<>();
        Queue<Position> queue = new ArrayDeque<>();
        parent.put(start, null);
        queue.add(start);

        while (!queue.isEmpty()) {
            Position cur = queue.poll();
            if (cur.equals(end)) {
                // Reconstruct path as set
                Set<Position> path = new HashSet<>();
                Position p = cur;
                while (p != null) { path.add(p); p = parent.get(p); }
                return path;
            }
            for (Direction dir : Direction.values()) {
                Position next = cur.move(dir);
                if (!level.isWall(next) && !parent.containsKey(next)) {
                    parent.put(next, cur);
                    queue.add(next);
                }
            }
        }
        return null;
    }

    /** Count non-wall neighbours of a position. */
    private static int freeNeighbours(Position pos, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            if (!level.isWall(pos.move(dir))) count++;
        }
        return count;
    }
}
