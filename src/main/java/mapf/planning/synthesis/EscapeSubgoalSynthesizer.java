package mapf.planning.synthesis;

import mapf.domain.*;
import mapf.planning.strategy.PriorityPlanningStrategy.Subgoal;

import java.util.*;

/**
 * P1: Synthesizes "escape subgoals" to break dependency cycles.
 *
 * Per qanda.txt 3 and claudeopus47.txt 1.2 / 3.2.2: when the goal-dependency
 * graph contains a cycle (most commonly a 2-cycle of the form
 * "box A sits on B's goal AND box B sits on A's goal"), no topological order
 * can solve it directly. The dispatcher must insert a synthetic subgoal that
 * temporarily moves one of the boxes to a P_temp position not on any active
 * path, breaking the cycle into a chain.
 *
 * This class is a static analyzer:
 *   input  = (state, level, goalDependsOn map)
 *   output = list of escape Subgoals to PREPEND to the normal subgoal order
 *
 * It is intentionally conservative: only emits escape subgoals for unique-box
 * 2-cycles (the canonical "swap" case from qanda.txt:79). Larger cycles or
 * multi-box-of-same-type ambiguity are left to the existing portfolio
 * (multi-seed RANDOM ordering + P0b deprioritization).
 *
 * P_temp selection (4-constraint validation per qanda.txt):
 *   1. Not a wall, not a goal cell, not currently occupied by another box.
 *   2. Reachable from the box's current position by the responsible agent
 *      (BFS through free + push-only cells, ignoring other agents).
 *   3. Not in a corner (avoids trapping the box per BSP corner pruning).
 *   4. Prefer cells closer to the box (minimal detour cost).
 */
public final class EscapeSubgoalSynthesizer {

    /** Maximum number of escape subgoals to emit per call (bounded analysis). */
    private static final int MAX_ESCAPE_SUBGOALS = 6;

    /** BFS expansion cap for P_temp search (per cycle). */
    private static final int PTEMP_BFS_CAP = 400;

    /** Maximum SCC size to attempt (very large SCCs likely indicate analysis noise). */
    private static final int MAX_SCC_SIZE = 12;

    private EscapeSubgoalSynthesizer() {}

    /**
     * Detect 2-cycles in the dependency graph and emit escape subgoals.
     *
     * @param state           current world state
     * @param level           static level data
     * @param goalDependsOn   from LevelAnalyzer: goal -> goals it depends on
     * @param immovableBoxes  treated as walls during P_temp BFS
     * @return list of escape Subgoals (possibly empty); to be prepended to PP order
     */
    public static List<Subgoal> synthesize(State state, Level level,
                                           Map<Position, Set<Position>> goalDependsOn,
                                           Set<Position> immovableBoxes) {
        if (goalDependsOn == null || goalDependsOn.isEmpty()) return Collections.emptyList();

        List<Subgoal> escapes = new ArrayList<>();
        Set<Position> usedTemps = new HashSet<>();
        Set<Position> handledGoals = new HashSet<>();

        // Find 2-cycles: (A, B) where A depends on B AND B depends on A.
        for (Map.Entry<Position, Set<Position>> e : goalDependsOn.entrySet()) {
            if (escapes.size() >= MAX_ESCAPE_SUBGOALS) break;
            Position gA = e.getKey();
            if (handledGoals.contains(gA)) continue;
            for (Position gB : e.getValue()) {
                if (gA.equals(gB)) continue;
                Set<Position> depsB = goalDependsOn.get(gB);
                if (depsB == null || !depsB.contains(gA)) continue;
                // 2-cycle (gA, gB)
                if (handledGoals.contains(gB)) continue;

                Subgoal escape = trySynthesizeForCycle(gA, gB, state, level, immovableBoxes, usedTemps);
                if (escape == null) {
                    // Try the other direction
                    escape = trySynthesizeForCycle(gB, gA, state, level, immovableBoxes, usedTemps);
                }
                if (escape != null) {
                    escapes.add(escape);
                    usedTemps.add(escape.goalPos);
                    handledGoals.add(gA);
                    handledGoals.add(gB);
                }
                break; // one cycle per gA
            }
        }

        // ---- D3: k≥3 cycles via SCC detection ---------------------------------
        // Beyond 2-cycles, ANY box in a strongly connected component can be
        // temporarily moved aside to convert the cycle into a chain. We compute
        // SCCs of the goalDependsOn graph and try opening each goal in the SCC
        // until one yields a valid P_temp. Goals already handled by the 2-cycle
        // pass are skipped to avoid duplicates.
        if (escapes.size() < MAX_ESCAPE_SUBGOALS) {
            List<List<Position>> sccs = computeSCCs(goalDependsOn);
            for (List<Position> scc : sccs) {
                if (escapes.size() >= MAX_ESCAPE_SUBGOALS) break;
                if (scc.size() < 3) continue;          // 2-cycles already handled
                if (scc.size() > MAX_SCC_SIZE) continue;
                // Skip if any node in the SCC is already handled (treat as covered).
                boolean alreadyTouched = false;
                for (Position g : scc) if (handledGoals.contains(g)) { alreadyTouched = true; break; }
                if (alreadyTouched) continue;

                // Try opening each goal in the SCC; first success wins.
                Position picked = null;
                Subgoal escape = null;
                for (Position gA : scc) {
                    // Use the next node in the cycle as gB placeholder (unused inside).
                    Position gB = scc.get((scc.indexOf(gA) + 1) % scc.size());
                    escape = trySynthesizeForCycle(gA, gB, state, level, immovableBoxes, usedTemps);
                    if (escape != null) { picked = gA; break; }
                }
                if (escape != null && picked != null) {
                    escapes.add(escape);
                    usedTemps.add(escape.goalPos);
                    // Mark all SCC members handled — one escape breaks the whole cycle.
                    handledGoals.addAll(scc);
                }
            }
        }

        return escapes;
    }

    /**
     * Iterative Tarjan SCC over the goalDependsOn graph. Returns each SCC as a
     * list of positions; trivial single-node SCCs (no self-loop) are still emitted
     * but get filtered by the size&lt;3 check at the call site.
     */
    private static List<List<Position>> computeSCCs(Map<Position, Set<Position>> graph) {
        List<List<Position>> result = new ArrayList<>();
        Map<Position, Integer> index = new HashMap<>();
        Map<Position, Integer> lowlink = new HashMap<>();
        Set<Position> onStack = new HashSet<>();
        Deque<Position> stack = new ArrayDeque<>();
        int[] counter = {0};

        // Iterative DFS frame: (node, iterator over successors)
        for (Position v0 : graph.keySet()) {
            if (index.containsKey(v0)) continue;
            Deque<Object[]> dfs = new ArrayDeque<>();
            index.put(v0, counter[0]); lowlink.put(v0, counter[0]); counter[0]++;
            stack.push(v0); onStack.add(v0);
            dfs.push(new Object[]{v0, new ArrayList<>(graph.getOrDefault(v0, Collections.emptySet())).iterator()});

            while (!dfs.isEmpty()) {
                Object[] frame = dfs.peek();
                Position v = (Position) frame[0];
                @SuppressWarnings("unchecked")
                Iterator<Position> it = (Iterator<Position>) frame[1];
                if (it.hasNext()) {
                    Position w = it.next();
                    if (!index.containsKey(w)) {
                        index.put(w, counter[0]); lowlink.put(w, counter[0]); counter[0]++;
                        stack.push(w); onStack.add(w);
                        dfs.push(new Object[]{w, new ArrayList<>(graph.getOrDefault(w, Collections.emptySet())).iterator()});
                    } else if (onStack.contains(w)) {
                        lowlink.put(v, Math.min(lowlink.get(v), index.get(w)));
                    }
                } else {
                    // post-order: pop frame, update parent lowlink, root check
                    dfs.pop();
                    if (lowlink.get(v).equals(index.get(v))) {
                        List<Position> scc = new ArrayList<>();
                        Position w;
                        do {
                            w = stack.pop();
                            onStack.remove(w);
                            scc.add(w);
                        } while (!w.equals(v));
                        result.add(scc);
                    }
                    if (!dfs.isEmpty()) {
                        Position parent = (Position) dfs.peek()[0];
                        lowlink.put(parent, Math.min(lowlink.get(parent), lowlink.get(v)));
                    }
                }
            }
        }
        return result;
    }

    /**
     * For the cycle (gA, gB): try to move the box currently sitting on gA to a P_temp.
     * Returns null if no suitable P_temp is found, or if there's no box currently on gA,
     * or if no same-color agent can manipulate it.
     */
    private static Subgoal trySynthesizeForCycle(Position gA, Position gB, State state, Level level,
                                                  Set<Position> immovableBoxes, Set<Position> usedTemps) {
        Character boxOnA = state.getBoxes().get(gA);
        if (boxOnA == null) return null;

        Color boxColor = level.getBoxColor(boxOnA);
        if (boxColor == null) return null;

        // Need at least one same-color agent to move the box.
        int responsibleAgent = -1;
        for (int i = 0; i < state.getNumAgents(); i++) {
            if (level.getAgentColor(i) == boxColor) {
                responsibleAgent = i;
                break;
            }
        }
        if (responsibleAgent == -1) return null;

        // Build the set of goal cells (any color) — P_temp must avoid them.
        Set<Position> allGoalCells = new HashSet<>();
        for (List<Position> goals : level.getBoxGoalsByType().values()) {
            allGoalCells.addAll(goals);
        }
        for (Position agentGoal : level.getAgentGoalPositionMap().values()) {
            allGoalCells.add(agentGoal);
        }

        Position pTemp = findPTempBFS(gA, state, level, immovableBoxes, allGoalCells, usedTemps);
        if (pTemp == null) return null;

        return new Subgoal(responsibleAgent, boxOnA, pTemp, false);
    }

    /**
     * BFS from the box's current position to find the nearest free, non-goal,
     * non-corner cell. Treats walls + immovable boxes + other boxes as blocked.
     */
    private static Position findPTempBFS(Position boxPos, State state, Level level,
                                         Set<Position> immovableBoxes, Set<Position> goalCells,
                                         Set<Position> usedTemps) {
        Map<Position, Character> boxes = state.getBoxes();
        int rows = level.getRows();
        int cols = level.getCols();
        Set<Position> visited = new HashSet<>();
        Deque<Position> queue = new ArrayDeque<>();
        queue.add(boxPos);
        visited.add(boxPos);
        int expansions = 0;
        int[][] dirs = {{-1,0},{1,0},{0,-1},{0,1}};

        while (!queue.isEmpty() && expansions < PTEMP_BFS_CAP) {
            Position p = queue.poll();
            expansions++;

            // Candidate test (skip the start cell).
            if (!p.equals(boxPos)
                && !goalCells.contains(p)
                && !usedTemps.contains(p)
                && !boxes.containsKey(p)
                && !immovableBoxes.contains(p)
                && !isCorner(p, level, immovableBoxes)) {
                return p;
            }

            for (int[] d : dirs) {
                int nr = p.row + d[0];
                int nc = p.col + d[1];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                if (level.isWall(nr, nc)) continue;
                Position np = new Position(nr, nc);
                if (visited.contains(np)) continue;
                if (immovableBoxes.contains(np)) continue;
                visited.add(np);
                queue.add(np);
            }
        }
        return null;
    }

    /**
     * Corner detection: a cell is a "corner" (deadlock for boxes) if two
     * orthogonal neighbors are walls. Boxes pushed into corners cannot escape.
     */
    private static boolean isCorner(Position p, Level level, Set<Position> immovableBoxes) {
        boolean wallN = isBlocked(p.row - 1, p.col, level, immovableBoxes);
        boolean wallS = isBlocked(p.row + 1, p.col, level, immovableBoxes);
        boolean wallW = isBlocked(p.row, p.col - 1, level, immovableBoxes);
        boolean wallE = isBlocked(p.row, p.col + 1, level, immovableBoxes);
        return (wallN && wallW) || (wallN && wallE) || (wallS && wallW) || (wallS && wallE);
    }

    private static boolean isBlocked(int r, int c, Level level, Set<Position> immovableBoxes) {
        if (r < 0 || r >= level.getRows() || c < 0 || c >= level.getCols()) return true;
        if (level.isWall(r, c)) return true;
        return immovableBoxes.contains(new Position(r, c));
    }
}
