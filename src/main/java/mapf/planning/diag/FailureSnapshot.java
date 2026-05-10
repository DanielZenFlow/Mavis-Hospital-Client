package mapf.planning.diag;

import mapf.domain.Color;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;
import mapf.planning.analysis.GoalTransitAnalyzer;
import mapf.planning.analysis.LevelAnalyzer.LevelFeatures;
import mapf.planning.signal.FailureReport;
import mapf.planning.strategy.PriorityPlanningStrategy.Subgoal;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * Writes a structured Markdown failure snapshot to {@code target/diagnostics/}.
 * <p>
 * The snapshot is the answer to "AI struggles parsing 36×45 ASCII grids": instead
 * of dumping raw logs and asking the user/AI to grep, we pre-aggregate the relevant
 * facts (level features, attempt history, the last PP failure, suspended/completed
 * goal sets) plus a small ASCII window centred on the blocker. One file per run.
 * <p>
 * Used by {@link mapf.planning.PortfolioController} when {@code search()} cannot
 * produce a full solution.
 */
public final class FailureSnapshot {

    private FailureSnapshot() {}

    /** Lightweight per-attempt record passed in from PortfolioController. */
    public static final class AttemptInfo {
        public final String phase;
        public final String label;       // e.g. "STRICT_ORDER(RANDOM#42)"
        public final long durationMs;
        public final boolean success;
        public final int planSteps;
        public final int unsatCount;
        public final String failedSubgoal;
        public final String failureKind;

        public AttemptInfo(String label, long durationMs, boolean success,
                           int planSteps, int unsatCount,
                           String failedSubgoal, String failureKind) {
            this.phase = phaseOf(label, failureKind);
            this.label = label;
            this.durationMs = durationMs;
            this.success = success;
            this.planSteps = planSteps;
            this.unsatCount = unsatCount;
            this.failedSubgoal = failedSubgoal;
            this.failureKind = failureKind;
        }
    }

    /**
     * Write a failure snapshot. Never throws — diagnostics must not crash the solver.
     *
     * @param outDir              destination directory (created if missing)
     * @param level               problem level
     * @param initialState        state before any planning
     * @param partialFinalState   state reached by the best partial plan (may equal initial)
     * @param features            analyzer features (may be null)
     * @param attempts            per-attempt records (may be empty)
     * @param lastReport          last PP FailureReport (may be null)
     * @param completedBoxGoals   from last PP instance (may be null)
     * @param suspendedGoals      from last PP instance (may be null)
     * @param transitProfiles     from last PP instance (may be null)
     * @param bestPartialSteps    length of best partial plan (0 if none)
     * @return file path written, or null if I/O failed
     */
    public static Path dump(Path outDir,
                            Level level,
                            State initialState,
                            State partialFinalState,
                            LevelFeatures features,
                            List<AttemptInfo> attempts,
                            FailureReport lastReport,
                            FailureReport bestPartialReport,
                            Set<Position> completedBoxGoals,
                            Set<Position> suspendedGoals,
                            Map<Position, GoalTransitAnalyzer.GoalProfile> transitProfiles,
                            int bestPartialSteps) {
        try {
            Files.createDirectories(outDir);
            String levelName = safeName(level.getName());
            Path file = outDir.resolve(levelName + "-failure.md");
            String body = render(level, initialState, partialFinalState, features,
                    attempts, lastReport, bestPartialReport,
                    completedBoxGoals, suspendedGoals,
                    transitProfiles, bestPartialSteps);
            Files.write(file, body.getBytes(StandardCharsets.UTF_8));
            return file;
        } catch (IOException e) {
            System.err.println("[FailureSnapshot] write failed: " + e.getMessage());
            return null;
        } catch (RuntimeException e) {
            System.err.println("[FailureSnapshot] render failed: " + e);
            return null;
        }
    }

    /** Convenience: default destination is {@code target/diagnostics/}. */
    public static Path defaultDir() {
        return Paths.get("target", "diagnostics");
    }

    // ---------- rendering ----------

    private static String render(Level level, State initial, State partialFinal,
                                 LevelFeatures f, List<AttemptInfo> attempts,
                                 FailureReport lastReport,
                                 FailureReport bestPartialReport,
                                 Set<Position> completed, Set<Position> suspended,
                                 Map<Position, GoalTransitAnalyzer.GoalProfile> profiles,
                                 int bestPartialSteps) {
        StringBuilder sb = new StringBuilder(8 * 1024);
        sb.append("# Failure Snapshot — ").append(level.getName()).append('\n');
        sb.append("Generated: ").append(new Date()).append("\n\n");

        // --- Header ---
        sb.append("## Level\n");
        sb.append("- Size: ").append(level.getRows()).append(" × ").append(level.getCols()).append('\n');
        sb.append("- Agents: ").append(level.getNumAgents()).append('\n');
        if (f != null) {
            sb.append("- Boxes: ").append(f.numBoxes).append("    Goals: ").append(f.numGoals).append('\n');
            sb.append("- Density: ").append(String.format(Locale.ROOT, "%.3f", f.density))
                    .append("    CorridorRatio: ").append(String.format(Locale.ROOT, "%.2f", f.corridorRatio))
                    .append('\n');
            sb.append("- HasCircularDependency: ").append(f.hasCircularDependency)
                    .append("    Coupling: ").append(String.format(Locale.ROOT, "%.2f", f.couplingDegree))
                    .append("    Recommended: ").append(f.recommendedStrategy).append('\n');
        }
        sb.append("- Best partial plan: ").append(bestPartialSteps).append(" steps\n\n");

        // --- Agent / color summary ---
        sb.append("## Agents (color → controllable box types)\n");
        Map<Color, List<Integer>> agentsByColor = new EnumMap<>(Color.class);
        Map<Color, Set<Character>> boxesByColor = new EnumMap<>(Color.class);
        for (int a = 0; a < level.getNumAgents(); a++) {
            Color c = level.getAgentColor(a);
            agentsByColor.computeIfAbsent(c, k -> new ArrayList<>()).add(a);
        }
        for (char box = 'A'; box <= 'Z'; box++) {
            Color c = level.getBoxColor(box);
            if (c != null) boxesByColor.computeIfAbsent(c, k -> new TreeSet<>()).add(box);
        }
        for (Map.Entry<Color, List<Integer>> e : agentsByColor.entrySet()) {
            Set<Character> boxes = boxesByColor.getOrDefault(e.getKey(), Collections.emptySet());
            sb.append("- ").append(e.getKey()).append(": agents=").append(e.getValue())
                    .append(", boxes=").append(boxes).append('\n');
        }
        sb.append('\n');

        // --- Attempts table ---
        sb.append("## Stage trace\n");
        if (attempts == null || attempts.isEmpty()) {
            sb.append("(none)\n\n");
        } else {
            appendPhaseSummary(sb, attempts);
            sb.append("| # | phase | attempt | ms | steps | kind | unsat | failed-subgoal |\n");
            sb.append("|---|-------|---------|----|-------|------|-------|----------------|\n");
            for (int i = 0; i < attempts.size(); i++) {
                AttemptInfo a = attempts.get(i);
                sb.append("| ").append(i + 1)
                        .append(" | ").append(a.phase)
                        .append(" | ").append(a.label)
                        .append(" | ").append(a.durationMs)
                        .append(" | ").append(a.planSteps)
                        .append(" | ").append(a.success ? "SUCCESS" : a.failureKind)
                        .append(" | ").append(a.unsatCount < 0 ? "-" : a.unsatCount)
                        .append(" | ").append(a.failedSubgoal == null ? "-" : a.failedSubgoal)
                        .append(" |\n");
            }
            sb.append('\n');
            appendBlockerSummary(sb, attempts);
        }

        // --- Goal status ---
        sb.append("## Goal status (after best partial)\n");
        List<Position> allBoxGoals = collectBoxGoals(level);
        State finalState = partialFinal != null ? partialFinal : initial;
        int satisfied = 0;
        List<String> unsatLines = new ArrayList<>();
        for (Position g : allBoxGoals) {
            char want = level.getBoxGoal(g);
            Character have = finalState.getBoxes().get(g);
            String tProf = profileTag(profiles, g);
            String marker;
            if (have != null && have == want) {
                satisfied++;
                continue;
            } else {
                marker = "✗";
            }
            String suspTag = (suspended != null && suspended.contains(g)) ? " [SUSPENDED]" : "";
            unsatLines.add(String.format(Locale.ROOT, "  %s %c@%s %s%s — have=%s",
                    marker, want, g, tProf, suspTag, have == null ? "(empty)" : ("'" + have + "'")));
        }
        sb.append("- Satisfied box goals: ").append(satisfied).append(" / ").append(allBoxGoals.size()).append('\n');
        if (completed != null) {
            sb.append("- PP completedBoxGoals (last attempt): ").append(completed.size()).append('\n');
        }
        if (suspended != null && !suspended.isEmpty()) {
            sb.append("- PP suspendedTransitGoals: ").append(suspended).append('\n');
        }
        sb.append("- Unsatisfied box goals:\n");
        if (unsatLines.isEmpty()) {
            sb.append("  (none — all box goals satisfied)\n");
        } else {
            for (String s : unsatLines) sb.append(s).append('\n');
        }
        sb.append('\n');

        // --- Last failure report ---
        sb.append("## Last PP FailureReport\n");
        if (lastReport == null) {
            sb.append("(none)\n\n");
        } else {
            sb.append("- Kind: ").append(lastReport.kind).append('\n');
            sb.append("- Cause: ").append(lastReport.cause).append('\n');
            sb.append("- Note: ").append(lastReport.note).append('\n');
            if (lastReport.blockedGoals != null && !lastReport.blockedGoals.isEmpty()) {
                sb.append("- Blocked goals: ").append(lastReport.blockedGoals).append('\n');
            }
            if (lastReport.blockedPositions != null && !lastReport.blockedPositions.isEmpty()) {
                sb.append("- Blocked cells: ").append(lastReport.blockedPositions).append('\n');
            }
            if (lastReport.lastAttemptedSubgoal != null) {
                Subgoal sg = lastReport.lastAttemptedSubgoal;
                sb.append("- Last attempted: agent").append(sg.agentId)
                        .append(sg.isAgentGoal ? " ->@" : " ->" + sg.boxType + "@")
                        .append(sg.goalPos)
                        .append(' ').append(profileTag(profiles, sg.goalPos))
                        .append('\n');
            }
            if (lastReport.unsatisfiedAtFailure != null && !lastReport.unsatisfiedAtFailure.isEmpty()) {
                sb.append("- Unsatisfied at failure: ").append(lastReport.unsatisfiedAtFailure.size())
                        .append(" subgoal(s)\n");
            }
            sb.append('\n');
        }

        // --- Local ASCII window centered on blocker ---
        // Render up to TWO views: best-partial blocker (most informative) + last attempt blocker.
        Position bestCentre = (bestPartialReport != null && bestPartialReport.lastAttemptedSubgoal != null)
                ? bestPartialReport.lastAttemptedSubgoal.goalPos : null;
        Position lastCentre = pickCentre(lastReport, allBoxGoals, completed);
        if (bestCentre != null) {
            sb.append("## Local view — BEST PARTIAL blocker (15×15 around ").append(bestCentre).append(")\n");
            sb.append("Legend: `#`=wall  `.`=free  `0-9`=agent  `A-Z`=box  lower-case=goal-only cell  ")
                    .append("`*`=goal+matching box  `!`=goal+wrong box\n");
            sb.append("```\n");
            sb.append(renderWindow(level, partialFinal != null ? partialFinal : initial, bestCentre, 7));
            sb.append("```\n\n");
        }
        if (lastCentre != null && !lastCentre.equals(bestCentre)) {
            sb.append("## Local view — LAST ATTEMPT blocker (15×15 around ").append(lastCentre).append(")\n");
            sb.append("```\n");
            sb.append(renderWindow(level, partialFinal != null ? partialFinal : initial, lastCentre, 7));
            sb.append("```\n\n");
        }

        // --- Diagnosis hints ---
        sb.append("## Diagnosis hints\n");
        appendHints(sb, lastReport, profiles, suspended, completed, allBoxGoals);
        return sb.toString();
    }

    // ---------- helpers ----------

    private static String safeName(String name) {
        if (name == null || name.isEmpty()) return "level";
        return name.replaceAll("[^A-Za-z0-9._-]", "_");
    }

    private static String phaseOf(String label, String failureKind) {
        String l = label == null ? "" : label;
        String k = failureKind == null ? "" : failureKind;
        if (l.startsWith("L3-ResidualOrderRepair")) return "L3 residual-order repair";
        if (l.startsWith("L3-PartialPlanContinuation")) return "L3 partial-plan continuation";
        if (l.startsWith("L3-CBSRBaseContinuation")) return "L3 CBSR-base continuation";
        if (k.contains("[CBSR#")) return "L3 CBSR ordering repair";
        if (l.contains("DISTANCE") || l.contains("TOPOLOGICAL") || l.contains("SINGLE_AGENT")) {
            return "initial ordering probe";
        }
        return "other";
    }

    private static void appendPhaseSummary(StringBuilder sb, List<AttemptInfo> attempts) {
        Map<String, long[]> byPhase = new LinkedHashMap<>();
        for (AttemptInfo a : attempts) {
            long[] row = byPhase.computeIfAbsent(a.phase, k -> new long[4]);
            row[0]++;
            row[1] += a.durationMs;
            row[2] = Math.max(row[2], a.planSteps);
            if (a.success) row[3]++;
        }
        sb.append("### Time by phase\n");
        sb.append("| phase | attempts | ms | best steps | success |\n");
        sb.append("|-------|----------|----|------------|---------|\n");
        for (Map.Entry<String, long[]> e : byPhase.entrySet()) {
            long[] row = e.getValue();
            sb.append("| ").append(e.getKey())
                    .append(" | ").append(row[0])
                    .append(" | ").append(row[1])
                    .append(" | ").append(row[2])
                    .append(" | ").append(row[3])
                    .append(" |\n");
        }
        sb.append('\n');
        sb.append("### Attempt timeline\n");
    }

    private static void appendBlockerSummary(StringBuilder sb, List<AttemptInfo> attempts) {
        Map<String, Integer> blockers = new LinkedHashMap<>();
        Map<String, Integer> kinds = new LinkedHashMap<>();
        for (AttemptInfo a : attempts) {
            if (a.failedSubgoal != null && !a.failedSubgoal.isEmpty()) {
                blockers.merge(a.failedSubgoal, 1, Integer::sum);
            }
            if (a.failureKind != null && !a.failureKind.isEmpty()) {
                kinds.merge(normalizeFailureKind(a.failureKind), 1, Integer::sum);
            }
        }
        if (!blockers.isEmpty()) {
            sb.append("### Repeated blockers\n");
            sb.append("| failed subgoal | count |\n");
            sb.append("|----------------|-------|\n");
            blockers.entrySet().stream()
                    .sorted((a, b) -> Integer.compare(b.getValue(), a.getValue()))
                    .limit(8)
                    .forEach(e -> sb.append("| ").append(e.getKey())
                            .append(" | ").append(e.getValue()).append(" |\n"));
            sb.append('\n');
        }
        if (!kinds.isEmpty()) {
            sb.append("### Failure kinds\n");
            sb.append("| kind | count |\n");
            sb.append("|------|-------|\n");
            kinds.entrySet().stream()
                    .sorted((a, b) -> Integer.compare(b.getValue(), a.getValue()))
                    .limit(8)
                    .forEach(e -> sb.append("| ").append(e.getKey())
                            .append(" | ").append(e.getValue()).append(" |\n"));
            sb.append('\n');
        }
    }

    private static String normalizeFailureKind(String kind) {
        int close = kind.indexOf(']');
        if (kind.startsWith("[CBSR#") && close >= 0 && close + 1 < kind.length()) {
            return kind.substring(close + 1);
        }
        return kind;
    }

    private static List<Position> collectBoxGoals(Level level) {
        List<Position> out = new ArrayList<>();
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                if (level.getBoxGoal(r, c) != '\0') out.add(Position.of(r, c));
            }
        }
        return out;
    }

    private static String profileTag(Map<Position, GoalTransitAnalyzer.GoalProfile> profiles,
                                     Position p) {
        if (profiles == null || p == null) return "";
        GoalTransitAnalyzer.GoalProfile pr = profiles.get(p);
        if (pr == null) return "";
        return "[" + pr.profile + "]";
    }

    private static Position pickCentre(FailureReport rep, List<Position> goals,
                                       Set<Position> completed) {
        if (rep != null && rep.blockedPositions != null && !rep.blockedPositions.isEmpty()) {
            return rep.blockedPositions.get(0);
        }
        if (rep != null && rep.blockedGoals != null && !rep.blockedGoals.isEmpty()) {
            return rep.blockedGoals.get(0);
        }
        if (rep != null && rep.lastAttemptedSubgoal != null
                && rep.lastAttemptedSubgoal.goalPos != null) {
            return rep.lastAttemptedSubgoal.goalPos;
        }
        // Fallback: first unsatisfied goal
        if (goals != null && completed != null) {
            for (Position g : goals) {
                if (!completed.contains(g)) return g;
            }
        }
        return goals == null || goals.isEmpty() ? null : goals.get(0);
    }

    private static String renderWindow(Level level, State state, Position centre, int radius) {
        int r0 = Math.max(0, centre.row - radius);
        int r1 = Math.min(level.getRows() - 1, centre.row + radius);
        int c0 = Math.max(0, centre.col - radius);
        int c1 = Math.min(level.getCols() - 1, centre.col + radius);
        StringBuilder sb = new StringBuilder();
        // header: column ticks every 5
        sb.append("      ");
        for (int c = c0; c <= c1; c++) sb.append(c % 10);
        sb.append('\n');
        for (int r = r0; r <= r1; r++) {
            sb.append(String.format(Locale.ROOT, "%4d  ", r));
            for (int c = c0; c <= c1; c++) {
                Position p = Position.of(r, c);
                if (level.isWall(r, c)) { sb.append('#'); continue; }
                int agentHere = state.getAgentAt(p);
                if (agentHere >= 0) { sb.append((char) ('0' + agentHere)); continue; }
                Character box = state.getBoxes().get(p);
                char goal = level.getBoxGoal(r, c);
                if (box != null) {
                    if (goal != '\0') sb.append(box == goal ? '*' : '!');
                    else sb.append(box);
                } else if (goal != '\0') {
                    sb.append(Character.toLowerCase(goal));
                } else {
                    sb.append('.');
                }
            }
            sb.append('\n');
        }
        return sb.toString();
    }

    private static void appendHints(StringBuilder sb, FailureReport rep,
                                    Map<Position, GoalTransitAnalyzer.GoalProfile> profiles,
                                    Set<Position> suspended, Set<Position> completed,
                                    List<Position> goals) {
        boolean any = false;
        if (rep != null && rep.lastAttemptedSubgoal != null
                && rep.lastAttemptedSubgoal.goalPos != null && profiles != null) {
            Position g = rep.lastAttemptedSubgoal.goalPos;
            GoalTransitAnalyzer.GoalProfile pr = profiles.get(g);
            if (pr != null && pr.profile != GoalTransitAnalyzer.TransitProfile.NEUTRAL
                    && pr.profile != GoalTransitAnalyzer.TransitProfile.TERMINAL) {
                sb.append("- Last failed goal ").append(g).append(" is ").append(pr.profile)
                        .append(" (crossings=").append(pr.crossingCount)
                        .append("); other boxes' BFS corridors run through it. Filling it last is correct;");
                sb.append(" check that its target box has a path that doesn't depend on already-suspended goals.\n");
                any = true;
            }
        }
        if (suspended != null && !suspended.isEmpty()) {
            sb.append("- ").append(suspended.size())
                    .append(" goal(s) are SUSPENDED. They were repeatedly disturbed by other subgoals; ")
                    .append("if the same one keeps reappearing across runs, its TRANSIT priority may need promotion.\n");
            any = true;
        }
        if (completed != null && goals != null && !goals.isEmpty()) {
            int unsat = 0;
            for (Position g : goals) if (!completed.contains(g)) unsat++;
            if (unsat == 1) {
                sb.append("- Only **1** box goal remains unsatisfied — the failure is a single localized blocker, ")
                        .append("not a global ordering bug.\n");
                any = true;
            }
        }
        if (!any) sb.append("(no automatic hints — inspect attempts table and local view above)\n");
    }
}
