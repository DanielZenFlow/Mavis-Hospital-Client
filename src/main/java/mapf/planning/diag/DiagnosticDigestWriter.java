package mapf.planning.diag;

import mapf.domain.Action;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Writes small AI-facing diagnostics next to the full replay.
 * <p>
 * The replay JSON is optimized for browser fidelity. These digest files are
 * optimized for fast triage: aggregate the high-signal planner events and keep
 * only a compact action window around the final failure.
 */
final class DiagnosticDigestWriter {
    private static final int FAILURE_WINDOW_STEPS = 40;
    private static final int MAX_TOP_ITEMS = 12;
    private static final int MAX_EVENT_SAMPLES = 24;
    private static final int MAX_WINDOW_EVENTS_PER_STEP = 12;

    List<Path> write(Path replayPath,
                     Level level,
                     List<ReplayRecorder.Frame> frames,
                     PlanTrace planTrace,
                     String outcome,
                     int executedSteps,
                     int plannedSteps,
                     int satisfiedBoxGoals,
                     int totalBoxGoals) throws IOException {
        if ("solved".equals(outcome)) {
            return List.of();
        }

        List<PlanTrace.Event> events = collectEvents(frames, planTrace);
        List<PlanTrace.Event> highSignal = highSignalEvents(events);
        Path summaryPath = siblingWithSuffix(replayPath, ".diagnostic-summary.json");
        Path windowPath = siblingWithSuffix(replayPath, ".failure-window.json");

        Files.writeString(summaryPath, summaryJson(replayPath, level, frames, outcome,
                executedSteps, plannedSteps, satisfiedBoxGoals, totalBoxGoals, highSignal),
                StandardCharsets.UTF_8);
        Files.writeString(windowPath, failureWindowJson(replayPath, frames, highSignal,
                outcome, executedSteps), StandardCharsets.UTF_8);

        return List.of(summaryPath, windowPath);
    }

    private List<PlanTrace.Event> collectEvents(List<ReplayRecorder.Frame> frames, PlanTrace planTrace) {
        List<PlanTrace.Event> events = new ArrayList<>();
        if (frames == null || planTrace == null) return events;
        for (ReplayRecorder.Frame frame : frames) {
            events.addAll(planTrace.eventsForFrame(frame.t()));
        }
        events.sort(Comparator.comparingInt(PlanTrace.Event::frame));
        return events;
    }

    private List<PlanTrace.Event> highSignalEvents(List<PlanTrace.Event> events) {
        List<PlanTrace.Event> highSignal = new ArrayList<>();
        for (PlanTrace.Event event : events) {
            if (event == null) continue;
            String kind = text(event.kind());
            if ("agent-intent".equals(kind)) continue;
            if ("subgoal-eval".equals(kind) && "ACCEPTED".equals(text(event.verdict()))) continue;
            highSignal.add(event);
        }
        return highSignal;
    }

    private String summaryJson(Path replayPath,
                               Level level,
                               List<ReplayRecorder.Frame> frames,
                               String outcome,
                               int executedSteps,
                               int plannedSteps,
                               int satisfiedBoxGoals,
                               int totalBoxGoals,
                               List<PlanTrace.Event> highSignal) {
        StringBuilder sb = new StringBuilder(32_768);
        State finalState = frames.isEmpty() ? null : frames.get(frames.size() - 1).state();
        List<GoalStatus> unsatisfied = unsatisfiedBoxGoals(level, finalState);
        int finalWindowStart = Math.max(0, executedSteps - FAILURE_WINDOW_STEPS);
        List<PlanTrace.Event> finalWindowEvents = highSignal.stream()
                .filter(e -> e.frame() >= finalWindowStart && e.frame() <= executedSteps)
                .toList();

        sb.append("{\n");
        field(sb, 1, "schema", "mavis-hospital-diagnostic-summary-v1", true);
        field(sb, 1, "replayFile", replayPath.getFileName().toString(), true);
        field(sb, 1, "level", level.getName(), true);
        field(sb, 1, "outcome", outcome, true);
        numberField(sb, 1, "executedSteps", executedSteps, true);
        numberField(sb, 1, "plannedSteps", plannedSteps, true);
        numberField(sb, 1, "failureWindowStart", finalWindowStart, true);
        numberField(sb, 1, "failureWindowEnd", executedSteps, true);
        numberField(sb, 1, "satisfiedBoxGoals", satisfiedBoxGoals, true);
        numberField(sb, 1, "totalBoxGoals", totalBoxGoals, true);
        numberField(sb, 1, "unsatisfiedBoxGoals", unsatisfied.size(), true);

        sb.append(indent(1)).append("\"focus\": ");
        appendFocus(sb, highSignal, finalWindowEvents, 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"eventCounts\": ");
        appendCountMap(sb, countBy(highSignal, e -> text(e.kind())), 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"topReasons\": ");
        appendCountMap(sb, countBy(highSignal, e -> text(e.reason())), 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"topSubgoals\": ");
        appendCountMap(sb, countBy(highSignal, e -> text(e.subgoal())), 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"topBlockers\": ");
        appendCountMap(sb, countBy(highSignal, this::blockerKey), 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"unsatisfiedGoals\": ");
        appendUnsatisfiedGoals(sb, unsatisfied, 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"firstSymptoms\": ");
        appendEventSamples(sb, highSignal.stream().limit(MAX_EVENT_SAMPLES).toList(), 1);
        sb.append(",\n");

        sb.append(indent(1)).append("\"finalFailureCluster\": ");
        appendEventSamples(sb, finalWindowEvents.stream().limit(MAX_EVENT_SAMPLES).toList(), 1);
        sb.append('\n');
        sb.append("}\n");
        return sb.toString();
    }

    private String failureWindowJson(Path replayPath,
                                     List<ReplayRecorder.Frame> frames,
                                     List<PlanTrace.Event> highSignal,
                                     String outcome,
                                     int executedSteps) {
        StringBuilder sb = new StringBuilder(24_576);
        int start = Math.max(0, executedSteps - FAILURE_WINDOW_STEPS);
        int end = Math.min(executedSteps, frames.size() - 1);
        Map<Integer, List<PlanTrace.Event>> eventsByFrame = new LinkedHashMap<>();
        for (PlanTrace.Event event : highSignal) {
            if (event.frame() < start || event.frame() > end) continue;
            eventsByFrame.computeIfAbsent(event.frame(), ignored -> new ArrayList<>()).add(event);
        }

        sb.append("{\n");
        field(sb, 1, "schema", "mavis-hospital-failure-window-v1", true);
        field(sb, 1, "replayFile", replayPath.getFileName().toString(), true);
        field(sb, 1, "outcome", outcome, true);
        numberField(sb, 1, "windowStart", start, true);
        numberField(sb, 1, "windowEnd", end, true);
        numberField(sb, 1, "preFailureSteps", end - start, true);
        numberField(sb, 1, "postFailureSteps", 0, true);
        field(sb, 1, "note", "This compact file intentionally keeps only final-window actions and high-signal planner events.", true);

        sb.append(indent(1)).append("\"steps\": [\n");
        boolean first = true;
        for (int t = Math.max(1, start + 1); t <= end; t++) {
            ReplayRecorder.Frame frame = frames.get(t);
            if (!first) sb.append(",\n");
            appendWindowStep(sb, frame, eventsByFrame.getOrDefault(t, List.of()), 2);
            first = false;
        }
        sb.append('\n').append(indent(1)).append("]\n");
        sb.append("}\n");
        return sb.toString();
    }

    private void appendFocus(StringBuilder sb,
                             List<PlanTrace.Event> highSignal,
                             List<PlanTrace.Event> finalWindowEvents,
                             int depth) {
        Map.Entry<String, Integer> subgoal = firstTop(countBy(highSignal, e -> text(e.subgoal())));
        Map.Entry<String, Integer> finalSubgoal = firstTop(countBy(finalWindowEvents, e -> text(e.subgoal())));
        Map.Entry<String, Integer> blocker = firstTop(countBy(highSignal, this::blockerKey));
        Map.Entry<String, Integer> reason = firstTop(countBy(highSignal, e -> text(e.reason())));

        sb.append("{\n");
        field(sb, depth + 1, "primarySubgoal", subgoal == null ? "" : subgoal.getKey(), true);
        field(sb, depth + 1, "finalWindowSubgoal", finalSubgoal == null ? "" : finalSubgoal.getKey(), true);
        field(sb, depth + 1, "dominantBlocker", blocker == null ? "" : blocker.getKey(), true);
        field(sb, depth + 1, "dominantReason", reason == null ? "" : reason.getKey(), true);
        field(sb, depth + 1, "firstSymptom", highSignal.isEmpty() ? "" : eventOneLine(highSignal.get(0)), true);
        field(sb, depth + 1, "finalSymptom", finalWindowEvents.isEmpty()
                ? "" : eventOneLine(finalWindowEvents.get(finalWindowEvents.size() - 1)), false);
        sb.append(indent(depth)).append('}');
    }

    private void appendWindowStep(StringBuilder sb, ReplayRecorder.Frame frame,
                                  List<PlanTrace.Event> events, int depth) {
        sb.append(indent(depth)).append("{\n");
        numberField(sb, depth + 1, "t", frame.t(), true);
        sb.append(indent(depth + 1)).append("\"actions\": ");
        appendActionList(sb, frame.actions(), frame.accepted());
        if (!events.isEmpty()) {
            List<PlanTrace.Event> sampledEvents = diverseEventSample(events, MAX_WINDOW_EVENTS_PER_STEP);
            sb.append(",\n");
            numberField(sb, depth + 1, "eventCount", events.size(), true);
            numberField(sb, depth + 1, "eventOverflow", Math.max(0, events.size() - sampledEvents.size()), true);
            sb.append(indent(depth + 1)).append("\"events\": ");
            appendEventSamples(sb, sampledEvents, depth + 1);
        } else {
            sb.append('\n');
        }
        sb.append(indent(depth)).append('}');
    }

    private void appendActionList(StringBuilder sb, Action[] actions, boolean[] accepted) {
        sb.append('[');
        boolean first = true;
        if (actions != null) {
            for (int agent = 0; agent < actions.length; agent++) {
                Action action = actions[agent] == null ? Action.noOp() : actions[agent];
                boolean ok = accepted != null && agent < accepted.length && accepted[agent];
                if (action.type == Action.ActionType.NOOP && ok) continue;
                if (!first) sb.append(", ");
                sb.append("{\"agent\":").append(agent).append(",\"action\":");
                quoted(sb, action.toServerString());
                sb.append(",\"accepted\":").append(ok).append('}');
                first = false;
            }
        }
        sb.append(']');
    }

    private void appendEventSamples(StringBuilder sb, List<PlanTrace.Event> events, int depth) {
        sb.append("[\n");
        for (int i = 0; i < events.size(); i++) {
            if (i > 0) sb.append(",\n");
            appendCompactEvent(sb, events.get(i), depth + 1);
        }
        sb.append('\n').append(indent(depth)).append(']');
    }

    private List<PlanTrace.Event> diverseEventSample(List<PlanTrace.Event> events, int limit) {
        if (events.size() <= limit) return events;
        List<PlanTrace.Event> sampled = new ArrayList<>();
        Map<String, PlanTrace.Event> bySignature = new LinkedHashMap<>();
        for (PlanTrace.Event event : events) {
            bySignature.putIfAbsent(eventSignature(event), event);
        }
        for (PlanTrace.Event event : bySignature.values()) {
            sampled.add(event);
            if (sampled.size() >= limit) return sampled;
        }
        for (PlanTrace.Event event : events) {
            if (sampled.contains(event)) continue;
            sampled.add(event);
            if (sampled.size() >= limit) break;
        }
        return sampled;
    }

    private String eventSignature(PlanTrace.Event event) {
        return text(event.kind()) + "|"
                + text(event.reason()) + "|"
                + text(event.subgoal()) + "|"
                + text(event.details().get("blocker")) + "|"
                + text(event.details().get("blockerType"));
    }

    private void appendCompactEvent(StringBuilder sb, PlanTrace.Event event, int depth) {
        sb.append(indent(depth)).append("{\n");
        numberField(sb, depth + 1, "frame", event.frame(), true);
        field(sb, depth + 1, "kind", event.kind(), true);
        field(sb, depth + 1, "severity", event.severity(), true);
        field(sb, depth + 1, "title", event.title(), true);
        if (event.subgoal() != null) field(sb, depth + 1, "subgoal", event.subgoal(), true);
        if (event.reason() != null) field(sb, depth + 1, "reason", event.reason(), true);
        if (event.verdict() != null) field(sb, depth + 1, "verdict", event.verdict(), true);
        if (event.boxType() != null) field(sb, depth + 1, "boxType", String.valueOf(event.boxType()), true);
        appendDetailIfPresent(sb, depth + 1, event, "blocker", true);
        appendDetailIfPresent(sb, depth + 1, event, "blockerType", true);
        appendDetailIfPresent(sb, depth + 1, event, "targetBlocker", true);
        appendDetailIfPresent(sb, depth + 1, event, "parking", true);
        appendDetailIfPresent(sb, depth + 1, event, "dominantReason", true);
        appendDetailIfPresent(sb, depth + 1, event, "candidateRejectCounts", true);
        appendDetailIfPresent(sb, depth + 1, event, "bspReason", true);
        appendDetailIfPresent(sb, depth + 1, event, "bspExplored", true);
        appendDetailIfPresent(sb, depth + 1, event, "bspBudget", true);
        appendDetailIfPresent(sb, depth + 1, event, "attempted", true);
        appendDetailIfPresent(sb, depth + 1, event, "bspFailed", true);
        appendDetailIfPresent(sb, depth + 1, event, "validationFailed", true);
        appendDetailIfPresent(sb, depth + 1, event, "skippedNogood", true);
        appendDetailIfPresent(sb, depth + 1, event, "supportPhase", true);
        appendDetailIfPresent(sb, depth + 1, event, "supportKind", true);
        appendDetailIfPresent(sb, depth + 1, event, "candidateStrategies", true);
        appendDetailIfPresent(sb, depth + 1, event, "attemptedStrategies", true);
        appendDetailIfPresent(sb, depth + 1, event, "standaloneProgress", true);
        appendDetailIfPresent(sb, depth + 1, event, "parentPathAvailable", true);
        appendDetailIfPresent(sb, depth + 1, event, "parentSubgoal", true);
        appendDetailIfPresent(sb, depth + 1, event, "helperAgent", true);
        appendDetailIfPresent(sb, depth + 1, event, "stepInSegment", true);
        appendDetailIfPresent(sb, depth + 1, event, "segmentSteps", true);
        appendDetailIfPresent(sb, depth + 1, event, "supportAction", true);
        appendDetailIfPresent(sb, depth + 1, event, "actualAction", true);
        appendDetailIfPresent(sb, depth + 1, event, "planStart", true);
        appendDetailIfPresent(sb, depth + 1, event, "planEnd", true);
        appendDetailIfPresent(sb, depth + 1, event, "supportSteps", true);
        appendDetailIfPresent(sb, depth + 1, event, "target", true);
        appendDetailIfPresent(sb, depth + 1, event, "agentFrom", true);
        appendDetailIfPresent(sb, depth + 1, event, "agentTo", true);
        appendDetailIfPresent(sb, depth + 1, event, "boxFrom", true);
        appendDetailIfPresent(sb, depth + 1, event, "boxTo", true);
        appendDetailIfPresent(sb, depth + 1, event, "movedBox", true);
        appendDetailIfPresent(sb, depth + 1, event, "movedBoxes", true);
        appendDetailIfPresent(sb, depth + 1, event, "movedBoxCount", true);
        appendDetailIfPresent(sb, depth + 1, event, "accessBlockersBefore", true);
        appendDetailIfPresent(sb, depth + 1, event, "accessBlockersAfter", true);
        appendDetailIfPresent(sb, depth + 1, event, "ordinaryBlockersBefore", true);
        appendDetailIfPresent(sb, depth + 1, event, "ordinaryBlockersAfter", true);
        appendDetailIfPresent(sb, depth + 1, event, "satisfiedBefore", true);
        appendDetailIfPresent(sb, depth + 1, event, "satisfiedAfter", true);
        trimTrailingComma(sb);
        sb.append(indent(depth)).append('}');
    }

    private void appendDetailIfPresent(StringBuilder sb, int depth, PlanTrace.Event event,
                                       String key, boolean comma) {
        String value = event.details().get(key);
        if (value != null) field(sb, depth, key, value, comma);
    }

    private List<GoalStatus> unsatisfiedBoxGoals(Level level, State state) {
        List<GoalStatus> out = new ArrayList<>();
        if (state == null) return out;
        for (Position goal : level.getAllBoxGoalPositions()) {
            char expected = level.getBoxGoal(goal);
            char actual = state.getBoxAt(goal);
            if (actual != expected) {
                out.add(new GoalStatus(expected, actual == '\0' ? "" : String.valueOf(actual), goal));
            }
        }
        out.sort(Comparator.comparing((GoalStatus g) -> g.expected)
                .thenComparingInt(g -> g.position.row)
                .thenComparingInt(g -> g.position.col));
        return out;
    }

    private void appendUnsatisfiedGoals(StringBuilder sb, List<GoalStatus> goals, int depth) {
        sb.append("[\n");
        for (int i = 0; i < goals.size(); i++) {
            if (i > 0) sb.append(",\n");
            GoalStatus goal = goals.get(i);
            sb.append(indent(depth + 1)).append("{");
            quoted(sb, "expected");
            sb.append(":");
            quoted(sb, String.valueOf(goal.expected));
            sb.append(", ");
            quoted(sb, "actual");
            sb.append(":");
            quoted(sb, goal.actual);
            sb.append(", ");
            quoted(sb, "position");
            sb.append(":");
            quoted(sb, position(goal.position));
            sb.append('}');
        }
        sb.append('\n').append(indent(depth)).append(']');
    }

    private Map<String, Integer> countBy(List<PlanTrace.Event> events, EventKey keyFn) {
        Map<String, Integer> counts = new LinkedHashMap<>();
        for (PlanTrace.Event event : events) {
            String key = keyFn.key(event);
            if (key == null || key.isBlank()) continue;
            counts.merge(key, 1, Integer::sum);
        }
        return counts;
    }

    private void appendCountMap(StringBuilder sb, Map<String, Integer> counts, int depth) {
        List<Map.Entry<String, Integer>> rows = counts.entrySet().stream()
                .sorted(Map.Entry.<String, Integer>comparingByValue().reversed()
                        .thenComparing(Map.Entry.comparingByKey()))
                .limit(MAX_TOP_ITEMS)
                .toList();
        sb.append("{");
        if (!rows.isEmpty()) sb.append('\n');
        for (int i = 0; i < rows.size(); i++) {
            Map.Entry<String, Integer> row = rows.get(i);
            if (i > 0) sb.append(",\n");
            sb.append(indent(depth + 1));
            quoted(sb, row.getKey());
            sb.append(": ").append(row.getValue());
        }
        if (!rows.isEmpty()) sb.append('\n').append(indent(depth));
        sb.append('}');
    }

    private Map.Entry<String, Integer> firstTop(Map<String, Integer> counts) {
        return counts.entrySet().stream()
                .max(Comparator.<Map.Entry<String, Integer>>comparingInt(Map.Entry::getValue)
                        .thenComparing(Map.Entry::getKey))
                .orElse(null);
    }

    private String blockerKey(PlanTrace.Event event) {
        String blocker = event.details().get("blocker");
        if (blocker == null || blocker.isBlank()) return "";
        String type = event.details().get("blockerType");
        return type == null || type.isBlank() ? blocker : blocker + " " + type;
    }

    private String eventOneLine(PlanTrace.Event event) {
        StringBuilder sb = new StringBuilder();
        sb.append("frame ").append(event.frame()).append(" ");
        sb.append(text(event.kind()));
        if (event.subgoal() != null) sb.append(" subgoal=").append(event.subgoal());
        if (event.reason() != null) sb.append(" reason=").append(event.reason());
        String blocker = event.details().get("blocker");
        if (blocker != null) sb.append(" blocker=").append(blocker);
        return sb.toString();
    }

    private Path siblingWithSuffix(Path replayPath, String suffix) {
        String name = replayPath.getFileName().toString();
        if (name.endsWith(".json")) {
            name = name.substring(0, name.length() - ".json".length());
        }
        return replayPath.resolveSibling(name + suffix);
    }

    private static void field(StringBuilder sb, int depth, String name, String value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ");
        quoted(sb, value == null ? "" : value);
        if (comma) sb.append(',');
        sb.append('\n');
    }

    private static void numberField(StringBuilder sb, int depth, String name, int value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ").append(value);
        if (comma) sb.append(',');
        sb.append('\n');
    }

    private static void quoted(StringBuilder sb, String value) {
        sb.append('"');
        if (value != null) {
            for (int i = 0; i < value.length(); i++) {
                char ch = value.charAt(i);
                switch (ch) {
                    case '"' -> sb.append("\\\"");
                    case '\\' -> sb.append("\\\\");
                    case '\n' -> sb.append("\\n");
                    case '\r' -> sb.append("\\r");
                    case '\t' -> sb.append("\\t");
                    default -> {
                        if (ch < 32) sb.append(String.format("\\u%04x", (int) ch));
                        else sb.append(ch);
                    }
                }
            }
        }
        sb.append('"');
    }

    private static String indent(int depth) {
        return "  ".repeat(depth);
    }

    private static String text(String value) {
        return value == null ? "" : value;
    }

    private static String position(Position position) {
        return position == null ? "" : position.row + "," + position.col;
    }

    private static void trimTrailingComma(StringBuilder sb) {
        int comma = sb.length() - 2;
        if (comma >= 0 && sb.charAt(comma) == ',') {
            sb.deleteCharAt(comma);
        }
    }

    @FunctionalInterface
    private interface EventKey {
        String key(PlanTrace.Event event);
    }

    private record GoalStatus(char expected, String actual, Position position) {
    }
}
