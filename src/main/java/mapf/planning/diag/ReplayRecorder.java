package mapf.planning.diag;

import mapf.domain.Action;
import mapf.domain.Color;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Writes replay data and a static HTML reviewer for offline debugging.
 * <p>
 * The JSON stores full post-step states rather than asking the browser to
 * reimplement Hospital-domain joint-action semantics. That keeps the replay
 * aligned with the server results the client actually received.
 */
public final class ReplayRecorder {
    private static final String VIEWER_RESOURCE_DIR = "replay-viewer/";

    private final Level level;
    private final List<Frame> frames = new ArrayList<>();
    private PlanTrace planTrace = new PlanTrace();
    private final long createdAt = System.currentTimeMillis();
    private String outcome = "unknown";
    private int executedSteps = 0;
    private int plannedSteps = 0;
    private List<Path> lastDiagnosticFiles = List.of();

    private ReplayRecorder(Level level, State initialState) {
        this.level = level;
        frames.add(Frame.initial(initialState));
    }

    public static ReplayRecorder start(Level level, State initialState) {
        return new ReplayRecorder(level, initialState);
    }

    public void recordStep(int step, Action[] actions, boolean[] accepted, State stateAfter) {
        frames.add(new Frame(step + 1, actions, accepted, stateAfter));
    }

    public void finish(boolean solved, int executedSteps, int plannedSteps) {
        this.outcome = solved ? "solved" : (plannedSteps > 0 ? "partial" : "no-plan");
        this.executedSteps = Math.max(0, executedSteps);
        this.plannedSteps = Math.max(0, plannedSteps);
        this.planTrace.truncate(this.executedSteps);
    }

    public void setPlanTrace(PlanTrace planTrace) {
        this.planTrace = planTrace == null ? new PlanTrace() : planTrace.copy();
    }

    public List<Path> getLastDiagnosticFiles() {
        return lastDiagnosticFiles;
    }

    public Path writeDefault() {
        try {
            Path root = Paths.get("target", "diagnostics");
            String levelName = safeName(level.getName());
            Path replayDir = root.resolve("replays").resolve(levelName);
            Path viewerDir = root.resolve("replay-viewer");
            Files.createDirectories(replayDir);
            writeViewer(viewerDir);

            String stamp = new SimpleDateFormat("yyyy-MM-dd'T'HH-mm-ss", Locale.ROOT)
                    .format(new Date(createdAt));
            String json = toJson();
            Path stamped = replayDir.resolve(levelName + "__" + stamp + "__"
                    + safeName(outcome) + "__" + executedSteps + "-steps.json");
            Files.writeString(stamped, json, StandardCharsets.UTF_8);
            writeDiagnosticDigests(stamped);
            writeLatestReplayScript(viewerDir, json);
            return stamped;
        } catch (RuntimeException | IOException e) {
            System.err.println("[ReplayRecorder] write failed: " + e.getMessage());
            return null;
        }
    }

    private String toJson() {
        StringBuilder sb = new StringBuilder(Math.max(16_384, frames.size() * 512));
        sb.append("{\n");
        field(sb, 1, "schema", "mavis-hospital-replay-v1", true);
        field(sb, 1, "generatedAt", new Date(createdAt).toString(), true);
        sb.append(indent(1)).append("\"summary\": ");
        appendSummary(sb);
        sb.append(",\n");
        sb.append(indent(1)).append("\"level\": ");
        appendLevel(sb);
        sb.append(",\n");
        sb.append(indent(1)).append("\"frames\": [\n");
        for (int i = 0; i < frames.size(); i++) {
            if (i > 0) sb.append(",\n");
            Frame previous = i > 0 ? frames.get(i - 1) : null;
            appendFrame(sb, previous, frames.get(i), 2);
        }
        sb.append('\n').append(indent(1)).append("]\n");
        sb.append("}\n");
        return sb.toString();
    }

    private void appendSummary(StringBuilder sb) {
        State finalState = frames.isEmpty() ? null : frames.get(frames.size() - 1).state;
        sb.append("{\n");
        field(sb, 2, "outcome", outcome, true);
        numberField(sb, 2, "executedSteps", executedSteps, true);
        numberField(sb, 2, "plannedSteps", plannedSteps, true);
        numberField(sb, 2, "frames", frames.size(), true);
        numberField(sb, 2, "satisfiedBoxGoals", countSatisfiedBoxGoals(finalState), true);
        numberField(sb, 2, "totalBoxGoals", level.getAllBoxGoalPositions().size(), true);
        numberField(sb, 2, "events", countEvents(), false);
        sb.append(indent(1)).append('}');
    }

    private int countEvents() {
        int count = 0;
        for (int i = 1; i < frames.size(); i++) {
            count += deriveEvents(frames.get(i - 1), frames.get(i)).size();
        }
        count += planTrace.count();
        return count;
    }

    private int countSatisfiedBoxGoals(State state) {
        if (state == null) return 0;
        int count = 0;
        for (Position goal : level.getAllBoxGoalPositions()) {
            char want = level.getBoxGoal(goal);
            if (state.getBoxAt(goal) == want) count++;
        }
        return count;
    }

    private void appendLevel(StringBuilder sb) {
        sb.append("{\n");
        field(sb, 2, "name", level.getName(), true);
        numberField(sb, 2, "rows", level.getRows(), true);
        numberField(sb, 2, "cols", level.getCols(), true);

        sb.append(indent(2)).append("\"walls\": [");
        for (int r = 0; r < level.getRows(); r++) {
            if (r > 0) sb.append(", ");
            StringBuilder row = new StringBuilder(level.getCols());
            for (int c = 0; c < level.getCols(); c++) {
                row.append(level.isWall(r, c) ? '+' : ' ');
            }
            quoted(sb, row.toString());
        }
        sb.append("],\n");

        sb.append(indent(2)).append("\"boxGoals\": [");
        boolean first = true;
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                char goal = level.getBoxGoal(r, c);
                if (goal == '\0') continue;
                if (!first) sb.append(", ");
                sb.append("{\"type\":");
                quoted(sb, String.valueOf(goal));
                sb.append(",\"r\":").append(r).append(",\"c\":").append(c).append('}');
                first = false;
            }
        }
        sb.append("],\n");

        sb.append(indent(2)).append("\"agentGoals\": [");
        first = true;
        for (Map.Entry<Integer, Position> e : level.getAgentGoalPositionMap().entrySet().stream()
                .sorted(Map.Entry.comparingByKey()).toList()) {
            if (!first) sb.append(", ");
            Position p = e.getValue();
            sb.append("{\"agent\":").append(e.getKey())
                    .append(",\"r\":").append(p.row)
                    .append(",\"c\":").append(p.col).append('}');
            first = false;
        }
        sb.append("],\n");

        sb.append(indent(2)).append("\"agentColors\": {");
        first = true;
        for (Map.Entry<Integer, Color> e : level.getAgentColors().entrySet().stream()
                .sorted(Map.Entry.comparingByKey()).toList()) {
            if (!first) sb.append(", ");
            quoted(sb, String.valueOf(e.getKey()));
            sb.append(':');
            quoted(sb, e.getValue().name());
            first = false;
        }
        sb.append("},\n");

        sb.append(indent(2)).append("\"boxColors\": {");
        first = true;
        for (Map.Entry<Character, Color> e : level.getBoxColors().entrySet().stream()
                .sorted(Map.Entry.comparingByKey()).toList()) {
            if (!first) sb.append(", ");
            quoted(sb, String.valueOf(e.getKey()));
            sb.append(':');
            quoted(sb, e.getValue().name());
            first = false;
        }
        sb.append("}\n");
        sb.append(indent(1)).append('}');
    }

    private void appendFrame(StringBuilder sb, Frame previous, Frame frame, int depth) {
        sb.append(indent(depth)).append("{\n");
        numberField(sb, depth + 1, "t", frame.t, true);
        sb.append(indent(depth + 1)).append("\"actions\": [");
        if (frame.actions != null) {
            for (int i = 0; i < frame.actions.length; i++) {
                if (i > 0) sb.append(", ");
                quoted(sb, frame.actions[i] != null ? frame.actions[i].toServerString() : "NoOp");
            }
        }
        sb.append("],\n");
        sb.append(indent(depth + 1)).append("\"accepted\": [");
        if (frame.accepted != null) {
            for (int i = 0; i < frame.accepted.length; i++) {
                if (i > 0) sb.append(", ");
                sb.append(frame.accepted[i]);
            }
        }
        sb.append("],\n");
        appendAgents(sb, frame.state, depth + 1);
        sb.append(",\n");
        appendBoxes(sb, frame.state, depth + 1);
        List<Object> events = new ArrayList<>();
        events.addAll(deriveEvents(previous, frame));
        events.addAll(planTrace.eventsForFrame(frame.t));
        if (!events.isEmpty()) {
            sb.append(",\n");
            appendEvents(sb, events, depth + 1);
        }
        sb.append('\n').append(indent(depth)).append('}');
    }

    private List<StepEvent> deriveEvents(Frame previous, Frame frame) {
        List<StepEvent> events = new ArrayList<>();
        if (previous == null || frame.actions == null) return events;

        for (int agentId = 0; agentId < frame.actions.length; agentId++) {
            Action action = frame.actions[agentId] != null ? frame.actions[agentId] : Action.noOp();
            boolean accepted = frame.accepted != null
                    && agentId < frame.accepted.length
                    && frame.accepted[agentId];
            if (accepted && action.type == Action.ActionType.NOOP) {
                continue;
            }

            Position from = previous.state.getAgentPosition(agentId);
            Position to = frame.state.getAgentPosition(agentId);
            ActionIntent intent = actionIntent(previous.state, action, from);

            String actionText = action.toServerString();
            String title = "agent" + agentId + " " + actionText;
            String kind = accepted ? "agent-action" : "rejected-action";
            String severity = accepted ? "info" : "warning";
            String message = accepted
                    ? "Server accepted the action and this frame records the resulting state."
                    : "Server rejected the action; the effective action for this agent was NoOp.";

            events.add(new StepEvent(kind, severity, title, message, agentId, actionText,
                    accepted, from, to, intent.agentTo, intent.boxType, intent.boxFrom,
                    intent.boxTo));
        }
        return events;
    }

    private ActionIntent actionIntent(State state, Action action, Position from) {
        if (state == null || from == null || action == null) {
            return new ActionIntent(null, null, null, null);
        }
        return switch (action.type) {
            case MOVE -> new ActionIntent(from.move(action.agentDir), null, null, null);
            case PUSH -> {
                Position boxFrom = from.move(action.agentDir);
                Position boxTo = boxFrom.move(action.boxDir);
                Character boxType = state.hasBoxAt(boxFrom) ? state.getBoxAt(boxFrom) : null;
                yield new ActionIntent(boxFrom, boxType, boxFrom, boxTo);
            }
            case PULL -> {
                Position agentTo = from.move(action.agentDir);
                Position boxFrom = from.move(action.boxDir.opposite());
                Character boxType = state.hasBoxAt(boxFrom) ? state.getBoxAt(boxFrom) : null;
                yield new ActionIntent(agentTo, boxType, boxFrom, from);
            }
            case NOOP -> new ActionIntent(from, null, null, null);
        };
    }

    private void appendEvents(StringBuilder sb, List<Object> events, int depth) {
        sb.append(indent(depth)).append("\"events\": [\n");
        for (int i = 0; i < events.size(); i++) {
            if (i > 0) sb.append(",\n");
            Object event = events.get(i);
            if (event instanceof StepEvent stepEvent) {
                appendEvent(sb, stepEvent, depth + 1);
            } else if (event instanceof PlanTrace.Event planEvent) {
                appendEvent(sb, planEvent, depth + 1);
            }
        }
        sb.append('\n').append(indent(depth)).append(']');
    }

    private void appendEvent(StringBuilder sb, StepEvent event, int depth) {
        sb.append(indent(depth)).append("{\n");
        field(sb, depth + 1, "kind", event.kind, true);
        field(sb, depth + 1, "severity", event.severity, true);
        field(sb, depth + 1, "title", event.title, true);
        field(sb, depth + 1, "message", event.message, true);
        numberField(sb, depth + 1, "agentId", event.agentId, true);
        field(sb, depth + 1, "action", event.action, true);
        booleanField(sb, depth + 1, "accepted", event.accepted, true);
        positionField(sb, depth + 1, "from", event.from, true);
        positionField(sb, depth + 1, "to", event.to, event.intentAgentTo != null
                || event.boxType != null || event.boxFrom != null || event.boxTo != null);
        if (event.intentAgentTo != null) {
            positionField(sb, depth + 1, "attemptedTo", event.intentAgentTo,
                    event.boxType != null || event.boxFrom != null || event.boxTo != null);
        }
        if (event.boxType != null) {
            field(sb, depth + 1, "boxType", String.valueOf(event.boxType),
                    event.boxFrom != null || event.boxTo != null);
        }
        if (event.boxFrom != null) {
            positionField(sb, depth + 1, "boxFrom", event.boxFrom, event.boxTo != null);
        }
        if (event.boxTo != null) {
            positionField(sb, depth + 1, "boxTo", event.boxTo, false);
        }
        sb.append(indent(depth)).append('}');
    }

    private void appendEvent(StringBuilder sb, PlanTrace.Event event, int depth) {
        sb.append(indent(depth)).append("{\n");
        field(sb, depth + 1, "kind", event.kind(), true);
        field(sb, depth + 1, "severity", event.severity(), true);
        field(sb, depth + 1, "title", event.title(), true);
        field(sb, depth + 1, "message", event.message(), true);
        if (event.agentId() != null) numberField(sb, depth + 1, "agentId", event.agentId(), true);
        if (event.phase() != null) field(sb, depth + 1, "phase", event.phase(), true);
        if (event.subgoal() != null) field(sb, depth + 1, "subgoal", event.subgoal(), true);
        if (event.subgoalType() != null) field(sb, depth + 1, "subgoalType", event.subgoalType(), true);
        if (event.goal() != null) positionField(sb, depth + 1, "goal", event.goal(), true);
        if (event.boxType() != null) field(sb, depth + 1, "boxType", String.valueOf(event.boxType()), true);
        if (event.stepInSegment() != null) numberField(sb, depth + 1, "stepInSegment", event.stepInSegment(), true);
        if (event.segmentSteps() != null) numberField(sb, depth + 1, "segmentSteps", event.segmentSteps(), true);
        if (event.action() != null) field(sb, depth + 1, "action", event.action(), true);
        if (event.actualAction() != null) field(sb, depth + 1, "actualAction", event.actualAction(), true);
        if (event.reason() != null) field(sb, depth + 1, "reason", event.reason(), true);
        if (event.verdict() != null) field(sb, depth + 1, "verdict", event.verdict(), true);
        if (event.synthetic() != null) booleanField(sb, depth + 1, "synthetic", event.synthetic(), true);
        if (event.agentReachBefore() != null) numberField(sb, depth + 1, "agentReachBefore", event.agentReachBefore(), true);
        if (event.agentReachAfter() != null) numberField(sb, depth + 1, "agentReachAfter", event.agentReachAfter(), true);
        if (event.totalReachBefore() != null) numberField(sb, depth + 1, "totalReachBefore", event.totalReachBefore(), true);
        if (event.totalReachAfter() != null) numberField(sb, depth + 1, "totalReachAfter", event.totalReachAfter(), true);
        if (event.goalAdjBefore() != null) booleanField(sb, depth + 1, "goalAdjBefore", event.goalAdjBefore(), true);
        if (event.goalAdjAfter() != null) booleanField(sb, depth + 1, "goalAdjAfter", event.goalAdjAfter(), true);
        for (Map.Entry<String, String> detail : event.details().entrySet()) {
            if (detail.getKey() == null || detail.getValue() == null) continue;
            field(sb, depth + 1, detail.getKey(), detail.getValue(), true);
        }
        trimTrailingComma(sb);
        sb.append(indent(depth)).append('}');
    }

    private void appendAgents(StringBuilder sb, State state, int depth) {
        sb.append(indent(depth)).append("\"agents\": [");
        boolean first = true;
        for (int id = 0; id < state.getNumAgents(); id++) {
            Position p = state.getAgentPosition(id);
            if (p == null) continue;
            if (!first) sb.append(", ");
            sb.append("{\"id\":").append(id)
                    .append(",\"r\":").append(p.row)
                    .append(",\"c\":").append(p.col).append('}');
            first = false;
        }
        sb.append(']');
    }

    private void appendBoxes(StringBuilder sb, State state, int depth) {
        sb.append(indent(depth)).append("\"boxes\": [");
        List<Map.Entry<Position, Character>> boxes = new ArrayList<>(state.getBoxes().entrySet());
        boxes.sort(Comparator.<Map.Entry<Position, Character>>comparingInt(e -> e.getKey().row)
                .thenComparingInt(e -> e.getKey().col)
                .thenComparing(e -> e.getValue()));
        for (int i = 0; i < boxes.size(); i++) {
            if (i > 0) sb.append(", ");
            Map.Entry<Position, Character> e = boxes.get(i);
            Position p = e.getKey();
            sb.append("{\"type\":");
            quoted(sb, String.valueOf(e.getValue()));
            sb.append(",\"r\":").append(p.row).append(",\"c\":").append(p.col).append('}');
        }
        sb.append(']');
    }

    private void writeViewer(Path viewerDir) throws IOException {
        Files.createDirectories(viewerDir);
        writeStaticViewerFile(viewerDir, "index.html");
        writeStaticViewerFile(viewerDir, "viewer.css");
        writeStaticViewerFile(viewerDir, "viewer.js");
        writeStaticViewerFile(viewerDir, "favicon.ico");
        writeStaticViewerFile(viewerDir, "favicon.png");
        writeStaticViewerFile(viewerDir, "favicon-32.png");
        writeStaticViewerFile(viewerDir, "apple-touch-icon.png");
    }

    private void writeStaticViewerFile(Path viewerDir, String fileName) throws IOException {
        Path file = viewerDir.resolve(fileName);
        String resource = VIEWER_RESOURCE_DIR + fileName;
        try (InputStream input = ReplayRecorder.class.getClassLoader().getResourceAsStream(resource)) {
            if (input == null) {
                throw new IOException("Missing bundled replay viewer resource: " + resource);
            }
            Files.copy(input, file, StandardCopyOption.REPLACE_EXISTING);
        }
    }

    private void writeLatestReplayScript(Path viewerDir, String json) throws IOException {
        String safeJson = json.replace("</script", "<\\/script");
        Files.writeString(viewerDir.resolve("latest-replay.js"),
                "window.DEFAULT_REPLAY = " + safeJson + ";\n", StandardCharsets.UTF_8);
    }

    private static void field(StringBuilder sb, int depth, String name, String value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ");
        quoted(sb, value);
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

    private static void booleanField(StringBuilder sb, int depth, String name, boolean value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ").append(value);
        if (comma) sb.append(',');
        sb.append('\n');
    }

    private static void positionField(StringBuilder sb, int depth, String name, Position value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ");
        if (value == null) {
            sb.append("null");
        } else {
            sb.append("{\"r\":").append(value.row).append(",\"c\":").append(value.col).append('}');
        }
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

    private static String safeName(String name) {
        if (name == null || name.isBlank()) return "level";
        return name.replaceAll("[^A-Za-z0-9._-]", "_");
    }

    record Frame(int t, Action[] actions, boolean[] accepted, State state) {
        static Frame initial(State state) {
            return new Frame(0, new Action[0], new boolean[0], state);
        }
    }

    private void writeDiagnosticDigests(Path replayPath) {
        try {
            State finalState = frames.isEmpty() ? null : frames.get(frames.size() - 1).state;
            lastDiagnosticFiles = new DiagnosticDigestWriter().write(
                    replayPath,
                    level,
                    frames,
                    planTrace,
                    outcome,
                    executedSteps,
                    plannedSteps,
                    countSatisfiedBoxGoals(finalState),
                    level.getAllBoxGoalPositions().size());
        } catch (IOException e) {
            lastDiagnosticFiles = List.of();
            System.err.println("[ReplayRecorder] diagnostic digest write failed: " + e.getMessage());
        }
    }

    private static void trimTrailingComma(StringBuilder sb) {
        int comma = sb.length() - 2;
        if (comma >= 0 && sb.charAt(comma) == ',') {
            sb.deleteCharAt(comma);
        }
    }

    private record ActionIntent(Position agentTo, Character boxType, Position boxFrom, Position boxTo) {
    }

    private record StepEvent(String kind, String severity, String title, String message,
                             int agentId, String action, boolean accepted,
                             Position from, Position to, Position intentAgentTo,
                             Character boxType, Position boxFrom, Position boxTo) {
    }

}
