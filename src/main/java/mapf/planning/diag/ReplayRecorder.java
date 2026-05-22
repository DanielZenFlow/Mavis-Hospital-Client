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
    private final long createdAt = System.currentTimeMillis();
    private String outcome = "unknown";
    private int executedSteps = 0;
    private int plannedSteps = 0;

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
            appendFrame(sb, frames.get(i), 2);
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
        numberField(sb, 2, "totalBoxGoals", level.getAllBoxGoalPositions().size(), false);
        sb.append(indent(1)).append('}');
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

    private void appendFrame(StringBuilder sb, Frame frame, int depth) {
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
        sb.append('\n').append(indent(depth)).append('}');
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

    private record Frame(int t, Action[] actions, boolean[] accepted, State state) {
        static Frame initial(State state) {
            return new Frame(0, new Action[0], new boolean[0], state);
        }
    }

}
