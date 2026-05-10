package mapf.planning.diag;

import mapf.domain.Action;
import mapf.domain.Color;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
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
            Path replayDir = root.resolve("replays");
            Path viewerDir = root.resolve("replay-viewer");
            Files.createDirectories(replayDir);
            writeViewer(viewerDir);

            String levelName = safeName(level.getName());
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
        boolean refresh = "1".equals(System.getenv("MAVIS_REPLAY_VIEWER_REFRESH"));
        writeStaticViewerFile(viewerDir.resolve("index.html"), INDEX_HTML, refresh);
        writeStaticViewerFile(viewerDir.resolve("viewer.css"), VIEWER_CSS, refresh);
        writeStaticViewerFile(viewerDir.resolve("viewer.js"), VIEWER_JS, refresh);
    }

    private void writeStaticViewerFile(Path file, String body, boolean refresh) throws IOException {
        if (refresh || !Files.exists(file)) {
            Files.writeString(file, body, StandardCharsets.UTF_8);
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

    private static final String INDEX_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MAvis Replay Reviewer</title>
  <link rel="stylesheet" href="viewer.css">
</head>
<body>
  <main class="app">
    <aside class="panel">
      <h1>Replay Reviewer</h1>
      <label class="filePick">
        <input id="fileInput" type="file" accept="application/json,.json">
        Load replay JSON
      </label>
      <div id="dropZone" class="dropZone">Drop replay JSON here</div>
      <div class="meta" id="meta">No replay loaded</div>
      <div class="controls">
        <button id="prevBtn" title="Previous step">&lt;</button>
        <button id="playBtn" title="Play or pause">Play</button>
        <button id="nextBtn" title="Next step">&gt;</button>
      </div>
      <label class="checkLine"><input id="autoPlayInput" type="checkbox"> Auto-play after load</label>
      <label class="searchLabel">Playback delay (ms)
        <input id="speedInput" type="number" min="20" value="120">
      </label>
      <input id="stepSlider" type="range" min="0" max="0" value="0">
      <div class="stepLine">
        <input id="stepInput" type="number" min="0" value="0">
        <span id="stepText">/ 0</span>
      </div>
      <label class="searchLabel">Track object
        <input id="trackInput" placeholder="agent7 or boxG">
      </label>
      <label class="checkLine"><input id="reachInput" type="checkbox"> Show selected agent reachability</label>
      <label class="searchLabel">Highlight coordinate
        <input id="coordInput" placeholder="28,3">
      </label>
      <label class="searchLabel">Reachability target
        <input id="targetInput" placeholder="28,3">
      </label>
      <button id="scanBtn" class="wideBtn">Scan reachability regression</button>
      <section>
        <h2>Suspicious events</h2>
        <div id="events" class="events"></div>
      </section>
      <section>
        <h2>Actions</h2>
        <div id="actions" class="actions"></div>
      </section>
    </aside>
    <section class="boardWrap">
      <div class="toolbar">
        <span id="hoverInfo">Java coordinates: row, col</span>
      </div>
      <div id="board" class="board"></div>
    </section>
  </main>
  <script src="latest-replay.js"></script>
  <script src="viewer.js"></script>
</body>
</html>
""";

    private static final String VIEWER_CSS = """
:root {
  --cell: 18px;
  --axis: 24px;
  --wall: #20242a;
  --floor: #f8fafc;
  --grid: #d5dbe3;
  --goal: #f5e7b8;
  --text: #101418;
}

* { box-sizing: border-box; }
body {
  margin: 0;
  font-family: Inter, Segoe UI, Arial, sans-serif;
  color: var(--text);
  background: #eef2f6;
}
.app {
  display: grid;
  grid-template-columns: 320px 1fr;
  min-height: 100vh;
}
.panel {
  padding: 16px;
  border-right: 1px solid #cbd5df;
  background: #ffffff;
  overflow: auto;
}
h1 { margin: 0 0 14px; font-size: 20px; }
h2 { margin: 16px 0 8px; font-size: 14px; }
.filePick {
  display: block;
  padding: 10px;
  border: 1px solid #9fb1c5;
  border-radius: 6px;
  cursor: pointer;
  text-align: center;
}
.filePick input { display: none; }
.dropZone {
  margin-top: 10px;
  padding: 16px;
  border: 1px dashed #9fb1c5;
  border-radius: 6px;
  text-align: center;
  color: #506070;
}
.dropZone.drag { background: #edf6ff; border-color: #3178c6; }
.meta { margin-top: 12px; font-size: 13px; line-height: 1.45; }
.controls { display: flex; gap: 8px; margin-top: 14px; }
button {
  height: 32px;
  border: 1px solid #9fb1c5;
  border-radius: 5px;
  background: #f8fafc;
  cursor: pointer;
}
.controls button { flex: 1; }
#stepSlider { width: 100%; margin: 14px 0 8px; }
.stepLine { display: flex; align-items: center; gap: 8px; font-size: 13px; }
#stepInput { width: 86px; }
.searchLabel { display: block; margin-top: 14px; font-size: 13px; }
.searchLabel input { width: 100%; margin-top: 6px; padding: 7px; }
.checkLine {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 10px;
  font-size: 13px;
}
.wideBtn { width: 100%; margin-top: 12px; }
.actions {
  display: grid;
  gap: 4px;
  font-family: Consolas, monospace;
  font-size: 12px;
}
.events {
  display: grid;
  gap: 6px;
  font-size: 12px;
}
.eventRow {
  padding: 7px;
  border-radius: 5px;
  background: #fff7ed;
  border: 1px solid #fed7aa;
  cursor: pointer;
}
.eventRow:hover { background: #ffedd5; }
.actionRow {
  display: grid;
  grid-template-columns: 28px 1fr;
  gap: 6px;
  padding: 4px 6px;
  border-radius: 4px;
  background: #f4f6f8;
}
.actionRow.rejected { background: #ffe9e7; color: #9b1c13; }
.boardWrap { padding: 14px; overflow: auto; }
.toolbar { height: 28px; font-size: 13px; color: #405060; }
.board {
  display: grid;
  grid-auto-rows: var(--cell);
  width: max-content;
  border: 1px solid #b7c2ce;
  background: #b7c2ce;
}
.axis, .corner {
  display: flex;
  align-items: center;
  justify-content: center;
  width: var(--cell);
  height: var(--cell);
  font-family: Consolas, monospace;
  font-size: 10px;
  background: #e2e8f0;
  color: #425466;
  user-select: none;
}
.rowAxis { width: var(--axis); }
.cell {
  position: relative;
  width: var(--cell);
  height: var(--cell);
  display: flex;
  align-items: center;
  justify-content: center;
  border: 1px solid var(--grid);
  background: var(--floor);
  font-family: Consolas, monospace;
  font-weight: 700;
  font-size: 12px;
}
.cell.wall { background: var(--wall); border-color: #2e343c; }
.cell.goal { background: var(--goal); }
.cell.changed { outline: 2px solid #f59e0b; z-index: 2; }
.cell.track { outline: 2px solid #2563eb; z-index: 3; }
.cell.coord { outline: 2px solid #dc2626; z-index: 4; }
.cell.reachable { box-shadow: inset 0 0 0 999px rgba(34, 197, 94, 0.18); }
.cell.unreachableTarget { outline: 3px solid #b91c1c; z-index: 5; }
.cell.trail::after {
  content: "";
  position: absolute;
  inset: 5px;
  border-radius: 50%;
  background: rgba(37, 99, 235, 0.25);
}
.token {
  min-width: 15px;
  height: 15px;
  border-radius: 4px;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  border: 1px solid rgba(0,0,0,0.35);
  color: #111;
  line-height: 1;
}
.goalMark {
  position: absolute;
  right: 1px;
  bottom: 0;
  font-size: 9px;
  color: rgba(0,0,0,0.5);
}
""";

    private static final String VIEWER_JS = """
const colors = {
  BLUE: '#93c5fd', RED: '#fca5a5', CYAN: '#67e8f9', PURPLE: '#c4b5fd',
  GREEN: '#86efac', ORANGE: '#fdba74', PINK: '#f9a8d4', GREY: '#cbd5e1',
  LIGHTBLUE: '#bae6fd', BROWN: '#d6a46d', DEFAULT: '#e5e7eb'
};

let replay = null;
let step = 0;
let timer = null;
const els = {
  board: document.getElementById('board'),
  meta: document.getElementById('meta'),
  fileInput: document.getElementById('fileInput'),
  dropZone: document.getElementById('dropZone'),
  playBtn: document.getElementById('playBtn'),
  prevBtn: document.getElementById('prevBtn'),
  nextBtn: document.getElementById('nextBtn'),
  slider: document.getElementById('stepSlider'),
  stepInput: document.getElementById('stepInput'),
  stepText: document.getElementById('stepText'),
  trackInput: document.getElementById('trackInput'),
  coordInput: document.getElementById('coordInput'),
  targetInput: document.getElementById('targetInput'),
  reachInput: document.getElementById('reachInput'),
  autoPlayInput: document.getElementById('autoPlayInput'),
  speedInput: document.getElementById('speedInput'),
  scanBtn: document.getElementById('scanBtn'),
  actions: document.getElementById('actions'),
  events: document.getElementById('events'),
  hoverInfo: document.getElementById('hoverInfo')
};

els.fileInput.addEventListener('change', e => {
  const file = e.target.files && e.target.files[0];
  if (file) loadFile(file);
});
['dragenter', 'dragover'].forEach(type => els.dropZone.addEventListener(type, e => {
  e.preventDefault(); els.dropZone.classList.add('drag');
}));
['dragleave', 'drop'].forEach(type => els.dropZone.addEventListener(type, e => {
  e.preventDefault(); els.dropZone.classList.remove('drag');
}));
els.dropZone.addEventListener('drop', e => {
  const file = e.dataTransfer.files && e.dataTransfer.files[0];
  if (file) loadFile(file);
});
els.playBtn.addEventListener('click', togglePlay);
els.prevBtn.addEventListener('click', () => setStep(step - 1));
els.nextBtn.addEventListener('click', () => setStep(step + 1));
els.slider.addEventListener('input', e => setStep(Number(e.target.value)));
els.stepInput.addEventListener('change', e => setStep(Number(e.target.value)));
els.trackInput.addEventListener('input', render);
els.coordInput.addEventListener('input', render);
els.targetInput.addEventListener('input', render);
els.reachInput.addEventListener('change', render);
els.scanBtn.addEventListener('click', scanReachabilityRegression);

async function loadFile(file) {
  const text = await file.text();
  loadReplayData(JSON.parse(text));
}

function loadReplayData(data) {
  replay = data;
  step = 0;
  els.slider.max = replay.frames.length - 1;
  els.stepInput.max = replay.frames.length - 1;
  const s = replay.summary || {};
  els.meta.innerHTML = `<b>${escapeHtml(replay.level.name)}</b><br>${replay.level.rows}x${replay.level.cols}, ${replay.frames.length} frames<br>${escapeHtml(s.outcome || 'unknown')} · ${s.executedSteps ?? 0} steps · ${s.satisfiedBoxGoals ?? 0}/${s.totalBoxGoals ?? 0} box goals<br>${escapeHtml(replay.generatedAt || '')}`;
  buildBoard();
  render();
  if (els.autoPlayInput.checked) startPlay();
}

function buildBoard() {
  const { rows, cols } = replay.level;
  els.board.style.gridTemplateColumns = `var(--axis) repeat(${cols}, var(--cell))`;
  els.board.innerHTML = '';
  els.board.appendChild(div('corner', ''));
  for (let c = 0; c < cols; c++) els.board.appendChild(div('axis', String(c)));
  for (let r = 0; r < rows; r++) {
    els.board.appendChild(div('axis rowAxis', String(r)));
    for (let c = 0; c < cols; c++) {
      const cell = div('cell', '');
      cell.dataset.r = r;
      cell.dataset.c = c;
      cell.title = `(${r},${c})`;
      cell.addEventListener('mouseenter', () => {
        els.hoverInfo.textContent = `Java coordinates: (${r}, ${c})`;
      });
      els.board.appendChild(cell);
    }
  }
}

function render() {
  if (!replay) return;
  step = Math.max(0, Math.min(step, replay.frames.length - 1));
  const frame = replay.frames[step];
  const prev = replay.frames[Math.max(0, step - 1)];
  els.slider.value = step;
  els.stepInput.value = step;
  els.stepText.textContent = `/ ${replay.frames.length - 1}`;

  const goals = goalMap();
  const agents = new Map(frame.agents.map(a => [`${a.r},${a.c}`, a]));
  const boxes = new Map(frame.boxes.map(b => [`${b.r},${b.c}`, b]));
  const changed = changedCells(prev, frame);
  const track = trackedCells();
  const trail = trailCells();
  const coord = parseCoord(els.coordInput.value);
  const target = parseCoord(els.targetInput.value);
  const reachable = els.reachInput.checked ? reachableCells(frame, selectedAgentId()) : new Set();

  for (const cell of els.board.querySelectorAll('.cell')) {
    const r = Number(cell.dataset.r), c = Number(cell.dataset.c);
    const key = `${r},${c}`;
    cell.className = 'cell';
    cell.innerHTML = '';
    if (replay.level.walls[r][c] === '+') cell.classList.add('wall');
    if (goals.has(key)) {
      cell.classList.add('goal');
      const mark = document.createElement('span');
      mark.className = 'goalMark';
      mark.textContent = goals.get(key);
      cell.appendChild(mark);
    }
    if (trail.has(key)) cell.classList.add('trail');
    if (reachable.has(key)) cell.classList.add('reachable');
    if (changed.has(key)) cell.classList.add('changed');
    if (track.has(key)) cell.classList.add('track');
    if (coord && coord.r === r && coord.c === c) cell.classList.add('coord');
    if (target && target.r === r && target.c === c && !reachable.has(key)) cell.classList.add('unreachableTarget');
    if (boxes.has(key)) cell.appendChild(token(boxes.get(key).type, boxColor(boxes.get(key).type)));
    if (agents.has(key)) cell.appendChild(token(String(agents.get(key).id), agentColor(agents.get(key).id)));
  }
  renderActions(frame);
}

function renderActions(frame) {
  els.actions.innerHTML = '';
  const actions = frame.actions || [];
  const accepted = frame.accepted || [];
  for (let i = 0; i < actions.length; i++) {
    const row = div(`actionRow ${accepted[i] ? '' : 'rejected'}`, '');
    row.innerHTML = `<span>${i}</span><span>${escapeHtml(actions[i])}</span>`;
    els.actions.appendChild(row);
  }
}

function setStep(next) {
  step = Math.max(0, Math.min(next, replay ? replay.frames.length - 1 : 0));
  render();
}

function togglePlay() {
  if (!replay) return;
  if (timer) {
    clearInterval(timer); timer = null; els.playBtn.textContent = 'Play'; return;
  }
  startPlay();
}

function startPlay() {
  if (!replay || timer) return;
  els.playBtn.textContent = 'Pause';
  timer = setInterval(() => {
    if (step >= replay.frames.length - 1) { togglePlay(); return; }
    setStep(step + 1);
  }, Math.max(20, Number(els.speedInput.value) || 120));
}

function goalMap() {
  const out = new Map();
  for (const g of replay.level.boxGoals) out.set(`${g.r},${g.c}`, g.type);
  for (const g of replay.level.agentGoals) out.set(`${g.r},${g.c}`, String(g.agent));
  return out;
}

function changedCells(prev, cur) {
  const out = new Set();
  if (!prev || prev === cur) return out;
  const addAgentMoves = () => {
    const before = new Map(prev.agents.map(x => [x.id, x]));
    for (const x of cur.agents) {
      const y = before.get(x.id);
      if (!y || y.r !== x.r || y.c !== x.c) {
        out.add(`${x.r},${x.c}`);
        if (y) out.add(`${y.r},${y.c}`);
      }
    }
  };
  const addBoxMoves = () => {
    const before = new Set(prev.boxes.map(x => `${x.type}@${x.r},${x.c}`));
    const after = new Set(cur.boxes.map(x => `${x.type}@${x.r},${x.c}`));
    for (const x of cur.boxes) if (!before.has(`${x.type}@${x.r},${x.c}`)) out.add(`${x.r},${x.c}`);
    for (const x of prev.boxes) if (!after.has(`${x.type}@${x.r},${x.c}`)) out.add(`${x.r},${x.c}`);
  };
  addAgentMoves();
  addBoxMoves();
  return out;
}

function trackedCells() {
  const q = els.trackInput.value.trim().toLowerCase();
  const out = new Set();
  if (!q || !replay) return out;
  const frame = replay.frames[step];
  if (q.startsWith('agent')) {
    const id = Number(q.replace('agent', ''));
    for (const a of frame.agents) if (a.id === id) out.add(`${a.r},${a.c}`);
  } else if (q.startsWith('box')) {
    const type = q.replace('box', '').toUpperCase();
    for (const b of frame.boxes) if (b.type === type) out.add(`${b.r},${b.c}`);
  }
  return out;
}

function trailCells() {
  const q = els.trackInput.value.trim().toLowerCase();
  const out = new Set();
  if (!q || !replay) return out;
  for (let i = 0; i <= step; i++) {
    const frame = replay.frames[i];
    if (q.startsWith('agent')) {
      const id = Number(q.replace('agent', ''));
      for (const a of frame.agents) if (a.id === id) out.add(`${a.r},${a.c}`);
    } else if (q.startsWith('box')) {
      const type = q.replace('box', '').toUpperCase();
      for (const b of frame.boxes) if (b.type === type) out.add(`${b.r},${b.c}`);
    }
  }
  return out;
}

function selectedAgentId() {
  const q = els.trackInput.value.trim().toLowerCase();
  if (!q.startsWith('agent')) return null;
  const id = Number(q.replace('agent', ''));
  return Number.isFinite(id) ? id : null;
}

function parseCoord(text) {
  const m = String(text || '').trim().match(/^\\(?\\s*(\\d+)\\s*,\\s*(\\d+)\\s*\\)?$/);
  if (!m) return null;
  return { r: Number(m[1]), c: Number(m[2]) };
}

function reachableCells(frame, agentId) {
  const out = new Set();
  if (agentId == null || !frame) return out;
  const agent = frame.agents.find(a => a.id === agentId);
  if (!agent) return out;
  const blocked = new Set(frame.boxes.map(b => `${b.r},${b.c}`));
  for (const a of frame.agents) {
    if (a.id !== agentId) blocked.add(`${a.r},${a.c}`);
  }
  const q = [{ r: agent.r, c: agent.c }];
  out.add(`${agent.r},${agent.c}`);
  for (let head = 0; head < q.length; head++) {
    const p = q[head];
    for (const d of [[1,0],[-1,0],[0,1],[0,-1]]) {
      const r = p.r + d[0], c = p.c + d[1];
      const key = `${r},${c}`;
      if (out.has(key) || isWall(r, c) || blocked.has(key)) continue;
      out.add(key);
      q.push({ r, c });
    }
  }
  return out;
}

function isWall(r, c) {
  return r < 0 || c < 0 || r >= replay.level.rows || c >= replay.level.cols
      || replay.level.walls[r][c] === '+';
}

function scanReachabilityRegression() {
  if (!replay) return;
  const agentId = selectedAgentId();
  const target = parseCoord(els.targetInput.value) || parseCoord(els.coordInput.value);
  els.events.innerHTML = '';
  if (agentId == null || !target) {
    els.events.textContent = 'Set Track object to agentN and target to r,c.';
    return;
  }
  const targetKey = `${target.r},${target.c}`;
  const rows = [];
  let prevCan = reachableCells(replay.frames[0], agentId).has(targetKey);
  for (let i = 1; i < replay.frames.length; i++) {
    const can = reachableCells(replay.frames[i], agentId).has(targetKey);
    if (prevCan && !can) {
      rows.push(describeRegression(i, agentId, targetKey));
    }
    prevCan = can;
  }
  if (rows.length === 0) {
    els.events.textContent = `No reachable -> unreachable transition found for agent${agentId} to (${target.r},${target.c}).`;
    return;
  }
  for (const row of rows.slice(0, 20)) {
    const el = div('eventRow', '');
    el.innerHTML = `<b>step ${row.step}</b> agent${agentId} lost target ${escapeHtml(targetKey)}<br>${escapeHtml(row.reason)}<br>${escapeHtml(row.actions)}`;
    el.addEventListener('click', () => setStep(row.step));
    els.events.appendChild(el);
  }
}

function describeRegression(stepIdx, agentId, targetKey) {
  const prev = replay.frames[stepIdx - 1];
  const cur = replay.frames[stepIdx];
  const moved = movedObjects(prev, cur);
  const goalInfo = goalMap();
  const interesting = moved
    .filter(x => x.kind === 'box' || x.kind === 'agent')
    .map(x => `${x.kind}${x.id} ${x.from}->${x.to}${goalInfo.has(x.to) ? ' on-goal ' + goalInfo.get(x.to) : ''}`);
  const actions = (cur.actions || []).map((a, i) => `${i}:${a}${cur.accepted && cur.accepted[i] ? '' : ' rejected'}`).join(' | ');
  return {
    step: stepIdx,
    reason: interesting.length ? interesting.join('; ') : 'No moved object identified; inspect changed cells.',
    actions
  };
}

function movedObjects(prev, cur) {
  const out = [];
  const prevAgents = new Map(prev.agents.map(a => [a.id, a]));
  for (const a of cur.agents) {
    const p = prevAgents.get(a.id);
    if (!p || p.r !== a.r || p.c !== a.c) {
      out.push({ kind: 'agent', id: a.id, from: p ? `${p.r},${p.c}` : '?', to: `${a.r},${a.c}` });
    }
  }
  const prevBoxes = new Set(prev.boxes.map(b => `${b.type}@${b.r},${b.c}`));
  const curBoxes = new Set(cur.boxes.map(b => `${b.type}@${b.r},${b.c}`));
  for (const b of cur.boxes) {
    const here = `${b.type}@${b.r},${b.c}`;
    if (prevBoxes.has(here)) continue;
    const candidates = prev.boxes.filter(x => x.type === b.type && !curBoxes.has(`${x.type}@${x.r},${x.c}`));
    const from = candidates.length ? `${candidates[0].r},${candidates[0].c}` : '?';
    out.push({ kind: 'box', id: b.type, from, to: `${b.r},${b.c}` });
  }
  return out;
}

function token(text, color) {
  const el = document.createElement('span');
  el.className = 'token';
  el.textContent = text;
  el.style.background = color;
  return el;
}
function boxColor(type) { return colors[replay.level.boxColors[type]] || colors.DEFAULT; }
function agentColor(id) { return colors[replay.level.agentColors[String(id)]] || colors.DEFAULT; }
function div(cls, text) { const el = document.createElement('div'); el.className = cls; el.textContent = text; return el; }
function escapeHtml(s) { return String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])); }

if (window.DEFAULT_REPLAY) {
  loadReplayData(window.DEFAULT_REPLAY);
}
""";
}
