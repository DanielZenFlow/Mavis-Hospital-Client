package mapf.planning.diag;

import mapf.domain.Action;
import mapf.domain.Color;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;
import mapf.planning.SearchConfig;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

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
    private final SearchConfig searchConfig;
    private final List<Frame> frames = new ArrayList<>();
    private PlanTrace planTrace = new PlanTrace();
    private final long createdAt = System.currentTimeMillis();
    private String outcome = "unknown";
    private int executedSteps = 0;
    private int plannedSteps = 0;
    private long effectiveServerTimeoutMs = -1;
    private long planningTimeoutMs = -1;
    private List<Path> lastDiagnosticFiles = List.of();

    private ReplayRecorder(Level level, State initialState) {
        this(level, initialState, null);
    }

    private ReplayRecorder(Level level, State initialState, SearchConfig searchConfig) {
        this.level = level;
        this.searchConfig = searchConfig;
        frames.add(Frame.initial(initialState));
    }

    public static ReplayRecorder start(Level level, State initialState) {
        return new ReplayRecorder(level, initialState);
    }

    public static ReplayRecorder start(Level level, State initialState, SearchConfig searchConfig) {
        return new ReplayRecorder(level, initialState, searchConfig);
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

    public void setTimingBudget(long effectiveServerTimeoutMs, long planningTimeoutMs) {
        this.effectiveServerTimeoutMs = effectiveServerTimeoutMs;
        this.planningTimeoutMs = planningTimeoutMs;
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
        sb.append(indent(1)).append("\"diagnostics\": ");
        appendDiagnostics(sb);
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
        numberField(sb, 2, "events", countEvents(), true);
        numberField(sb, 2, "derivedActionEvents", countDerivedActionEvents(), false);
        sb.append(indent(1)).append('}');
    }

    private void appendDiagnostics(StringBuilder sb) {
        sb.append("{\n");
        sb.append(indent(2)).append("\"runContext\": ");
        appendRunContext(sb);
        sb.append(",\n");
        sb.append(indent(2)).append("\"portfolioAttempts\": ");
        appendPortfolioAttempts(sb, planTrace.portfolioAttempts(), 2);
        sb.append(",\n");
        sb.append(indent(2)).append("\"diagnosticFocus\": ");
        appendDiagnosticFocus(sb);
        sb.append('\n').append(indent(1)).append('}');
    }

    private void appendRunContext(StringBuilder sb) {
        sb.append("{\n");
        field(sb, 3, "schema", "mavis-hospital-run-context-v1", true);
        field(sb, 3, "recorder", "ReplayRecorder", true);
        sb.append(indent(3)).append("\"codeVersion\": ");
        appendCodeVersion(sb);
        sb.append(",\n");
        sb.append(indent(3)).append("\"levelFingerprint\": ");
        appendLevelFingerprint(sb);
        sb.append(",\n");
        sb.append(indent(3)).append("\"searchConfig\": ");
        appendSearchConfigSnapshot(sb);
        sb.append(",\n");
        field(sb, 3, "javaVersion", System.getProperty("java.version", ""), true);
        field(sb, 3, "os", System.getProperty("os.name", "") + " " + System.getProperty("os.version", ""), true);
        field(sb, 3, "mavisTimeoutMsEnv", System.getenv("MAVIS_TIMEOUT_MS"), true);
        numberField(sb, 3, "logLevel", SearchConfig.LOG_LEVEL, true);
        numberField(sb, 3, "configuredTimeoutMs", searchConfig != null
                ? searchConfig.getTimeoutMs() : SearchConfig.DEFAULT_TIMEOUT_MS, true);
        numberField(sb, 3, "planningTimeoutMs", planningTimeoutMs, true);
        numberField(sb, 3, "maxActions", SearchConfig.MAX_ACTIONS, true);
        numberField(sb, 3, "defaultMaxStates", SearchConfig.DEFAULT_MAX_STATES, true);
        numberField(sb, 3, "maxBspBudget", SearchConfig.MAX_BSP_BUDGET, true);
        numberField(sb, 3, "maxBspBudgetLarge", SearchConfig.MAX_BSP_BUDGET_LARGE, true);
        numberField(sb, 3, "framesRecorded", frames.size(), true);
        numberField(sb, 3, "plannerEventsRecorded", planTrace.count(), false);
        sb.append(indent(2)).append('}');
    }

    private void appendCodeVersion(StringBuilder sb) {
        GitInfo git = readGitInfo();
        GitStatus gitStatus = readGitStatus();
        RuntimeArtifact artifact = runtimeArtifactFingerprint();
        sb.append("{\n");
        field(sb, 4, "schema", "mavis-hospital-code-version-v1", true);
        field(sb, 4, "gitBranch", git.branch, true);
        field(sb, 4, "gitCommit", git.commit, true);
        field(sb, 4, "gitCommitShort", shortCommit(git.commit), true);
        field(sb, 4, "gitSource", git.source, true);
        booleanField(sb, 4, "gitDirty", gitStatus.dirty, true);
        numberField(sb, 4, "gitChangedFileCount", gitStatus.changedFileCount, true);
        field(sb, 4, "gitChangedFilesSample", gitStatus.changedFilesSample, true);
        field(sb, 4, "gitStatusSource", gitStatus.source, true);
        field(sb, 4, "runtimeArtifact", artifact.location, true);
        field(sb, 4, "runtimeArtifactKind", artifact.kind, true);
        field(sb, 4, "runtimeArtifactSha256", artifact.sha256, false);
        sb.append(indent(3)).append('}');
    }

    private GitInfo readGitInfo() {
        String envCommit = firstNonBlank(
                System.getenv("MAVIS_GIT_COMMIT"),
                System.getenv("GIT_COMMIT"),
                System.getenv("SOURCE_VERSION"));
        String envBranch = firstNonBlank(
                System.getenv("MAVIS_GIT_BRANCH"),
                System.getenv("GIT_BRANCH"),
                System.getenv("BRANCH_NAME"));

        try {
            Path gitDir = resolveGitDir(Paths.get(".git"));
            if (gitDir != null) {
                Path headPath = gitDir.resolve("HEAD");
                String head = Files.exists(headPath)
                        ? Files.readString(headPath, StandardCharsets.UTF_8).trim()
                        : "";
                if (head.startsWith("ref:")) {
                    String ref = head.substring(4).trim();
                    String branch = ref.startsWith("refs/heads/")
                            ? ref.substring("refs/heads/".length())
                            : ref;
                    String commit = readGitRef(gitDir, ref);
                    return new GitInfo(firstNonBlank(envBranch, branch),
                            firstNonBlank(envCommit, commit), "git-dir");
                }
                if (!head.isEmpty()) {
                    return new GitInfo(firstNonBlank(envBranch, "detached"),
                            firstNonBlank(envCommit, head), "git-dir");
                }
            }
        } catch (RuntimeException | IOException ignored) {
            // Version metadata must never prevent replay writing.
        }

        String source = !envCommit.isEmpty() || !envBranch.isEmpty() ? "environment" : "unavailable";
        return new GitInfo(envBranch, envCommit, source);
    }

    private Path resolveGitDir(Path dotGit) throws IOException {
        if (Files.isDirectory(dotGit)) return dotGit;
        if (!Files.isRegularFile(dotGit)) return null;
        String content = Files.readString(dotGit, StandardCharsets.UTF_8).trim();
        if (!content.startsWith("gitdir:")) return null;
        Path gitDir = Paths.get(content.substring("gitdir:".length()).trim());
        return gitDir.isAbsolute() ? gitDir : dotGit.getParent().resolve(gitDir).normalize();
    }

    private String readGitRef(Path gitDir, String ref) throws IOException {
        Path refPath = gitDir.resolve(ref);
        if (Files.exists(refPath)) {
            return Files.readString(refPath, StandardCharsets.UTF_8).trim();
        }
        Path packedRefs = gitDir.resolve("packed-refs");
        if (!Files.exists(packedRefs)) return "";
        for (String line : Files.readAllLines(packedRefs, StandardCharsets.UTF_8)) {
            String trimmed = line.trim();
            if (trimmed.isEmpty() || trimmed.startsWith("#") || trimmed.startsWith("^")) continue;
            String[] parts = trimmed.split("\\s+", 2);
            if (parts.length == 2 && parts[1].equals(ref)) return parts[0];
        }
        return "";
    }

    private GitStatus readGitStatus() {
        try {
            Process process = new ProcessBuilder(
                    "git", "status", "--porcelain=v1", "--untracked-files=normal")
                    .redirectErrorStream(true)
                    .start();
            if (!process.waitFor(1_000, TimeUnit.MILLISECONDS)) {
                process.destroyForcibly();
                return new GitStatus(false, -1, "", "git-status-timeout");
            }
            String output;
            try (InputStream input = process.getInputStream()) {
                output = new String(input.readAllBytes(), StandardCharsets.UTF_8).trim();
            }
            if (process.exitValue() != 0) {
                return new GitStatus(false, -1, firstLines(output, 3), "git-status-exit-" + process.exitValue());
            }
            if (output.isEmpty()) {
                return new GitStatus(false, 0, "", "git-status");
            }
            String[] lines = output.split("\\R+");
            return new GitStatus(true, lines.length, firstLines(output, 12), "git-status");
        } catch (Exception ignored) {
            return new GitStatus(false, -1, "", "unavailable");
        }
    }

    private void appendLevelFingerprint(StringBuilder sb) {
        State initialState = frames.isEmpty() ? null : frames.get(0).state;
        sb.append("{\n");
        field(sb, 4, "schema", "mavis-hospital-level-fingerprint-v1", true);
        field(sb, 4, "levelName", level.getName(), true);
        numberField(sb, 4, "rows", level.getRows(), true);
        numberField(sb, 4, "cols", level.getCols(), true);
        numberField(sb, 4, "wallCount", countWalls(), true);
        numberField(sb, 4, "boxGoalCount", level.getAllBoxGoalPositions().size(), true);
        numberField(sb, 4, "agentGoalCount", level.getAgentGoalPositionMap().size(), true);
        numberField(sb, 4, "initialAgentCount", initialState != null ? initialState.getNumAgents() : 0, true);
        numberField(sb, 4, "initialBoxCount", initialState != null ? initialState.getBoxes().size() : 0, true);
        field(sb, 4, "levelStaticSha256", levelStaticFingerprint(), true);
        field(sb, 4, "initialStateSha256", stateFingerprint(initialState), false);
        sb.append(indent(3)).append('}');
    }

    private void appendSearchConfigSnapshot(StringBuilder sb) {
        sb.append("{\n");
        field(sb, 4, "schema", "mavis-hospital-search-config-v1", true);
        numberField(sb, 4, "defaultTimeoutMs", SearchConfig.DEFAULT_TIMEOUT_MS, true);
        numberField(sb, 4, "defaultMaxStates", SearchConfig.DEFAULT_MAX_STATES, true);
        numberField(sb, 4, "jointAstarAgentThreshold", SearchConfig.JOINT_ASTAR_AGENT_THRESHOLD, true);
        doubleField(sb, 4, "defaultAstarWeight", SearchConfig.DEFAULT_ASTAR_WEIGHT, true);
        doubleField(sb, 4, "fallbackAstarWeight", SearchConfig.FALLBACK_ASTAR_WEIGHT, true);
        numberField(sb, 4, "firstAttemptTimeoutMs", SearchConfig.FIRST_ATTEMPT_TIMEOUT_MS, true);
        numberField(sb, 4, "fallbackTimeoutMs", SearchConfig.FALLBACK_TIMEOUT_MS, true);
        numberField(sb, 4, "maxActions", SearchConfig.MAX_ACTIONS, true);
        numberField(sb, 4, "maxAgents", SearchConfig.MAX_AGENTS, true);
        numberField(sb, 4, "maxBoxTypes", SearchConfig.MAX_BOX_TYPES, true);
        numberField(sb, 4, "boxNotAtGoalWorkScore", SearchConfig.BOX_NOT_AT_GOAL_WORK_SCORE, true);
        numberField(sb, 4, "progressLogInterval", SearchConfig.PROGRESS_LOG_INTERVAL, true);
        numberField(sb, 4, "maxStatesPerSubgoal", SearchConfig.MAX_STATES_PER_SUBGOAL, true);
        numberField(sb, 4, "maxReorderAttempts", SearchConfig.MAX_REORDER_ATTEMPTS, true);
        numberField(sb, 4, "stuckIterationsBeforeClearing", SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING, true);
        numberField(sb, 4, "maxStatesPerClearing", SearchConfig.MAX_STATES_PER_CLEARING, true);
        numberField(sb, 4, "maxClearingAttempts", SearchConfig.MAX_CLEARING_ATTEMPTS, true);
        numberField(sb, 4, "maxParkingDistance", SearchConfig.MAX_PARKING_DISTANCE, true);
        numberField(sb, 4, "randomSeed", SearchConfig.RANDOM_SEED, true);
        numberField(sb, 4, "maxStuckIterations", SearchConfig.MAX_STUCK_ITERATIONS, true);
        numberField(sb, 4, "stuckLogInterval", SearchConfig.STUCK_LOG_INTERVAL, true);
        numberField(sb, 4, "dependencyCheckThreshold", SearchConfig.DEPENDENCY_CHECK_THRESHOLD, true);
        numberField(sb, 4, "minBspBudget", SearchConfig.MIN_BSP_BUDGET, true);
        numberField(sb, 4, "maxBspBudget", SearchConfig.MAX_BSP_BUDGET, true);
        numberField(sb, 4, "maxBspBudgetLarge", SearchConfig.MAX_BSP_BUDGET_LARGE, true);
        numberField(sb, 4, "bspBudgetPerDistance", SearchConfig.BSP_BUDGET_PER_DISTANCE, true);
        numberField(sb, 4, "logLevel", SearchConfig.LOG_LEVEL, true);
        numberField(sb, 4, "configuredTimeoutMs", searchConfig != null
                ? searchConfig.getTimeoutMs() : SearchConfig.DEFAULT_TIMEOUT_MS, true);
        numberField(sb, 4, "configuredMaxStates", searchConfig != null
                ? searchConfig.getMaxStates() : SearchConfig.DEFAULT_MAX_STATES, true);
        doubleField(sb, 4, "configuredAstarWeight", searchConfig != null
                ? searchConfig.getAstarWeight() : SearchConfig.DEFAULT_ASTAR_WEIGHT, true);
        booleanField(sb, 4, "configuredUseGreedyFallback", searchConfig == null
                || searchConfig.isUseGreedyFallback(), true);
        numberField(sb, 4, "effectiveServerTimeoutMs", effectiveServerTimeoutMs, true);
        numberField(sb, 4, "planningTimeoutMs", planningTimeoutMs, true);
        field(sb, 4, "envMavisTimeoutMs", System.getenv("MAVIS_TIMEOUT_MS"), true);
        field(sb, 4, "envMavisLogLevel", System.getenv("MAVIS_LOG_LEVEL"), true);
        field(sb, 4, "envMavisLogRejected", System.getenv("MAVIS_LOG_REJECTED"), true);
        field(sb, 4, "envMavisLogInvalidActions", System.getenv("MAVIS_LOG_INVALID_ACTIONS"), true);
        field(sb, 4, "envMavisFailureSnapshot", System.getenv("MAVIS_FAILURE_SNAPSHOT"), false);
        sb.append(indent(3)).append('}');
    }

    private RuntimeArtifact runtimeArtifactFingerprint() {
        try {
            java.security.CodeSource source = ReplayRecorder.class
                    .getProtectionDomain().getCodeSource();
            if (source == null || source.getLocation() == null) {
                return new RuntimeArtifact("", "unknown", "");
            }
            Path path = Paths.get(source.getLocation().toURI());
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            String kind;
            if (Files.isDirectory(path)) {
                kind = "directory";
                List<Path> files;
                try (java.util.stream.Stream<Path> stream = Files.walk(path)) {
                    files = stream.filter(Files::isRegularFile)
                            .sorted(Comparator.comparing(p -> path.relativize(p).toString()))
                            .toList();
                }
                for (Path file : files) {
                    digest.update(path.relativize(file).toString().replace('\\', '/')
                            .getBytes(StandardCharsets.UTF_8));
                    digest.update((byte) 0);
                    digest.update(Files.readAllBytes(file));
                    digest.update((byte) 0);
                }
            } else if (Files.isRegularFile(path)) {
                kind = "file";
                digest.update(Files.readAllBytes(path));
            } else {
                kind = "unknown";
            }
            return new RuntimeArtifact(path.toString(), kind, hex(digest.digest()));
        } catch (Exception ignored) {
            return new RuntimeArtifact("", "unavailable", "");
        }
    }

    private String levelStaticFingerprint() {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            updateDigest(digest, "name=" + firstNonBlank(level.getName()));
            updateDigest(digest, "size=" + level.getRows() + "x" + level.getCols());
            for (int r = 0; r < level.getRows(); r++) {
                StringBuilder walls = new StringBuilder(level.getCols());
                StringBuilder boxGoals = new StringBuilder(level.getCols());
                StringBuilder agentGoals = new StringBuilder(level.getCols());
                for (int c = 0; c < level.getCols(); c++) {
                    walls.append(level.isWall(r, c) ? '+' : ' ');
                    char bg = level.getBoxGoal(r, c);
                    boxGoals.append(bg == '\0' ? '.' : bg);
                    int ag = level.getAgentGoal(r, c);
                    agentGoals.append(ag < 0 ? '.' : (char) ('0' + ag));
                }
                updateDigest(digest, "walls:" + walls);
                updateDigest(digest, "boxGoals:" + boxGoals);
                updateDigest(digest, "agentGoals:" + agentGoals);
            }
            for (Map.Entry<Integer, Color> e : level.getAgentColors().entrySet().stream()
                    .sorted(Map.Entry.comparingByKey()).toList()) {
                updateDigest(digest, "agentColor:" + e.getKey() + "=" + e.getValue().name());
            }
            for (Map.Entry<Character, Color> e : level.getBoxColors().entrySet().stream()
                    .sorted(Map.Entry.comparingByKey()).toList()) {
                updateDigest(digest, "boxColor:" + e.getKey() + "=" + e.getValue().name());
            }
            return hex(digest.digest());
        } catch (Exception ignored) {
            return "";
        }
    }

    private String stateFingerprint(State state) {
        if (state == null) return "";
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            for (int id = 0; id < state.getNumAgents(); id++) {
                updateDigest(digest, "agent:" + id + "=" + positionText(state.getAgentPosition(id)));
            }
            List<Map.Entry<Position, Character>> boxes = new ArrayList<>(state.getBoxes().entrySet());
            boxes.sort(Comparator.<Map.Entry<Position, Character>>comparingInt(e -> e.getKey().row)
                    .thenComparingInt(e -> e.getKey().col)
                    .thenComparing(e -> e.getValue()));
            for (Map.Entry<Position, Character> e : boxes) {
                updateDigest(digest, "box:" + e.getValue() + "=" + positionText(e.getKey()));
            }
            return hex(digest.digest());
        } catch (Exception ignored) {
            return "";
        }
    }

    private void updateDigest(MessageDigest digest, String value) {
        digest.update(firstNonBlank(value).getBytes(StandardCharsets.UTF_8));
        digest.update((byte) '\n');
    }

    private int countWalls() {
        int count = 0;
        for (int r = 0; r < level.getRows(); r++) {
            for (int c = 0; c < level.getCols(); c++) {
                if (level.isWall(r, c)) count++;
            }
        }
        return count;
    }

    private void appendDiagnosticFocus(StringBuilder sb) {
        List<PlanTrace.Event> events = plannerEvents();
        List<PlanTrace.Event> focusEvents = new ArrayList<>();
        Map<String, Integer> kindCounts = new LinkedHashMap<>();
        Map<String, Integer> reasonCounts = new LinkedHashMap<>();
        Map<String, Integer> subgoalCounts = new LinkedHashMap<>();
        for (PlanTrace.Event event : events) {
            increment(kindCounts, event.kind());
            if (event.reason() != null) increment(reasonCounts, event.reason());
            if (event.subgoal() != null) increment(subgoalCounts, event.subgoal());
            if (isFocusEvent(event)) focusEvents.add(event);
        }
        PlanTrace.Event first = focusEvents.isEmpty() ? null : focusEvents.get(0);
        PlanTrace.Event last = focusEvents.isEmpty() ? null : focusEvents.get(focusEvents.size() - 1);
        String headline = switch (outcome) {
            case "solved" -> "Solved replay; focus on portfolio route and committed transactions.";
            case "partial" -> "Partial replay; inspect first failed planner focus and best portfolio attempt.";
            case "no-plan" -> "No plan; inspect box selection, BSP exhaustion, and support failures.";
            default -> "Replay diagnostic focus.";
        };
        String detail = "plannerEvents=" + events.size()
                + ", focusEvents=" + focusEvents.size()
                + ", portfolioAttempts=" + planTrace.portfolioAttempts().size();

        sb.append("{\n");
        field(sb, 3, "schema", "mavis-hospital-diagnostic-focus-v1", true);
        field(sb, 3, "headline", headline, true);
        field(sb, 3, "detail", detail, true);
        numberField(sb, 3, "focusEventCount", focusEvents.size(), true);
        field(sb, 3, "firstFocus", eventSummary(first), true);
        field(sb, 3, "finalFocus", eventSummary(last), true);
        sb.append(indent(3)).append("\"topEventKinds\": ");
        appendCountEntries(sb, kindCounts, 8, 3);
        sb.append(",\n");
        sb.append(indent(3)).append("\"topReasons\": ");
        appendCountEntries(sb, reasonCounts, 8, 3);
        sb.append(",\n");
        sb.append(indent(3)).append("\"topSubgoals\": ");
        appendCountEntries(sb, subgoalCounts, 8, 3);
        sb.append('\n');
        sb.append(indent(2)).append('}');
    }

    private List<PlanTrace.Event> plannerEvents() {
        List<PlanTrace.Event> events = new ArrayList<>();
        int maxFrame = frames.isEmpty() ? executedSteps : frames.get(frames.size() - 1).t;
        for (int frame = 0; frame <= maxFrame; frame++) {
            events.addAll(planTrace.eventsForFrame(frame));
        }
        return events;
    }

    private boolean isFocusEvent(PlanTrace.Event event) {
        if (event == null) return false;
        String severity = firstNonBlank(event.severity()).toLowerCase(Locale.ROOT);
        String kind = firstNonBlank(event.kind()).toLowerCase(Locale.ROOT);
        String verdict = firstNonBlank(event.verdict()).toLowerCase(Locale.ROOT);
        return severity.contains("warn") || severity.contains("error") || severity.contains("fail")
                || kind.contains("failed") || kind.contains("exhausted") || kind.contains("rollback")
                || kind.contains("reject") || kind.contains("transaction") || kind.contains("candidate")
                || verdict.contains("failed") || verdict.contains("reject") || verdict.contains("rollback");
    }

    private String eventSummary(PlanTrace.Event event) {
        if (event == null) return "";
        StringBuilder sb = new StringBuilder();
        sb.append("step ").append(event.frame())
                .append(" ").append(firstNonBlank(event.kind(), "event"));
        String verdict = firstNonBlank(event.verdict());
        if (!verdict.isEmpty()) sb.append(" ").append(verdict);
        String subgoal = firstNonBlank(event.subgoal());
        if (!subgoal.isEmpty()) sb.append(" ").append(subgoal);
        String reason = firstNonBlank(event.reason());
        if (!reason.isEmpty()) sb.append(" reason=").append(reason);
        return sb.toString();
    }

    private void appendCountEntries(StringBuilder sb, Map<String, Integer> counts, int limit, int depth) {
        if (counts == null || counts.isEmpty()) {
            sb.append("[]");
            return;
        }
        List<Map.Entry<String, Integer>> entries = counts.entrySet().stream()
                .sorted((a, b) -> {
                    int byCount = Integer.compare(b.getValue(), a.getValue());
                    return byCount != 0 ? byCount : a.getKey().compareTo(b.getKey());
                })
                .limit(limit)
                .toList();
        sb.append("[\n");
        for (int i = 0; i < entries.size(); i++) {
            if (i > 0) sb.append(",\n");
            Map.Entry<String, Integer> entry = entries.get(i);
            sb.append(indent(depth + 1)).append("{\"key\":");
            quoted(sb, entry.getKey());
            sb.append(",\"count\":").append(entry.getValue()).append('}');
        }
        sb.append('\n').append(indent(depth)).append(']');
    }

    private void increment(Map<String, Integer> counts, String key) {
        if (key == null || key.isBlank()) return;
        counts.merge(key, 1, Integer::sum);
    }

    private String firstLines(String text, int maxLines) {
        if (text == null || text.isBlank()) return "";
        String[] lines = text.trim().split("\\R+");
        List<String> selected = new ArrayList<>();
        for (int i = 0; i < Math.min(maxLines, lines.length); i++) {
            selected.add(lines[i]);
        }
        if (lines.length > maxLines) selected.add("+" + (lines.length - maxLines) + " more");
        return String.join("; ", selected);
    }

    private static String firstNonBlank(String... values) {
        if (values == null) return "";
        for (String value : values) {
            if (value != null && !value.trim().isEmpty()) return value.trim();
        }
        return "";
    }

    private static String shortCommit(String commit) {
        return commit != null && commit.length() >= 12 ? commit.substring(0, 12) : firstNonBlank(commit);
    }

    private static String hex(byte[] bytes) {
        StringBuilder sb = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) {
            sb.append(String.format("%02x", b));
        }
        return sb.toString();
    }

    private record GitInfo(String branch, String commit, String source) {}

    private record GitStatus(boolean dirty, int changedFileCount, String changedFilesSample, String source) {}

    private record RuntimeArtifact(String location, String kind, String sha256) {}

    private void appendPortfolioAttempts(StringBuilder sb,
                                         List<PlanTrace.PortfolioAttempt> attempts,
                                         int depth) {
        sb.append("[\n");
        for (int i = 0; i < attempts.size(); i++) {
            if (i > 0) sb.append(",\n");
            PlanTrace.PortfolioAttempt attempt = attempts.get(i);
            sb.append(indent(depth + 1)).append("{\n");
            numberField(sb, depth + 2, "ordinal", attempt.ordinal(), true);
            field(sb, depth + 2, "phase", attempt.phase(), true);
            field(sb, depth + 2, "label", attempt.label(), true);
            field(sb, depth + 2, "strategy", attempt.strategy(), true);
            field(sb, depth + 2, "orderingMode", attempt.orderingMode(), true);
            numberField(sb, depth + 2, "randomSeed", attempt.randomSeed(), true);
            numberField(sb, depth + 2, "durationMs", attempt.durationMs(), true);
            booleanField(sb, depth + 2, "success", attempt.success(), true);
            numberField(sb, depth + 2, "planSteps", attempt.planSteps(), true);
            numberField(sb, depth + 2, "unsatCount", attempt.unsatCount(), true);
            field(sb, depth + 2, "failedSubgoal", attempt.failedSubgoal(), true);
            field(sb, depth + 2, "failureKind", attempt.failureKind(), true);
            numberField(sb, depth + 2, "reliefCount", attempt.reliefCount(), true);
            numberField(sb, depth + 2, "suspendedCount", attempt.suspendedCount(), true);
            numberField(sb, depth + 2, "finalSatisfiedGoals", attempt.finalSatisfiedGoals(), true);
            numberField(sb, depth + 2, "finalTotalGoals", attempt.finalTotalGoals(), true);
            numberField(sb, depth + 2, "finalSatisfiedBoxGoals", attempt.finalSatisfiedBoxGoals(), true);
            numberField(sb, depth + 2, "finalTotalBoxGoals", attempt.finalTotalBoxGoals(), true);
            field(sb, depth + 2, "finalUnsatisfiedGoalsSample", attempt.finalUnsatisfiedGoalsSample(), true);
            field(sb, depth + 2, "finalStateHash", attempt.finalStateHash(), false);
            sb.append(indent(depth + 1)).append('}');
        }
        sb.append('\n').append(indent(depth)).append(']');
    }

    private int countEvents() {
        int count = 0;
        for (int i = 1; i < frames.size(); i++) {
            count += deriveEvents(frames.get(i - 1), frames.get(i)).size();
        }
        count += planTrace.count();
        return count;
    }

    private int countDerivedActionEvents() {
        int count = 0;
        for (int i = 1; i < frames.size(); i++) {
            Frame frame = frames.get(i);
            if (frame.actions == null) continue;
            for (int agentId = 0; agentId < frame.actions.length; agentId++) {
                Action action = frame.actions[agentId] != null ? frame.actions[agentId] : Action.noOp();
                boolean accepted = frame.accepted != null
                        && agentId < frame.accepted.length
                        && frame.accepted[agentId];
                if (accepted && action.type != Action.ActionType.NOOP) count++;
            }
        }
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
            if (accepted) {
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

    private static void numberField(StringBuilder sb, int depth, String name, long value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ").append(value);
        if (comma) sb.append(',');
        sb.append('\n');
    }

    private static void doubleField(StringBuilder sb, int depth, String name, double value, boolean comma) {
        sb.append(indent(depth));
        quoted(sb, name);
        sb.append(": ").append(Double.toString(value));
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

    private static String positionText(Position position) {
        return position == null ? "" : "(" + position.row + "," + position.col + ")";
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
