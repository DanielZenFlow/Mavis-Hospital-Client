package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.SearchConfig;
import mapf.planning.SearchStrategy;
import mapf.planning.analysis.CrossColorBarrierAnalyzer;
import mapf.planning.analysis.DependencyAnalyzer;
import mapf.planning.analysis.GoalTransitAnalyzer;
import mapf.planning.analysis.LevelAnalyzer;
import mapf.planning.coordination.ConflictDetector;
import mapf.planning.coordination.DeadlockResolver;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.signal.FailureReport;
import mapf.planning.spacetime.ReservationTable;
import mapf.planning.synthesis.BlockerReliefSynthesizer;
import java.util.*;

/**
 * Priority-based planning strategy with subgoal decomposition.
 * Per ARCHITECTURE.md: "Independent Planning + Conflict Resolution"
 */
public class PriorityPlanningStrategy implements SearchStrategy {

    private final Heuristic heuristic;
    private final SearchConfig config;
    private long timeoutMs;
    private int maxStates;
    private Random random = new Random(SearchConfig.RANDOM_SEED);

    // Core helper classes (SRP: each handles one responsibility)
    private final SubgoalManager subgoalManager;
    private final ConflictResolver conflictResolver;
    private final BoxSearchPlanner boxSearchPlanner;
    private final GreedyPlanner greedyPlanner;
    private final PlanMerger planMerger;
    private final PathAnalyzer pathAnalyzer;
    private final AgentCoordinator agentCoordinator;
    private final DeadlockBreaker deadlockBreaker;
    private final DeadlockResolver deadlockResolver = new DeadlockResolver();
    private final ConflictDetector jointActionValidator = new ConflictDetector();

    /** Tracks displaced boxes to avoid infinite loops. */
    private Set<String> displacementHistory = new HashSet<>();
    
    /** Counts displacement attempts; forces CBS after MAX_DISPLACEMENT_ATTEMPTS. */
    private int displacementAttempts = 0;
    private static final int MAX_DISPLACEMENT_ATTEMPTS = 3;
    
    /** 
     * Goal-level cycle detection: tracks how many times each goal position has been
     * completed. If a goal is completed > MAX_GOAL_COMPLETIONS times, PP is cycling
     * (push to goal → pull off → push back → ...). This supplements displacementHistory
     * which only tracks box-level displacement events and misses goal-level cycles.
     */
    private Map<Position, Integer> goalCompletionCount = new HashMap<>();
    private static final int MAX_GOAL_COMPLETIONS = 3;
    
    /** Tracks agent goals that have been completed at least once, to detect phantom progress. */
    private Set<Position> completedAgentGoals = new HashSet<>();;
    
    /**
     * Tracks completed goals that were intentionally displaced by tryRecovery.
     * These positions remain in completedBoxGoals (to satisfy dependency checks)
     * but are excluded from the frozen set (so BSP can path through them).
     * Cleared when another subgoal makes genuine progress.
     */
    private Set<Position> displacedGoals = new HashSet<>();
    
    /**
     * Borrow-and-return: goals displaced by the most recent tryPathClearing call.
     * After clearing+BSP succeeds, these are re-planned back to their goals.
     * Max MAX_CLEARING_DISPLACEMENTS allowed per clearing operation.
     */
    private List<Position> lastClearingDisplacedGoals = new ArrayList<>();
    private Map<Position, Position> lastBorrowedGoalBoxPositions = new HashMap<>();
    private static final int MAX_CLEARING_DISPLACEMENTS = 4;
    
    /** Pre-computed goal execution order from LevelAnalyzer (optional). */
    private List<Position> precomputedGoalOrder = null;
    
    /** Goal dependency graph from LevelAnalyzer: goal → set of goals it depends on. */
    private Map<Position, Set<Position>> goalDependsOn = Collections.emptyMap();
    
    /** Immovable boxes (treated as walls in pathfinding). */
    private Set<Position> immovableBoxes = Collections.emptySet();

    /**
     * P0b: goal positions to demote to the back of the subgoal order.
     * Populated by PortfolioController from prior attempts' FailureReport.lastAttemptedSubgoal:
     * if attempt N got stuck on goal G, attempt N+1 tries G last (other goals first may
     * unblock G physically). Pure ordering hint — no goals are removed.
     */
    private Set<Position> deprioritizedGoals = Collections.emptySet();

    /**
     * P4 (conflict-driven PP): goals to PROMOTE to the front of the order.
     * Populated by PortfolioController from prior FailureReport blockers (goals that
     * the failed subgoal logically depends on but were still unsatisfied at failure).
     * Per claudeopus47.txt §3.2.2: subgoal-level reorder is the cheapest CBS-lite repair.
     * Complementary to deprioritizedGoals (failed goal goes LAST, its blockers go FIRST).
     */
    private Set<Position> prioritizedGoals = Collections.emptySet();

    /**
     * F2 (NAMO blocker-relief co-suspension): box-goal positions to TEMPORARILY
     * remove from the subgoal queue for the duration of this PP attempt.
     * Populated by PortfolioController when a blocker-relief subgoal moves a box
     * away from where it would naturally settle on its original goal. Without this,
     * the planner re-pushes the box from P_temp back to the original goal,
     * undoing the relief and re-creating the geometric blockage — a push-pull
     * thrash that prevents the originally-blocked agent from ever reaching its goal.
     *
     * <p>Suspension is per-attempt: each new attempt starts fresh. The next attempt
     * can re-suspend (or not) based on whether NAMO synthesizer still emits reliefs
     * against the replayed state.
     */
    private Set<Position> suspendedBoxGoals = Collections.emptySet();

    /**
     * P1: synthetic escape subgoals to PREPEND to the order (highest priority).
     * Produced by EscapeSubgoalSynthesizer when LevelFeatures.hasCircularDependency=true:
     * a 2-cycle (A,B) where box-on-A blocks goal-B is broken by first moving box-on-A
     * to a P_temp parking position. Per qanda.txt 3 / claudeopus47.txt 3.2.2.
     */
    private List<Subgoal> escapeSubgoals = Collections.emptyList();

    /**
     * P2: goal positions of escape subgoals — derived index for fast O(1) lookup
     * in planSubgoal() to decide whether to try IW(1) as Round 0.
     */
    private Set<Position> escapeGoalPositions = Collections.emptySet();

    /**
     * P2 / P4b: Iterated Width(1) planner (lazy init).
     * Used for two paths: (a) escape subgoals (P2, Round 0), and
     * (b) find-route fallback for moderate-distance non-escape subgoals
     * (P4b, Round 1.5 — see planSubgoal ~L3700).
     * See also IW1_FINDROUTE_{MIN,MAX}_MANHATTAN below.
     */
    private IW1Planner iw1Planner = null;

    /**
     * P4b (subtask classifier): box-to-goal Manhattan distance gate for trying IW(1)
     * on non-escape subgoals (IW(1) FIND-ROUTE FALLBACK per claudeopus47 §1.2.2 / §3.2).
     * Below MIN: A* with Manhattan is trivially fast — IW(1) is wasteful.
     * Above MAX: IW(1) state explosion likely; trust BSP/A*+TrueDistance.
     * Used in planSubgoal ~L3700 (P4b branch).
     * Per qanda.txt §3.3 / §4.4 (IW(1) sweet spot for moderate find-route paths).
     */
    private static final int IW1_FINDROUTE_MIN_MANHATTAN = 10;
    private static final int IW1_FINDROUTE_MAX_MANHATTAN = 35;

    // === Diagnostics (reset per search() call) ===
    private int diagIw1Invoked = 0;
    private int diagIw1Succeeded = 0;
    private int diagPlanSubgoalNull = 0;
    private int diagPathClearingRescued = 0;
    private static final int REPEATED_LOG_PRINT_LIMIT = 2;
    private final Map<String, Integer> repeatedLogCounts = new LinkedHashMap<>();

    // Box oscillation tracking: detects boxes that LEAVE a position and RETURN to it
    // across subgoal completions. Smoking-gun for NAMO↔PP thrashing.
    // Memory: O(boxTypes * snapshots * boxesPerType) — typically <2KB even for ISO.
    private final java.util.Map<Character, java.util.List<java.util.List<Position>>> boxPositionHistory = new java.util.LinkedHashMap<>();
    private final java.util.List<String> subgoalLabelHistory = new java.util.ArrayList<>();

    /**
     * Cache of barrier clearing orders that were found non-extractable.
     * Prevents the infinite loop where dynamic barrier re-detection keeps
     * finding the same non-extractable barrier every stuck iteration.
     * Reset when genuine progress is made (completedBoxGoals changes).
     */
    private Set<List<Position>> skippedBarrierClearingOrders = new HashSet<>();

    /**
     * Permanent barrier failure cache: maps clearingAgent → position SETS of barriers
     * where that specific agent couldn't reach deeper boxes (structural maze constraint).
     * Per-agent because a different agent may approach from a different side.
     * Unlike skippedBarrierClearingOrders, this is NEVER cleared on progress
     * because the failure is structural (maze geometry), not state-dependent.
     */
    private Map<Integer, Set<Set<Position>>> permanentlyFailedBarriers = new HashMap<>();

    /**
     * Counter for dynamic barrier re-detection rounds (in the stuck recovery loop).
     * Limited to prevent wasting hundreds of steps on incomplete barrier clearing
     * that doesn't produce additional goal solves.
     */
    private int dynamicBarrierRounds = 0;
    private static final int MAX_DYNAMIC_BARRIER_ROUNDS = 5;

    /**
     * TRAP blacklist: goal positions where wouldTrapAgent returned true.
     * Within a single PP run, once a goal triggers TRAP rollback, it won't be
     * re-attempted (different ordering modes will reset this).
     */
    private Set<Position> trapBlacklist = new HashSet<>();

    /**
     * Dynamic L3 soft-deferral: a real goal that is theoretically prerequisite
     * in the static dependency graph, but was just proven unavailable in the
     * current state after blocker relief and path clearing both failed.
     *
     * Treating such a goal as a hard prerequisite can lock PP into retrying the
     * same blocked root forever. Soft-deferral lets dependent goals make progress
     * first, while the deferred goal stays in the queue and is retried once the
     * world changes.
     */
    private Set<Position> deferredBlockedGoals = new HashSet<>();

    /**
     * Synthetic box targets (P_temp relief/escape cells) rejected in this PP run
     * because they did not satisfy their concrete relief certificate. These are
     * not real Hospital box goals; accepting them without releasing the blocker
     * they were created for creates NAMO oscillation.
     */
    private Set<Position> syntheticReliefBlacklist = new HashSet<>();

    /**
     * Failed task-conditioned relief attempts in one PP run. A key is
     * task goal + blocker start + temporary parking cell. This keeps the
     * local-clearing loop from repeating the same impossible move.
     */
    private final Set<String> taskReliefNogoods = new HashSet<>();
    private final Set<String> sealStagingNogoods = new HashSet<>();
    private final Map<String, Set<Position>> taskSupportForbidden = new HashMap<>();
    private static final int MAX_TASK_RELIEF_MOVES = 8;
    private static final int MAX_SUPPORT_CHAIN_DEPTH = 4;

    /**
     * REGRESS blacklist: counts how many times each completed goal has been
     * disturbed (by any subgoal). After MAX_REGRESS_PER_GOAL disturbances,
     * that completed goal is marked as "unprotectable" and removed from
     * completedBoxGoals, allowing future paths to go through it.
     */
    private Map<Position, Integer> regressDisturbCount = new HashMap<>();
    private static final int MAX_REGRESS_PER_GOAL = 3;

    /** Timestamp of last successful subgoal execution, for early termination. */
    private long lastProgressTime = 0;
    private static final double NORMAL_EARLY_EXIT_FRACTION = 0.35;
    private static final double OPEN_TRANSACTION_EARLY_EXIT_FRACTION = 0.85;

    /**
     * P0a (cheapest path, see qanda.txt §5.2 / claudeopus47.txt §3.2.2):
     * Structured failure signal for the most recent search() call.
     * Producers (this class) populate it at every non-success exit; consumers
     * (PortfolioController for now, SubgoalManager later in P0b) read it via
     * getLastFailureReport(). Null when last call returned a full solution.
     */
    private FailureReport lastFailureReport = null;

    /** Tracks which subgoal tryExecuteSubgoals last touched, for failure reporting. */
    private Subgoal lastAttemptedSubgoalForReport = null;
    private FailureReport.Cause lastFailureCauseForReport = FailureReport.Cause.UNKNOWN;
    private final Set<Position> lastBlockedGoalsForReport = new LinkedHashSet<>();
    private final Set<Position> lastBlockedPositionsForReport = new LinkedHashSet<>();

    /** Read-only access to the most recent failure signal. May be null. */
    public FailureReport getLastFailureReport() {
        return lastFailureReport;
    }

    /** Read-only access to completed box goals (for diagnostic snapshots). */
    public Set<Position> getCompletedBoxGoals() {
        return java.util.Collections.unmodifiableSet(completedBoxGoals);
    }

    /** Read-only access to suspended TRANSIT goals (for diagnostic snapshots). */
    public Set<Position> getSuspendedTransitGoals() {
        return java.util.Collections.unmodifiableSet(suspendedTransitGoals);
    }

    /** Read-only access to GoalTransitAnalyzer profiles (for diagnostic snapshots). */
    public Map<Position, GoalTransitAnalyzer.GoalProfile> getTransitProfiles() {
        return java.util.Collections.unmodifiableMap(transitProfiles);
    }
    
    /** Effective max BSP budget after level-size adaptation. 
     *  Large levels (freeSpaces > 500) use a lower cap to prevent OOM. */
    private int effectiveMaxBspBudget = SearchConfig.MAX_BSP_BUDGET;
    
    /**
     * Ordering mode for subgoal execution. Different modes explore different
     * orderings, which is the most effective "retry knob" for PP.
     */
    public enum OrderingMode {
        /** Use LevelAnalyzer's topological sort (dependency-first). Default. */
        TOPOLOGICAL,
        /** Reverse of topological sort. Breaks deadlocks when normal order fails. */
        REVERSE_TOPOLOGICAL,
        /** Sort by box-to-goal distance, nearest first. Good for low-dependency levels. */
        DISTANCE_GREEDY,
        /** Sort by box-to-goal distance, farthest first. Critical for spiral topologies
         *  where outer goals must be filled before inner goals block narrow corridors. */
        DISTANCE_FARTHEST,
        /** Random shuffle. Last-resort diversification. */
        RANDOM
    }
    
    /** Current ordering mode for subgoal sorting. */
    private OrderingMode orderingMode = OrderingMode.TOPOLOGICAL;
    
    /** Completed box goals - treated as permanent obstacles per MAPF standard. */
    private Set<Position> completedBoxGoals = new HashSet<>();
    /** GoalTransitAnalyzer profiles — classified once per search(), used for Steps 2-4. */
    private Map<Position, GoalTransitAnalyzer.GoalProfile> transitProfiles = Collections.emptyMap();
    /** TRANSIT goals temporarily suspended (displaced by NAMO; will be filled last). */
    private Set<Position> suspendedTransitGoals = new HashSet<>();

    /** Space-time reservation table for collision avoidance. */
    private ReservationTable reservationTable = new ReservationTable();
    
    /** Current global time step for space-time planning. */
    private int globalTimeStep = 0;
    
    /** Flag: was the last progress from re-solving a previously-completed agent goal? */
    private boolean lastProgressWasPhantom = false;
    
    /** Maps agent ID to connected component ID. Agents in different components can execute in parallel. */
    private Map<Integer, Integer> agentComponentId = Collections.emptyMap();
    
    /** Tracks subgoals being executed via stored plans for independent agents. */
    private Map<Integer, Subgoal> storedPlanSubgoals = new HashMap<>();

    // Logging helpers
    private void logMinimal(String msg) {
        if (SearchConfig.isMinimal()) System.err.println(msg);
    }
    private void logNormal(String msg) {
        if (SearchConfig.isNormal()) System.err.println(msg);
    }
    private void logVerbose(String msg) {
        if (SearchConfig.isVerbose()) System.err.println(msg);
    }

    private void logNormalRepeated(String key, String msg) {
        if (!SearchConfig.isNormal()) return;
        int count = repeatedLogCounts.merge(key, 1, Integer::sum);
        if (count <= REPEATED_LOG_PRINT_LIMIT) {
            System.err.println(msg);
        }
    }

    private void printRepeatedLogSummary(String outcome) {
        if (!SearchConfig.isNormal() || repeatedLogCounts.isEmpty()) return;
        List<Map.Entry<String, Integer>> repeated = new ArrayList<>();
        for (Map.Entry<String, Integer> entry : repeatedLogCounts.entrySet()) {
            if (entry.getValue() > REPEATED_LOG_PRINT_LIMIT) {
                repeated.add(entry);
            }
        }
        if (repeated.isEmpty()) return;
        repeated.sort((a, b) -> Integer.compare(b.getValue(), a.getValue()));
        int cap = Math.min(8, repeated.size());
        System.err.println("[PP][LOG-SUMMARY][" + outcome + "] suppressed repeated diagnostics (showing top "
                + cap + "/" + repeated.size() + ")");
        for (int i = 0; i < cap; i++) {
            Map.Entry<String, Integer> entry = repeated.get(i);
            int suppressed = entry.getValue() - REPEATED_LOG_PRINT_LIMIT;
            System.err.println("[PP][LOG-SUMMARY][" + outcome + "] " + entry.getKey()
                    + " repeated=" + entry.getValue() + " suppressed=" + suppressed);
        }
    }

    private String barrierLogKey(String reason, CrossColorBarrierAnalyzer.Barrier barrier) {
        String first = barrier.clearingOrder.isEmpty() ? "?" : barrier.clearingOrder.get(0).toString();
        return "barrier." + reason
                + " blockedAgent=" + barrier.blockedAgentId
                + " clearingType=" + barrier.blockingBoxType
                + " size=" + barrier.clearingOrder.size()
                + " first=" + first;
    }

    private void recordFailureSignal(FailureReport.Cause cause,
                                     Collection<Position> blockedGoals,
                                     Collection<Position> blockedPositions) {
        if (cause != null && cause != FailureReport.Cause.UNKNOWN
                && lastFailureCauseForReport == FailureReport.Cause.UNKNOWN) {
            lastFailureCauseForReport = cause;
        }
        if (blockedGoals != null) {
            for (Position p : blockedGoals) {
                if (p != null) lastBlockedGoalsForReport.add(p);
            }
        }
        if (blockedPositions != null) {
            for (Position p : blockedPositions) {
                if (p != null) lastBlockedPositionsForReport.add(p);
            }
        }
    }

    private FailureReport.Cause effectiveFailureCause() {
        if (lastFailureCauseForReport != FailureReport.Cause.UNKNOWN) {
            return lastFailureCauseForReport;
        }
        if (lastAttemptedSubgoalForReport == null) {
            return FailureReport.Cause.UNKNOWN;
        }
        return lastAttemptedSubgoalForReport.isAgentGoal
                ? FailureReport.Cause.AGENT_GOAL_BLOCKED
                : FailureReport.Cause.BOX_GOAL_BLOCKED;
    }

    private FailureReport buildFailureReport(FailureReport.Kind kind, String note) {
        List<Subgoal> snapshot = FailureReport.snapshot(cachedSubgoalOrder);
        List<Position> blockedGoals = new ArrayList<>(lastBlockedGoalsForReport);
        List<Position> blockedPositions = new ArrayList<>(lastBlockedPositionsForReport);
        switch (kind) {
            case STUCK_NO_PROGRESS:
                return FailureReport.stuck(lastAttemptedSubgoalForReport, snapshot,
                        effectiveFailureCause(), blockedGoals, blockedPositions, note);
            case PARTIAL_PLAN:
                return FailureReport.partial(lastAttemptedSubgoalForReport, snapshot,
                        effectiveFailureCause(), blockedGoals, blockedPositions, note);
            case NO_PLAN:
            default:
                return FailureReport.noPlan(lastAttemptedSubgoalForReport, snapshot,
                        effectiveFailureCause(), blockedGoals, blockedPositions, note);
        }
    }

    private void printDiagSummary(String outcome) {
        if (SearchConfig.isVerbose()) {
        System.err.println("[PP][DIAG][" + outcome + "] IW1: invoked=" + diagIw1Invoked
                + " succeeded=" + diagIw1Succeeded
                + " | planSubgoalNull=" + diagPlanSubgoalNull
                + " pathClearingRescued=" + diagPathClearingRescued
                + " | conflictResolver: calls=" + conflictResolver.getCallCount()
                + " withConflicts=" + conflictResolver.getConflictingCallCount());
        }
        // Symmetric STUCK-DIAG info: also print on PARTIAL/SUCCESS/FAIL paths so we
        // can compare TRAP / completed-goal counts across all portfolio attempts.
        try {
            StringBuilder dump = new StringBuilder();
            dump.append("[PP][DIAG][").append(outcome).append("] completed=").append(completedBoxGoals.size());
            dump.append(" trapBlacklisted=").append(trapBlacklist.size());
            if (!trapBlacklist.isEmpty()) {
                dump.append(" {");
                int i = 0;
                for (Position bp : trapBlacklist) {
                    if (i++ > 0) dump.append(",");
                    dump.append(bp);
                    if (i >= 8) { dump.append(",..."); break; }
                }
                dump.append("}");
            }
            if (SearchConfig.isVerbose()) System.err.println(dump.toString());
        } catch (Exception ignored) {
            // Diagnostic must never crash the planner.
        }
        printRepeatedLogSummary(outcome);
        printBoxOscillationReport(outcome);
    }

    /**
     * Snapshot per-box-type position multisets after a subgoal completes.
     * Called only on madeProgress=true so rolled-back attempts don't pollute history.
     */
    private void snapshotBoxPositions(State state, String label) {
        subgoalLabelHistory.add(label);
        java.util.Map<Character, java.util.List<Position>> byType = new java.util.HashMap<>();
        for (java.util.Map.Entry<Position, Character> e : state.getBoxes().entrySet()) {
            byType.computeIfAbsent(e.getValue(), k -> new java.util.ArrayList<>()).add(e.getKey());
        }
        // Track each box type seen (also init empty lists for types not present this snapshot)
        java.util.Set<Character> allTypes = new java.util.HashSet<>(boxPositionHistory.keySet());
        allTypes.addAll(byType.keySet());
        for (Character t : allTypes) {
            java.util.List<Position> positions = byType.getOrDefault(t, java.util.Collections.emptyList());
            java.util.List<Position> sorted = new java.util.ArrayList<>(positions);
            sorted.sort((a, b) -> a.row != b.row ? a.row - b.row : a.col - b.col);
            boxPositionHistory.computeIfAbsent(t, k -> new java.util.ArrayList<>()).add(sorted);
        }
    }

    /**
     * Report boxes that LEFT and RETURNED to a position — the smoking gun for
     * NAMO↔PP thrashing. Caps output to avoid log explosion.
     */
    private void printBoxOscillationReport(String outcome) {
        int snapshots = subgoalLabelHistory.size();
        if (snapshots < 3) return;
        java.util.List<String> oscEvents = new java.util.ArrayList<>();
        for (java.util.Map.Entry<Character, java.util.List<java.util.List<Position>>> e : boxPositionHistory.entrySet()) {
            char type = e.getKey();
            java.util.List<java.util.List<Position>> trail = e.getValue();
            // For each position, find indices where this type occupied it
            java.util.Map<Position, java.util.List<Integer>> visits = new java.util.HashMap<>();
            for (int i = 0; i < trail.size(); i++) {
                for (Position p : trail.get(i)) {
                    visits.computeIfAbsent(p, k -> new java.util.ArrayList<>()).add(i);
                }
            }
            for (java.util.Map.Entry<Position, java.util.List<Integer>> v : visits.entrySet()) {
                java.util.List<Integer> idx = v.getValue();
                if (idx.size() < 2) continue;
                // Count gaps (left and returned)
                int gaps = 0;
                for (int i = 1; i < idx.size(); i++) {
                    if (idx.get(i) - idx.get(i - 1) > 1) gaps++;
                }
                if (gaps >= 1) {
                    StringBuilder sgs = new StringBuilder();
                    for (int j = 0; j < Math.min(idx.size(), 4); j++) {
                        if (j > 0) sgs.append(",");
                        int snapIdx = idx.get(j);
                        sgs.append(snapIdx).append(":").append(subgoalLabelHistory.get(snapIdx));
                    }
                    if (idx.size() > 4) sgs.append(",...");
                    oscEvents.add("[PP][BOX-OSC][" + outcome + "] box=" + type + " pos=" + v.getKey()
                            + " returned=" + gaps + "x visits=" + idx.size() + " via=[" + sgs + "]");
                }
            }
        }
        if (oscEvents.isEmpty()) return;
        // Sort by gap count descending — most thrashing first
        oscEvents.sort((a, b) -> {
            int ga = Integer.parseInt(a.substring(a.indexOf("returned=") + 9, a.indexOf("x visits")));
            int gb = Integer.parseInt(b.substring(b.indexOf("returned=") + 9, b.indexOf("x visits")));
            return gb - ga;
        });
        int cap = Math.min(oscEvents.size(), 10);
        System.err.println("[PP][BOX-OSC-SUMMARY][" + outcome + "] " + oscEvents.size()
                + " oscillation events across " + snapshots + " snapshots (showing top " + cap + ")");
        if (SearchConfig.isVerbose()) {
            for (int i = 0; i < cap; i++) System.err.println(oscEvents.get(i));
        }
    }

    public PriorityPlanningStrategy(Heuristic heuristic, SearchConfig config) {
        this(heuristic, config, new SubgoalManager(heuristic));
    }

    /**
     * Constructor with shared SubgoalManager for cross-strategy cache reuse.
     * Follows Dependency Inversion: Portfolio injects shared instance.
     */
    public PriorityPlanningStrategy(Heuristic heuristic, SearchConfig config, SubgoalManager sharedSubgoalManager) {
        this.heuristic = heuristic;
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
        this.maxStates = config.getMaxStates();
        
        this.subgoalManager = sharedSubgoalManager;
        this.conflictResolver = new ConflictResolver();
        this.boxSearchPlanner = new BoxSearchPlanner(heuristic);
        this.greedyPlanner = new GreedyPlanner();
        this.planMerger = new PlanMerger();
        this.pathAnalyzer = new PathAnalyzer();
        this.agentCoordinator = new AgentCoordinator();
        this.deadlockBreaker = new DeadlockBreaker();
    }

    @Override
    public String getName() { return "Priority Planning"; }

    @Override
    public void setTimeout(long timeoutMs) { this.timeoutMs = timeoutMs; }

    @Override
    public void setMaxStates(int maxStates) { this.maxStates = maxStates; }
    
    /** Sets pre-computed goal execution order from LevelAnalyzer. */
    public void setGoalExecutionOrder(List<Position> order) {
        this.precomputedGoalOrder = order;
        if (order != null && SearchConfig.isVerbose()) {
            System.err.println("[PP] Using pre-computed goal order: " + order.size() + " goals");
        }
    }
    
    /** Sets goal dependency graph from LevelAnalyzer. Used to distinguish hard/soft frozen goals. */
    public void setGoalDependencies(Map<Position, Set<Position>> deps) {
        this.goalDependsOn = deps != null ? deps : Collections.emptyMap();
    }
    
    /** Sets immovable goal positions (treated as walls in pathfinding). */
    public void setImmovableBoxes(Set<Position> immovable) {
        this.immovableBoxes = immovable != null ? immovable : Collections.emptySet();
    }

    /**
     * P0b: hint from PortfolioController to push these goals to the end of the order.
     * Source: prior attempts' FailureReport.lastAttemptedSubgoal.goalPos accumulated
     * across the portfolio. Does NOT remove goals — only changes ordering.
     */
    public void setDeprioritizedGoals(Set<Position> goals) {
        this.deprioritizedGoals = goals != null ? goals : Collections.emptySet();
    }

    /**
     * P4 (conflict-driven PP): set goals to promote to the front of the order.
     * PortfolioController fills this from the previous FailureReport's blocker set
     * (the unsatisfied goals that the failed subgoal depends on). Has higher priority
     * than TOPOLOGICAL/RANDOM ordering for these specific goals; other goals retain
     * their normal sort order behind them.
     */
    public void setPrioritizedGoals(Set<Position> goals) {
        this.prioritizedGoals = goals != null ? goals : Collections.emptySet();
    }

    /**
     * F2: suspend these box-goal positions from the subgoal queue for this attempt.
     * See {@link #suspendedBoxGoals} for rationale.
     */
    public void setSuspendedBoxGoals(Set<Position> goals) {
        this.suspendedBoxGoals = goals != null ? goals : Collections.emptySet();
    }

    /**
     * P1: synthetic escape subgoals from EscapeSubgoalSynthesizer; prepended to the
     * cached order on next compute. Used by PortfolioController to break 2-cycles
     * after a TOPOLOGICAL attempt failed on a level with hasCircularDependency=true.
     */
    public void setEscapeSubgoals(List<Subgoal> subgoals) {
        this.escapeSubgoals = subgoals != null ? subgoals : Collections.emptyList();
        // P2: build position index for IW(1) routing.
        if (this.escapeSubgoals.isEmpty()) {
            this.escapeGoalPositions = Collections.emptySet();
        } else {
            Set<Position> set = new HashSet<>(this.escapeSubgoals.size() * 2);
            for (Subgoal sg : this.escapeSubgoals) set.add(sg.goalPos);
            this.escapeGoalPositions = set;
        }
    }

    /** Sets the ordering mode for subgoal execution. */
    public void setOrderingMode(OrderingMode mode) {
        this.orderingMode = mode != null ? mode : OrderingMode.TOPOLOGICAL;
    }

    /** Sets the random seed for RANDOM ordering mode. Allows portfolio to try different shuffles. */
    public void setRandomSeed(int seed) {
        this.random = new Random(seed);
    }

    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        int numAgents = initialState.getNumAgents();

        // Ensure robust dependency analysis is used (SRP fix form step 2)
        // If external controller provided no order, compute it ourselves using the standard analyzer
        if (precomputedGoalOrder == null) {
             LevelAnalyzer.LevelFeatures features = LevelAnalyzer.analyze(level, initialState);
             setGoalExecutionOrder(features.executionOrder);
             setGoalDependencies(features.goalDependsOn);
             if (features.taskFilter != null && (immovableBoxes == null || immovableBoxes.isEmpty())) {
                 setImmovableBoxes(features.taskFilter.immovableBoxes);
             }
        }

        // Precompute BFS distance maps for all goals (O(G×N) once, then O(1) per query)
        subgoalManager.initDistanceCache(initialState, level);
        
        // Adapt max BSP budget based on level size — large levels (50×50) with many
        // entities can OOM if each BSP search explores 80K+ states.
        int freeSpaces = 0;
        for (int r = 0; r < level.getRows(); r++)
            for (int c = 0; c < level.getCols(); c++)
                if (!level.isWall(r, c)) freeSpaces++;
        effectiveMaxBspBudget = freeSpaces > 500 
            ? SearchConfig.MAX_BSP_BUDGET_LARGE 
            : SearchConfig.MAX_BSP_BUDGET;

        // Precompute articulation points for parking avoidance (O(V+E) once)
        Set<Position> ap = ArticulationPointFinder.findArticulationPoints(level, immovableBoxes);
        pathAnalyzer.setArticulationPoints(ap);
        if (SearchConfig.isVerbose()) {
            System.err.println("[PP] Articulation points: " + ap.size());
        }

        // Compute independent agent groups for parallel execution
        agentComponentId = computeAgentComponents(initialState, level);

        // Reset for new search
        displacementHistory.clear();
        displacementAttempts = 0;
        completedBoxGoals.clear();
        completedAgentGoals.clear();
        goalCompletionCount.clear();
        displacedGoals.clear();
        reservationTable.clear();
        planMerger.clearAllPlans();
        storedPlanSubgoals.clear();
        globalTimeStep = 0;
        lastComputedState = null;
        lastComputedPlanSize = 0;
        dynamicBarrierRounds = 0;
        trapBlacklist.clear();
        deferredBlockedGoals.clear();
        syntheticReliefBlacklist.clear();
        taskReliefNogoods.clear();
        sealStagingNogoods.clear();
        regressDisturbCount.clear();
        lastProgressTime = System.currentTimeMillis();
        lastFailureReport = null;
        lastAttemptedSubgoalForReport = null;
        lastFailureCauseForReport = FailureReport.Cause.UNKNOWN;
        lastBlockedGoalsForReport.clear();
        lastBlockedPositionsForReport.clear();
        diagIw1Invoked = 0;
        diagIw1Succeeded = 0;
        diagPlanSubgoalNull = 0;
        diagPathClearingRescued = 0;
        repeatedLogCounts.clear();
        boxPositionHistory.clear();
        subgoalLabelHistory.clear();
        lastBorrowedGoalBoxPositions.clear();
        suspendedTransitGoals.clear();
        transitProfiles = Collections.emptyMap();
        conflictResolver.resetCounts();
        seedCompletedGoalsFromState(initialState, level);

        // Layer-3 Steps 2-4: classify goals by transit pressure (computed once per search).
        {
            List<Position> goalPositions = new ArrayList<>();
            for (int r = 0; r < level.getRows(); r++) {
                for (int c = 0; c < level.getCols(); c++) {
                    if (level.getBoxGoal(r, c) != '\0') {
                        Position gp = new Position(r, c);
                        if (immovableBoxes == null || !immovableBoxes.contains(gp)) {
                            goalPositions.add(gp);
                        }
                    }
                }
            }
            transitProfiles = GoalTransitAnalyzer.analyze(goalPositions, level, initialState);
        }

        return planWithSubgoals(initialState, level, startTime);
    }

    private void seedCompletedGoalsFromState(State state, Level level) {
        int seededBoxes = 0;
        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            for (Position goalPos : entry.getValue()) {
                Character actual = state.getBoxes().get(goalPos);
                if (actual != null && actual == goalType && completedBoxGoals.add(goalPos)) {
                    seededBoxes++;
                }
            }
        }

        int seededAgents = 0;
        for (Map.Entry<Integer, Position> entry : level.getAgentGoalPositionMap().entrySet()) {
            int agentId = entry.getKey();
            Position goalPos = entry.getValue();
            if (agentId >= 0 && agentId < state.getNumAgents()
                    && goalPos.equals(state.getAgentPosition(agentId))
                    && completedAgentGoals.add(goalPos)) {
                seededAgents++;
            }
        }

        if (seededBoxes > 0 || seededAgents > 0) {
            logNormal("[PP] Seeded completed goals from start state: boxes="
                    + seededBoxes + ", agents=" + seededAgents);
        }
    }

    /** Cached subgoal order - MAPF PP requires fixed priority (computed once, reused). */
    private List<Subgoal> cachedSubgoalOrder = null;

    /**
     * Core PP algorithm: iteratively solve subgoals in priority order.
     * Per ARCHITECTURE.md: "moves one box at a time, naturally handles dependencies"
     * MAPF FIX: Priority is fixed at start, not recomputed each iteration.
     */
    private List<Action[]> planWithSubgoals(State initialState, Level level, long startTime) {
        List<Action[]> fullPlan = new ArrayList<>();
        State currentState = initialState;
        int numAgents = initialState.getNumAgents();
        int stuckCount = 0;
        
        // MAPF FIX: Compute subgoal order ONCE at start, then reuse
        cachedSubgoalOrder = null;

        // Cross-color barrier clearing: detect if any agent is blocked from its goals
        // by boxes of a different color, and pre-clear a path before the main loop.
        currentState = detectAndExecuteClearingPhase(
                currentState, initialState, level, fullPlan, numAgents, startTime);
        int lastStablePlanSize = hasOpenGoalTransaction(currentState, level) ? 0 : fullPlan.size();

        while (!currentState.isGoalState(level)) {
            // PRODUCT.md constraints: 3 minutes, 20,000 actions
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                logVerbose(getName() + ": Timeout");
                currentState = rollbackToStablePartialIfNeeded(initialState, currentState,
                        level, fullPlan, numAgents, lastStablePlanSize, "timeout");
                break;
            }
            if (fullPlan.size() >= SearchConfig.MAX_ACTIONS) {
                logVerbose(getName() + ": Action limit");
                currentState = rollbackToStablePartialIfNeeded(initialState, currentState,
                        level, fullPlan, numAgents, lastStablePlanSize, "action-limit");
                break;
            }
            if (stuckCount > SearchConfig.MAX_STUCK_ITERATIONS) {
                logVerbose(getName() + ": Stuck");
                currentState = rollbackToStablePartialIfNeeded(initialState, currentState,
                        level, fullPlan, numAgents, lastStablePlanSize, "stuck");
                break;
            }

            // Early termination: if no genuine progress for 35% of timeout,
            // return partial plan and let portfolio try next strategy.
            // This prevents burning the entire budget on repeated failures.
            // Note: 35% balances responsiveness (give up early enough for retries)
            // vs persistence (don't abort strategies that need sustained effort).
            // Plan-A: also fire when fullPlan IS empty — a strategy that has
            // produced ZERO actions in 35% of its budget is unlikely to ever
            // produce any, and yielding the rest of the budget to the next
            // strategy is strictly better than spinning silently to timeout.
            long noProgressMs = System.currentTimeMillis() - lastProgressTime;
            boolean openGoalTransaction = hasOpenGoalTransaction(currentState, level);
            double earlyExitFraction = openGoalTransaction
                    ? OPEN_TRANSACTION_EARLY_EXIT_FRACTION
                    : NORMAL_EARLY_EXIT_FRACTION;
            if (noProgressMs > timeoutMs * earlyExitFraction) {
                if (!fullPlan.isEmpty()) {
                    logVerbose(getName() + ": [EARLY-EXIT] No progress for " + noProgressMs + "ms — returning partial plan");
                } else {
                    logVerbose(getName() + ": [EARLY-EXIT] No progress for " + noProgressMs + "ms — yielding to next strategy (empty plan)");
                }
                // Diagnostic dump: surface what state we got stuck in.
                // 2026-05 (post Bug-1): when STUCK fires we want to know
                //   - which goals were blacklisted by TRAP this run
                //   - which goal was last attempted
                //   - how many goals are still unsatisfied
                // Helps differentiate "stuck because TRAP killed all options" from
                // "stuck because BSP couldn't plan a clear path".
                try {
                    StringBuilder dump = new StringBuilder();
                    dump.append(getName()).append(": [STUCK-DIAG] ");
                    dump.append("completed=").append(completedBoxGoals.size());
                    dump.append(" trapBlacklisted=").append(trapBlacklist.size());
                    if (!trapBlacklist.isEmpty()) {
                        dump.append(" {");
                        int i = 0;
                        for (Position bp : trapBlacklist) {
                            if (i++ > 0) dump.append(",");
                            char bg = level.getBoxGoal(bp);
                            if (bg != 0) dump.append(bg);
                            dump.append("@").append(bp);
                            if (i >= 8) { dump.append(",..."); break; }
                        }
                        dump.append("}");
                    }
                    if (lastAttemptedSubgoalForReport != null) {
                        dump.append(" lastAttempted=agent").append(lastAttemptedSubgoalForReport.agentId)
                            .append("->").append(lastAttemptedSubgoalForReport.boxType)
                            .append("@").append(lastAttemptedSubgoalForReport.goalPos);
                    }
                    logVerbose(dump.toString());
                } catch (Exception e) {
                    // Diagnostic must never crash the planner.
                }
                currentState = rollbackToStablePartialIfNeeded(initialState, currentState,
                        level, fullPlan, numAgents, lastStablePlanSize, "early-exit");
                lastFailureReport = buildFailureReport(FailureReport.Kind.STUCK_NO_PROGRESS,
                        "early-exit noProgressMs=" + noProgressMs
                                + " stuckCount=" + stuckCount
                                + (openGoalTransaction ? " openGoalTransaction=true" : ""));
                break;
            }

            // Try displacement-based cycle recovery on cyclic dependency detection
            if (stuckCount == SearchConfig.DEPENDENCY_CHECK_THRESHOLD) {
                tryCycleRecovery(currentState, level, fullPlan, numAgents);
                currentState = recomputeState(initialState, fullPlan, level, numAgents);
            }

            // MAPF FIX: Use cached order, only filter out completed goals
            List<Subgoal> unsatisfied = getOrComputeSubgoalOrder(currentState, level);
            if (unsatisfied.isEmpty()) break;

            // Try to execute highest priority subgoal
            boolean madeProgress = tryExecuteSubgoals(unsatisfied, fullPlan, currentState, level, numAgents, initialState);
            
            if (madeProgress) {
                // Update state from plan
                currentState = recomputeState(initialState, fullPlan, level, numAgents);
                // [BOX-OSC] snapshot box layout to detect NAMO↔PP thrashing
                {
                    Subgoal sg = lastAttemptedSubgoalForReport;
                    String label = sg == null ? "?"
                            : (sg.isAgentGoal ? "A" + sg.agentId + "@" + sg.goalPos
                                              : sg.boxType + "@" + sg.goalPos);
                    snapshotBoxPositions(currentState, label);
                }
                // Only reset stuckCount for genuine progress (not re-solving same agent goal)
                if (!lastProgressWasPhantom) {
                    stuckCount = 0;
                    lastProgressTime = System.currentTimeMillis();
                    if (!hasOpenGoalTransaction(currentState, level)) {
                        lastStablePlanSize = fullPlan.size();
                    }
                    // MAPF FIX: Clear displacement history on genuine progress
                    displacementHistory.clear();
                    displacementAttempts = 0;
                    // Clear barrier cache — state changed, barriers may now be extractable
                    skippedBarrierClearingOrders.clear();
                    dynamicBarrierRounds = 0;
                    // Decay regress disturbance counts on genuine progress.
                    // Goals disturbed by earlier exploration get credit for progress,
                    // preventing premature REGRESS-UNPROTECT marking from early-phase
                    // disturbances that were resolved by subsequent subgoal success.
                    if (!regressDisturbCount.isEmpty()) {
                        regressDisturbCount.replaceAll((pos, count) -> Math.max(0, count - 1));
                        regressDisturbCount.values().removeIf(c -> c <= 0);
                    }
                } else {
                    stuckCount++;
                }
            } else {
                stuckCount++;
                
                // Try recovery mechanisms when stuck
                if (stuckCount >= SearchConfig.STUCK_ITERATIONS_BEFORE_CLEARING) {
                    boolean recovered = tryRecovery(unsatisfied, fullPlan, currentState, level, numAgents, initialState);
                    if (recovered) {
                        currentState = recomputeState(initialState, fullPlan, level, numAgents);
                        if (!hasOpenGoalTransaction(currentState, level)) {
                            lastStablePlanSize = fullPlan.size();
                        }
                        stuckCount = 0;
                    } else {
                        // Dynamic barrier re-detection: state changes may have created
                        // new cross-color barriers. Run barrier analysis again.
                        // Limited to prevent wasting steps on incomplete clearing.
                        if (dynamicBarrierRounds < MAX_DYNAMIC_BARRIER_ROUNDS) {
                            State afterBarrier = detectAndExecuteClearingPhase(
                                    currentState, initialState, level, fullPlan, numAgents, startTime);
                            dynamicBarrierRounds++;
                            if (afterBarrier != currentState) {
                                currentState = afterBarrier;
                                if (!hasOpenGoalTransaction(currentState, level)) {
                                    lastStablePlanSize = fullPlan.size();
                                }
                                stuckCount = 0;
                                logVerbose("[PP] Dynamic barrier re-detection cleared new barriers (round "
                                        + dynamicBarrierRounds + "/" + MAX_DYNAMIC_BARRIER_ROUNDS + ")");
                            }
                        }
                    }
                }
            }
        }

        currentState = rollbackToStablePartialIfNeeded(initialState, currentState,
                level, fullPlan, numAgents, lastStablePlanSize, "partial-exit");

        if (currentState.isGoalState(level)) {
            logMinimal(getName() + ": [OK] Goal state reached!");
            // MAPF FIX: Validate and optimize the plan before returning
            fullPlan = validateAndOptimizePlan(fullPlan, initialState, level, numAgents);
            lastFailureReport = null; // success clears any prior failure signal
            printDiagSummary("SUCCESS");
            return fullPlan.isEmpty() ? null : fullPlan;
        }
        // Goal not fully reached — return partial plan for use as fallback
        if (!fullPlan.isEmpty()) {
            logMinimal(getName() + ": [PARTIAL] Partial plan (" + fullPlan.size() + " steps) — goal not reached");
            fullPlan = validateAndOptimizePlan(fullPlan, initialState, level, numAgents);
            // Only set if a more specific report wasn't already attached at break-time.
            if (lastFailureReport == null) {
                lastFailureReport = buildFailureReport(FailureReport.Kind.PARTIAL_PLAN,
                        "plan size=" + fullPlan.size());
            }
            printDiagSummary("PARTIAL");
            return fullPlan.isEmpty() ? null : fullPlan;
        }
        logMinimal(getName() + ": [FAIL] Could not reach goal state, no partial plan available");
        if (lastFailureReport == null) {
            lastFailureReport = buildFailureReport(FailureReport.Kind.NO_PLAN,
                    "no partial plan");
        }
        printDiagSummary("FAIL");
        return null;
    }

    // ==================== Cross-Color Barrier Clearing ====================

    /**
     * Pre-pass: detects cross-color barriers and clears them before the main PP loop.
     * 
     * In pull-supporting Sokoban, a single agent can move any same-color box through
     * arbitrarily narrow corridors using pull chains. When an agent is blocked by
     * OTHER-color boxes, we identify the minimum set of boxes to clear, assign
     * a same-color agent to clear each one via planBoxDisplacement, and execute.
     * 
     * After clearing, the displaced box-goals become unsatisfied. The normal PP loop
     * will generate return subgoals automatically (cyan agents push A-boxes back).
     * 
     * @return updated state after clearing (or original state if no barriers)
     */
    private State detectAndExecuteClearingPhase(State currentState, State initialState,
                                                  Level level, List<Action[]> fullPlan,
                                                  int numAgents, long startTime) {
        // Only analyze barriers if there are multiple colors of boxes
        if (level.getNumAgents() < 2) return currentState;

        // Build a minimal subgoal list for barrier analysis
        List<Subgoal> checkSubgoals = subgoalManager.getUnsatisfiedSubgoals(currentState, level, completedBoxGoals);
        if (checkSubgoals.isEmpty()) return currentState;

        List<CrossColorBarrierAnalyzer.Barrier> barriers =
                CrossColorBarrierAnalyzer.analyzeBarriers(currentState, level, checkSubgoals);

        if (barriers.isEmpty()) return currentState;

        logNormalRepeated("barrier.detected count=" + barriers.size(),
                "[PP] Cross-color barriers detected: " + barriers.size());

        // Sort barriers by clearing order size: solve small barriers first. 
        // This opens up parking space (e.g., clearing Z opens left-right connection)
        // before tackling large barriers that need many parking spots.
        barriers.sort((a, b) -> Integer.compare(a.clearingOrder.size(), b.clearingOrder.size()));

        // Plan-A guard: cap total clearing-phase wall-clock at 30% of strategy
        // timeout. Without this, a single barrier whose pre-assignment / BFS
        // explodes (NameHere: 10 barriers, deep maze) consumes the entire
        // strategy budget without producing ANY plan steps, leaving the
        // portfolio's bestPartialPlan permanently null.
        long clearingPhaseBudget = Math.max(5_000L, timeoutMs * 30 / 100);
        long clearingPhaseDeadline = startTime + clearingPhaseBudget;

        for (CrossColorBarrierAnalyzer.Barrier barrier : barriers) {
            long now = System.currentTimeMillis();
            if (now > clearingPhaseDeadline || now - startTime > timeoutMs / 2) {
                logVerbose("[PP] Clearing phase timeout — skipping remaining barriers"
                        + " (elapsed=" + (now - startTime) + "ms, budget=" + clearingPhaseBudget + "ms)");
                break;
            }

            // Cache check: skip barriers that were already found non-extractable
            if (skippedBarrierClearingOrders.contains(barrier.clearingOrder)) {
                logVerbose("[PP] Barrier already cached as non-extractable — skipping");
                continue;
            }

            // Find clearing agent: try all eligible agents in distance order.
            // If the closest agent is permanently cached as failed, try the next.
            List<int[]> allClearingAgents = CrossColorBarrierAnalyzer.findAllClearingAgents(
                    barrier.blockingColor, barrier.clearingOrder.get(0), currentState, level);
            if (allClearingAgents.isEmpty()) {
                logNormal("[PP] No agent of color " + barrier.blockingColor + " found — skipping");
                continue;
            }

            Set<Position> barrierPositionSet = new HashSet<>(barrier.clearingOrder);
            int clearingAgent = -1;
            for (int[] agentEntry : allClearingAgents) {
                int candidateAgent = agentEntry[0];
                Set<Set<Position>> agentFailures = permanentlyFailedBarriers.getOrDefault(
                        candidateAgent, Collections.emptySet());
                boolean isPermanentlyFailed = agentFailures.stream()
                        .anyMatch(cached -> cached.containsAll(barrierPositionSet));
                if (!isPermanentlyFailed) {
                    clearingAgent = candidateAgent;
                    break;
                }
                logVerbose("[PP] Agent " + candidateAgent + " permanently failed for this barrier — trying next");
            }
            if (clearingAgent < 0) {
                logNormal("[PP] All agents permanently failed for this barrier — skipping");
                continue;
            }

            logNormalRepeated(barrierLogKey("clearing-start.agent" + clearingAgent, barrier),
                    "[PP] Clearing barrier for agent " + barrier.blockedAgentId
                    + ": " + barrier.clearingOrder.size() + " " + barrier.blockingBoxType
                    + "-boxes to clear (clearing agent " + clearingAgent + ")");

            // Find parking positions for the cleared boxes
            Set<Position> agentReachable = bfsReachableForClearing(
                    currentState.getAgentPosition(clearingAgent), currentState, level);

            // PRE-CHECK: Verify the first barrier box is physically extractable.
            // Use barrier-aware version: other barrier boxes will be cleared sequentially,
            // so they should be treated as removable when checking push/bypass routes.
            Position firstBox = barrier.clearingOrder.get(0);
            Set<Position> otherBarrierBoxes = new HashSet<>(barrier.clearingOrder);
            otherBarrierBoxes.remove(firstBox);
            if (!CrossColorBarrierAnalyzer.isBoxExtractable(firstBox, agentReachable, currentState, level, otherBarrierBoxes)) {
                logNormal("[PP] First barrier box " + firstBox + " is not extractable "
                        + "(dead-end corridor, no bypass) — skipping barrier");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }

            Set<Position> baseAvoidPositions = new HashSet<>();
            // Determine exclusion aggressiveness based on barrier size.
            // Large barriers (>3 boxes) need nearby parking (e.g., adjacent row);
            // aggressive exclusion zones would eliminate all viable parking.
            boolean isLargeBarrier = barrier.clearingOrder.size() > 3;
            
            if (isLargeBarrier) {
                // Large barriers: only exclude the barrier positions themselves.
                // BSP handles collateral boxes naturally during routing.
                baseAvoidPositions.addAll(barrier.clearingOrder);
            } else {
                // Small barriers: original conservative exclusion.
                baseAvoidPositions.addAll(barrier.clearingOrder);
                // Radius-1 around ALL barrier boxes
                for (Position barrierPos : barrier.clearingOrder) {
                    for (Direction dir : Direction.values()) {
                        Position adj = barrierPos.move(dir);
                        if (!level.isWall(adj)) {
                            baseAvoidPositions.add(adj);
                        }
                    }
                }
                // Extended exclusion around gap entry (first box = closest to agent)
                Position gapEntry = barrier.clearingOrder.get(0);
                for (int dr = -2; dr <= 2; dr++) {
                    for (int dc = -2; dc <= 2; dc++) {
                        if (Math.abs(dr) + Math.abs(dc) <= 2) {
                            Position near = Position.of(gapEntry.row + dr, gapEntry.col + dc);
                            if (!level.isWall(near)) {
                                baseAvoidPositions.add(near);
                            }
                        }
                    }
                }
            }

            // APPROACH CORRIDOR: compute BFS shortest path from agent to the first
            // barrier box. All cells on this path are critical for reaching the barrier;
            // parking on them would block subsequent box extractions.
            List<Position> approachPath = computeApproachPath(
                    currentState.getAgentPosition(clearingAgent), firstBox, currentState, level);
            baseAvoidPositions.addAll(approachPath);
            
            // For large barriers: the entire approach path is a bottleneck corridor.
            // We must ensure agent can still reach the gap from both sides after parking.
            // Strategy: also protect the cell immediately through the gap (one step past
            // the gap on the non-barrier side). This is the critical connector.
            if (isLargeBarrier && approachPath.size() >= 2) {
                // Last cell of approach path is adjacent to gap, the gap itself
                // is already in baseAvoidPositions. Also protect immediate neighbors
                // of all approach path cells in the gap area (last 3 cells of path).
                int protectStart = Math.max(0, approachPath.size() - 3);
                for (int pi = protectStart; pi < approachPath.size(); pi++) {
                    Position pathCell = approachPath.get(pi);
                    for (Direction dir : Direction.values()) {
                        Position adj = pathCell.move(dir);
                        if (!level.isWall(adj)) {
                            baseAvoidPositions.add(adj);
                        }
                    }
                }
            }

            // GAP EXIT ROW PROTECTION: the row connecting the agent's area to the gap
            // is a critical corridor. Boxes parked here block BSP routing and trap
            // the clearing agent. The approach path structure is:
            //   path = [..., gapExitCell, gapCell, firstBarrierBox]
            // Protect the entire continuous segment of the gap exit row.
            // Pre-assignment ensures connectivity elsewhere; this protection prevents
            // BSP collateral damage from blocking the approach corridor.
            if (approachPath.size() >= 3) {
                Position gapExitCell = approachPath.get(approachPath.size() - 3);
                int gapExitRow = gapExitCell.row;
                for (int c = gapExitCell.col; c < level.getCols(); c++) {
                    Position pos = Position.of(gapExitRow, c);
                    if (level.isWall(pos)) break;
                    baseAvoidPositions.add(pos);
                }
                for (int c = gapExitCell.col - 1; c >= 0; c--) {
                    Position pos = Position.of(gapExitRow, c);
                    if (level.isWall(pos)) break;
                    baseAvoidPositions.add(pos);
                }
                logVerbose("[PP] [CLEAR-BARRIER] Gap exit row " + gapExitRow + " protected from parking");
            }

            // Build unfreeze positions INCREMENTALLY per box, not for the whole barrier.
            // This prevents BSP from pushing future barrier boxes into the approach corridor
            // as collateral damage. Each box displacement only unfreezes:
            // (a) the current box's position (so BSP can move it off its goal)
            // (b) positions already cleared (so BSP can route through them)
            Set<Position> clearedPositions = new HashSet<>();

            // PRE-CHECK: verify sufficient parking for ALL barrier boxes.
            // Incomplete clearing wastes steps without enabling the blocked agent's goals.
            // Dynamic threshold: skip only if parking is genuinely insufficient.
            if (barrier.clearingOrder.size() > 15) {
                logNormal("[PP] [CLEAR-BARRIER] Barrier too large ("
                        + barrier.clearingOrder.size() + " boxes) — skipping");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }
            // Request a large candidate pool for pre-assignment backtracking
            List<Position> allParking = CrossColorBarrierAnalyzer.findParkingPositions(
                    Math.max(barrier.clearingOrder.size() * 3, 20), agentReachable, currentState, level,
                    baseAvoidPositions, firstBox);
            if (allParking.size() < barrier.clearingOrder.size()) {
                recordFailureSignal(FailureReport.Cause.PARKING_UNAVAILABLE,
                        barrier.clearingOrder, barrier.clearingOrder);
                logNormal("[PP] [CLEAR-BARRIER] Insufficient parking (" + allParking.size()
                        + "/" + barrier.clearingOrder.size() + ") — skipping barrier");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }

            // FULL-HORIZON PRE-ASSIGNMENT: Assign parking for ALL barrier boxes
            // before clearing any. Uses greedy + backtracking to ensure agent
            // maintains connectivity to the gap at every intermediate step.
            // This prevents the "self-trapping" failure where per-box greedy
            // parking fills narrow corridors and cuts off the return path.
            // TRANSIT PROTECTION: also verifies the blocked agent can still reach
            // the gap area after all boxes are parked. Without this, parking fills
            // all vertical corridors and traps the blocked agent in the lower area.
            Position blockedAgentPos = currentState.getAgentPosition(barrier.blockedAgentId);
            Position gapExitCell = (approachPath.size() >= 3) 
                    ? approachPath.get(approachPath.size() - 3) : null;
            List<Position> preAssignedSlots = preAssignParkingSlots(
                    barrier.clearingOrder, allParking, currentState, level,
                    blockedAgentPos, gapExitCell);
            int maxSafeClears = preAssignedSlots.size();
            if (maxSafeClears == 0) {
                recordFailureSignal(FailureReport.Cause.PARKING_UNAVAILABLE,
                        barrier.clearingOrder, barrier.clearingOrder);
                logNormal("[PP] [CLEAR-BARRIER] Pre-assignment found no safe parking — skipping barrier");
                skippedBarrierClearingOrders.add(barrier.clearingOrder);
                continue;
            }
            if (maxSafeClears < barrier.clearingOrder.size()) {
                logVerbose("[PP] [CLEAR-BARRIER] Pre-assignment: max " + maxSafeClears
                        + "/" + barrier.clearingOrder.size() + " boxes safely clearable");
            } else {
                logVerbose("[PP] [CLEAR-BARRIER] Pre-assignment: all " + maxSafeClears
                        + " boxes have safe parking slots");
            }

            // Track parking positions used so far — avoid them for subsequent boxes
            Set<Position> usedParkings = new HashSet<>();

            int cleared = 0;
            boolean earlyExit = false;
            for (int i = 0; i < Math.min(barrier.clearingOrder.size(), maxSafeClears); i++) {
                Position boxPos = barrier.clearingOrder.get(i);

                // Verify box is still at the expected position
                Character boxAtPos = currentState.getBoxes().get(boxPos);
                if (boxAtPos == null || boxAtPos != barrier.blockingBoxType) {
                    logVerbose("[PP] [CLEAR-BARRIER] Box at " + boxPos + " changed — skipping");
                    continue;
                }

                // Check extractability for each box (reachability changes as boxes are cleared)
                Set<Position> updatedReachable = bfsReachableForClearing(
                        currentState.getAgentPosition(clearingAgent), currentState, level);
                if (cleared == 0) {
                    // Full extractability pre-check for the first box (barrier-aware)
                    Set<Position> remainingBarrier = new HashSet<>(barrier.clearingOrder);
                    remainingBarrier.remove(boxPos);
                    if (!CrossColorBarrierAnalyzer.isBoxExtractable(boxPos, updatedReachable, currentState, level, remainingBarrier)) {
                        logNormal("[PP] [CLEAR-BARRIER] Box " + boxPos + " not extractable — stopping");
                        break;
                    }
                } else {
                    // For subsequent boxes: skip the expensive extractability pre-check.
                    // The first box's clearance confirms the gap is workable; parked boxes
                    // may block the pull-chain bypass BFS but BSP can route around them.
                    // Only verify the agent can physically reach an adjacent cell.
                    boolean canReach = false;
                    for (Direction dir : Direction.values()) {
                        if (updatedReachable.contains(boxPos.move(dir))) {
                            canReach = true;
                            break;
                        }
                    }
                    if (!canReach) {
                        logNormal("[PP] [CLEAR-BARRIER] Box " + boxPos + " not reachable by agent"
                                + " (agent at " + currentState.getAgentPosition(clearingAgent) + ") — stopping");
                        break;
                    }
                }

                // Use pre-assigned parking slot (computed by full-horizon pre-assignment)
                Position parkPos = preAssignedSlots.get(i);

                // Dynamic verification: if state diverged (BSP collateral, conflict resolution),
                // the pre-assigned slot may no longer be free. Fall back to per-box selection.
                if (currentState.hasBoxAt(parkPos)) {
                    logNormal("[PP] [CLEAR-BARRIER] Pre-assigned slot " + parkPos
                            + " is now occupied — falling back to per-box selection");
                    Set<Position> avoidPositions = new HashSet<>(baseAvoidPositions);
                    avoidPositions.addAll(usedParkings);
                    List<Position> fallbackCandidates = CrossColorBarrierAnalyzer.findParkingPositions(
                            Math.max(5, barrier.clearingOrder.size()), updatedReachable, currentState, level,
                            avoidPositions, boxPos);
                    parkPos = null;
                    Position nextBarrierBox = (i + 1 < maxSafeClears)
                            ? barrier.clearingOrder.get(i + 1) : null;
                    for (Position candidate : fallbackCandidates) {
                        if (nextBarrierBox == null) {
                            parkPos = candidate;
                            break;
                        }
                        boolean gapOk = isGapStillAccessible(candidate, boxPos, usedParkings,
                                barrier.clearingOrder, i, currentState, level);
                        if (gapOk) {
                            parkPos = candidate;
                            break;
                        }
                    }
                    if (parkPos == null) {
                        recordFailureSignal(FailureReport.Cause.PARKING_UNAVAILABLE,
                                Collections.singletonList(boxPos), barrier.clearingOrder);
                        logNormal("[PP] [CLEAR-BARRIER] No fallback parking for " + boxPos + " — stopping");
                        break;
                    }
                }

                logVerbose("[PP] [CLEAR-BARRIER] Moving " + barrier.blockingBoxType + " from "
                        + boxPos + " to " + parkPos + " (agent " + clearingAgent + ")");

                int planSizeBefore = fullPlan.size();

                // Use higher BSP budget — pull-chain paths through narrow gaps need more exploration
                // Progressive budget: deeper boxes need exponentially longer displacement paths
                // because they must navigate through already-cleared corridor + complex maze
                boolean singleBoxBarrier = barrier.clearingOrder.size() == 1;
                int clearingBudget = SearchConfig.MIN_BSP_BUDGET
                        * (singleBoxBarrier ? 20 : (8 + cleared * 4));
                
                // Build per-box unfreeze set: current box + already-cleared positions.
                // This prevents BSP from pushing remaining barrier boxes as collateral damage.
                Set<Position> unfreezePositions = new HashSet<>(clearedPositions);
                unfreezePositions.add(boxPos);
                Set<Position> protectedPositions = new HashSet<>(baseAvoidPositions);
                protectedPositions.addAll(usedParkings);
                
                List<Action> displacePath = boxSearchPlanner.planBoxDisplacementWithUnfreeze(
                        clearingAgent, boxPos, parkPos, barrier.blockingBoxType,
                        currentState, level, unfreezePositions, clearingBudget,
                        protectedPositions);

                if (displacePath == null || displacePath.isEmpty()) {
                    // Retry with full unfreeze — limited unfreeze may be too restrictive
                    // when BSP needs to push other barrier boxes to navigate through gap
                    Set<Position> fullUnfreeze = new HashSet<>(barrier.clearingOrder);
                    displacePath = boxSearchPlanner.planBoxDisplacementWithUnfreeze(
                            clearingAgent, boxPos, parkPos, barrier.blockingBoxType,
                            currentState, level, fullUnfreeze, clearingBudget,
                            protectedPositions);
                }

                // RETRY WITH ALTERNATIVE PARKING TARGETS
                // Pre-assigned target may be in a congested area (e.g., deep in finger corridor
                // behind parked boxes). Try simpler targets: row 10 (relaxed for last box),
                // upper-left corridor, etc.
                if (displacePath == null || displacePath.isEmpty()) {
                    logVerbose("[PP] [CLEAR-BARRIER] BSP failed for " + boxPos + " -> " + parkPos
                            + " — trying alternative targets");
                    
                    boolean isLastBox = (i == barrier.clearingOrder.size() - 1);
                    
                    // Build relaxed avoidSet for alternative candidates:
                    // - Always exclude barrier positions and used parkings
                    // - For last box: DROP gap exit row protection (no subsequent boxes need it)
                    //   and DROP approach corridor protection
                    // - For non-last box: keep full baseAvoidPositions
                    Set<Position> retryAvoid = new HashSet<>();
                    retryAvoid.addAll(barrier.clearingOrder);
                    retryAvoid.addAll(usedParkings);
                    if (!isLastBox) {
                        retryAvoid.addAll(baseAvoidPositions);
                    }
                    
                    Set<Position> updReach = bfsReachableForClearing(
                            currentState.getAgentPosition(clearingAgent), currentState, level);
                    // Sort by distance to the BOX (not the gap reference) — closer = simpler BSP path
                    List<Position> retryCandidates = CrossColorBarrierAnalyzer.findParkingPositions(
                            30, updReach, currentState, level, retryAvoid, boxPos);

                    int rayCandidatesBefore = retryCandidates.size();
                    addBoxRayParkingCandidates(retryCandidates, boxPos, retryAvoid,
                            currentState, level, 12);
                    if (retryCandidates.size() > rayCandidatesBefore) {
                        logNormalRepeated(barrierLogKey("ray-parking-candidates.box" + boxPos, barrier),
                                "[PP] [CLEAR-BARRIER] Added "
                                        + (retryCandidates.size() - rayCandidatesBefore)
                                        + " box-ray parking candidate(s) for " + boxPos);
                    }
                    
                    // BARRIER EXTENSION CANDIDATES: cells beyond the last barrier box
                    // along the corridor direction are not walkable (the box blocks the
                    // path) but ARE reachable via BSP pushing. Add them as candidates.
                    if (barrier.clearingOrder.size() >= 2) {
                        Position lastBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 1);
                        Position prevBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 2);
                        int bdr = lastBox.row - prevBox.row;
                        int bdc = lastBox.col - prevBox.col;
                        Position ext = lastBox;
                        for (int step = 0; step < 20; step++) {
                            ext = Position.of(ext.row + bdr, ext.col + bdc);
                            if (level.isWall(ext)) break;
                            if (retryAvoid.contains(ext)) continue;
                            if (retryCandidates.contains(ext)) continue;
                            if (level.getBoxGoal(ext) != '\0') continue;
                            if (level.getAgentGoal(ext.row, ext.col) != -1) continue;
                            // Add if not already a box (it will be free after push)
                            if (!currentState.hasBoxAt(ext) || ext.equals(boxPos)) {
                                retryCandidates.add(ext);
                            }
                        }
                    }
                    
                    // Build simulated obstacle set for transit checks:
                    // post-clearing state = current boxes - all barrier positions + used parkings
                    Set<Position> baseRetryObs = new HashSet<>();
                    for (Position bp : currentState.getBoxes().keySet()) {
                        baseRetryObs.add(bp);
                    }
                    for (Position bp : barrier.clearingOrder) {
                        baseRetryObs.remove(bp); // barrier boxes will be removed
                    }
                    // usedParkings are already reflected in currentState
                    
                    Position blockedPos = currentState.getAgentPosition(barrier.blockedAgentId);
                    
                    // Compute barrier extension direction for corridor-bypass check.
                    // Cells on the barrier extension (past the last barrier box) that have
                    // walls on BOTH perpendicular sides are "no-bypass bottlenecks" —
                    // parking there permanently blocks the blocked agent.
                    Set<Position> noBypassCells = new HashSet<>();
                    if (barrier.clearingOrder.size() >= 2) {
                        Position lastBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 1);
                        Position prevBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 2);
                        int dr = lastBox.row - prevBox.row;
                        int dc = lastBox.col - prevBox.col;
                        Position ext = lastBox;
                        for (int step = 0; step < 20; step++) {
                            ext = Position.of(ext.row + dr, ext.col + dc);
                            if (level.isWall(ext)) break;
                            // Check perpendicular bypass
                            boolean hasBypass;
                            if (dr == 0) { // horizontal extension → check north/south
                                hasBypass = !level.isWall(Position.of(ext.row - 1, ext.col))
                                         || !level.isWall(Position.of(ext.row + 1, ext.col));
                            } else { // vertical extension → check east/west
                                hasBypass = !level.isWall(Position.of(ext.row, ext.col - 1))
                                         || !level.isWall(Position.of(ext.row, ext.col + 1));
                            }
                            if (!hasBypass) {
                                noBypassCells.add(ext);
                            } else {
                                break; // First cell with bypass → corridor opens up
                            }
                        }
                    }
                    
                    Set<Position> tried = new HashSet<>();
                    tried.add(parkPos);
                    int maxRetries = singleBoxBarrier ? 16 : 8;
                    int retryCount = 0;
                    
                    for (Position altTarget : retryCandidates) {
                        if (tried.contains(altTarget)) continue;
                        if (retryCount >= maxRetries) break;
                        tried.add(altTarget);
                        
                        // CORRIDOR BYPASS CHECK: reject cells on barrier extension
                        // that are no-bypass bottlenecks (walls on both sides)
                        if (noBypassCells.contains(altTarget)) {
                            logVerbose("[PP] [CLEAR-BARRIER] BSP retry skip " + altTarget
                                    + " — no-bypass corridor bottleneck");
                            continue;
                        }
                        
                        // TRANSIT CHECK: verify blocked agent can still reach the gap
                        // exit cell after parking at this position. We check gap-exit
                        // (not individual goals) because goals may be behind pushable
                        // boxes that the walk-only BFS can't traverse.
                        if (blockedPos != null && gapExitCell != null) {
                            Set<Position> simObs = new HashSet<>(baseRetryObs);
                            simObs.add(altTarget); // box parked here
                            simObs.remove(boxPos); // current box position freed
                            if (!bfsCanReach(blockedPos, gapExitCell, simObs, level)) {
                                logVerbose("[PP] [CLEAR-BARRIER] BSP retry skip " + altTarget
                                        + " — blocks agent " + barrier.blockedAgentId + " transit to gap");
                                continue;
                            }
                        }
                        
                        retryCount++;
                        logVerbose("[PP] [CLEAR-BARRIER] BSP retry " + retryCount
                                + ": " + boxPos + " -> " + altTarget);
                        
                        Set<Position> retryUnfreeze = new HashSet<>(barrier.clearingOrder);
                        Set<Position> retryProtected = new HashSet<>(retryAvoid);
                        displacePath = boxSearchPlanner.planBoxDisplacementWithUnfreeze(
                                clearingAgent, boxPos, altTarget, barrier.blockingBoxType,
                                currentState, level, retryUnfreeze, clearingBudget,
                                retryProtected);
                        
                        if (displacePath != null && !displacePath.isEmpty()) {
                            parkPos = altTarget;
                            logNormal("[PP] [CLEAR-BARRIER] BSP retry succeeded: "
                                    + boxPos + " -> " + altTarget);
                            break;
                        }
                    }
                }

                if (displacePath == null || displacePath.isEmpty()) {
                    recordFailureSignal(FailureReport.Cause.BARRIER_BSP_EXHAUSTED,
                            Collections.singletonList(boxPos), barrier.clearingOrder);
                    logNormalRepeated(barrierLogKey("bsp-all-targets-exhausted.box" + boxPos, barrier),
                            "[PP] [CLEAR-BARRIER] BSP failed for " + boxPos
                            + " (all targets exhausted)");
                    // First box failure → deeper boxes are certainly harder. Abort this barrier.
                    if (cleared == 0) {
                        logNormalRepeated(barrierLogKey("first-box-failed.box" + boxPos, barrier),
                                "[PP] [CLEAR-BARRIER] First box failed — aborting barrier");
                        skippedBarrierClearingOrders.add(barrier.clearingOrder);
                        earlyExit = true;
                        break;
                    }
                    continue;
                }

                // Execute the displacement path
                for (Action action : displacePath) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            clearingAgent, action, currentState, level, numAgents,
                            false, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(
                            jointAction, currentState, level, clearingAgent);
                    fullPlan.add(jointAction);
                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    globalTimeStep++;
                }

                // POST-DISPLACEMENT VERIFICATION: check if box actually left its position
                // AND reached parking target. Joint action conflict resolution may have 
                // converted some BSP actions to NoOp, leaving the box at an unexpected position.
                Character boxStillThere = currentState.getBoxes().get(boxPos);
                if (boxStillThere != null && boxStillThere == barrier.blockingBoxType) {
                    logNormal("[PP] [CLEAR-BARRIER] Box still at " + boxPos
                            + " after BSP execution (conflict resolution interference) — rolling back");
                    // Rollback: remove the executed actions
                    while (fullPlan.size() > planSizeBefore) {
                        fullPlan.remove(fullPlan.size() - 1);
                        globalTimeStep--;
                    }
                    currentState = recomputeState(initialState, fullPlan, level, numAgents);
                    earlyExit = true;
                    break; // Don't increment cleared, don't track parking
                }
                
                // Verify box reached parking target (not stuck at intermediate position)
                Character boxAtPark = currentState.getBoxes().get(parkPos);
                if (boxAtPark == null || boxAtPark != barrier.blockingBoxType) {
                    // Box left source but didn't reach target — it's stuck at an intermediate
                    // position potentially blocking the approach corridor. Rollback.
                    logNormal("[PP] [CLEAR-BARRIER] Box left " + boxPos + " but didn't reach "
                            + parkPos + " — rolling back to avoid blocking corridor");
                    while (fullPlan.size() > planSizeBefore) {
                        fullPlan.remove(fullPlan.size() - 1);
                        globalTimeStep--;
                    }
                    currentState = recomputeState(initialState, fullPlan, level, numAgents);
                    earlyExit = true;
                    break;
                }

                // Verify previous clears were not undone as collateral damage.
                // Parking is a commitment: later BSP calls must not move an already
                // parked box or refill a cleared barrier cell with the same box type.
                boolean previousClearDisturbed = false;
                for (Position usedParking : usedParkings) {
                    Character parkedBox = currentState.getBoxes().get(usedParking);
                    if (parkedBox == null || parkedBox != barrier.blockingBoxType) {
                        previousClearDisturbed = true;
                        break;
                    }
                }
                if (!previousClearDisturbed) {
                    for (Position clearedPos : clearedPositions) {
                        Character atCleared = currentState.getBoxes().get(clearedPos);
                        if (atCleared != null && atCleared == barrier.blockingBoxType) {
                            previousClearDisturbed = true;
                            break;
                        }
                    }
                }
                if (previousClearDisturbed) {
                    logNormal("[PP] [CLEAR-BARRIER] BSP disturbed a previous clear while moving "
                            + boxPos + " -> " + parkPos + " - rolling back");
                    while (fullPlan.size() > planSizeBefore) {
                        fullPlan.remove(fullPlan.size() - 1);
                        globalTimeStep--;
                    }
                    currentState = recomputeState(initialState, fullPlan, level, numAgents);
                    earlyExit = true;
                    break;
                }

                // Track the displaced goal position
                char goalType = level.getBoxGoal(boxPos);
                if (goalType != '\0') {
                    displacedGoals.add(boxPos);
                }

                // Count the displacement BEFORE reachability check — the box IS moved
                // successfully regardless of whether we can reach the next one.
                clearedPositions.add(boxPos);
                usedParkings.add(parkPos);
                cleared++;
                logNormal("[PP] [CLEAR-BARRIER] Cleared " + barrier.blockingBoxType + " from "
                        + boxPos + " in " + displacePath.size() + " steps (total cleared: " + cleared + ")");

                // REACHABILITY CHECK: Verify the agent can reach the next barrier box.
                // Don't rollback — keep the successful displacement. If the agent can't
                // reach deeper boxes, accept partial clearing and let a second round
                // (or another agent) handle the rest.
                if (i + 1 < barrier.clearingOrder.size()) {
                    Set<Position> postReachable = bfsReachableForClearing(
                            currentState.getAgentPosition(clearingAgent), currentState, level);
                    Position nextBox = barrier.clearingOrder.get(i + 1);
                    boolean canReachNext = false;
                    for (Direction dir : Direction.values()) {
                        if (postReachable.contains(nextBox.move(dir))) {
                            canReachNext = true;
                            break;
                        }
                    }
                    if (!canReachNext) {
                        logNormal("[PP] [CLEAR-BARRIER] Can't reach next box " + nextBox 
                                + " — stopping (reachable=" + postReachable.size() + ")");
                        earlyExit = true;
                        break; // Exit per-box loop
                    }
                }
            }

            if (cleared > 0) {
                logMinimal("[PP] Barrier clearing: " + cleared + "/" + barrier.clearingOrder.size()
                        + " boxes cleared in " + fullPlan.size() + " steps");
            }
            
            // If we couldn't clear all barrier boxes (either pre-assignment limited us
            // or agent got stuck), mark this barrier for this agent so a different
            // agent can attempt the remainder.
            if ((earlyExit || cleared < barrier.clearingOrder.size()) && cleared > 0) {
                List<Position> remaining = barrier.clearingOrder.subList(cleared, barrier.clearingOrder.size());
                skippedBarrierClearingOrders.add(new ArrayList<>(remaining));
                permanentlyFailedBarriers
                        .computeIfAbsent(clearingAgent, k -> new HashSet<>())
                        .add(new HashSet<>(barrier.clearingOrder));
                logNormal("[PP] [CLEAR-BARRIER] Permanently caching barrier for agent " + clearingAgent
                        + " (" + barrier.clearingOrder.size() + " positions) — "
                        + (maxSafeClears < barrier.clearingOrder.size() 
                            ? "pre-assignment limited to " + maxSafeClears 
                            : "structural failure")
                        + " after " + cleared + " clears");
            }
        }

        return currentState;
    }

    /**
     * BFS reachability for clearing agent (treats all boxes as obstacles).
     */
    private Set<Position> bfsReachableForClearing(Position start, State state, Level level) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (state.hasBoxAt(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    /**
     * Adds parking candidates that are reachable by moving the box, not
     * necessarily by the clearing agent walking there before the box is moved.
     * This matters for pull-push Sokoban "door latch" blockers: the box itself
     * can be pushed or pulled into the corridor beyond the blocked cell, while
     * that corridor is absent from the agent's walk-reachable set until the box
     * leaves.
     */
    private void addBoxRayParkingCandidates(List<Position> candidates,
                                            Position boxPos,
                                            Set<Position> avoidPositions,
                                            State state,
                                            Level level,
                                            int maxStepsPerDirection) {
        Set<Position> seen = new HashSet<>(candidates);
        List<Position> rayCandidates = new ArrayList<>();
        for (Direction dir : Direction.values()) {
            Position pos = boxPos;
            for (int step = 1; step <= maxStepsPerDirection; step++) {
                pos = pos.move(dir);
                if (level.isWall(pos)) break;
                if (avoidPositions != null && avoidPositions.contains(pos)) continue;
                if (state.hasAgentAt(pos)) continue;
                if (state.hasBoxAt(pos)) break;
                if (level.getBoxGoal(pos) != '\0') continue;
                if (level.getAgentGoal(pos.row, pos.col) != -1) continue;
                if (seen.add(pos)) {
                    rayCandidates.add(pos);
                }
            }
        }
        candidates.addAll(0, rayCandidates);
    }

    /**
     * Connectivity check for parking selection during barrier clearing.
     * 
     * Simulates placing a box at candidatePark (and removing the box from boxPos),
     * then checks if the agent — assumed to be adjacent to candidatePark after
     * the parking manoeuvre — can still reach the next barrier box.
     * 
     * This prevents parking choices that cut off the agent's return path through
     * the gap corridor (e.g., filling row 10 positions between gap and agent in ZOOM).
     *
     * @param candidatePark  proposed parking position
     * @param boxPos         current position of box being moved (will become free)
     * @param usedParkings   positions already occupied by previously parked boxes
     * @param clearingOrder  full barrier clearing order
     * @param currentIdx     index of current box in clearingOrder
     * @param state          current state
     * @param level          level definition
     * @return true if agent can still reach the next barrier box after parking
     */
    private boolean isGapStillAccessible(Position candidatePark, Position boxPos,
                                          Set<Position> usedParkings, List<Position> clearingOrder,
                                          int currentIdx, State state, Level level) {
        // The next barrier box the agent needs to reach
        Position nextBox = clearingOrder.get(currentIdx + 1);
        
        // Build a simulated obstacle set: current boxes + parked boxes + candidatePark - boxPos
        // Use a lightweight BFS without creating a new State object
        Set<Position> obstacles = new HashSet<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            obstacles.add(entry.getKey());
        }
        obstacles.remove(boxPos);         // box will leave this position
        obstacles.add(candidatePark);     // box will be placed here
        // usedParkings are already in state.getBoxes() (they were parked in earlier iterations)
        
        // Also remove all remaining barrier boxes from obstacles — they WILL be cleared
        // and shouldn't block the reachability check
        for (int j = currentIdx + 1; j < clearingOrder.size(); j++) {
            obstacles.remove(clearingOrder.get(j));
        }
        
        // Agent will be adjacent to candidatePark after parking.
        // BFS from ALL cells adjacent to candidatePark to see if any can reach nextBox.
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        
        // Start BFS from all free cells adjacent to candidatePark (potential agent positions)
        for (Direction dir : Direction.values()) {
            Position adj = candidatePark.move(dir);
            if (!level.isWall(adj) && !obstacles.contains(adj) && !visited.contains(adj)) {
                visited.add(adj);
                queue.add(adj);
            }
        }
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            // Check if we can reach any cell adjacent to the next barrier box
            for (Direction dir : Direction.values()) {
                if (current.move(dir).equals(nextBox)) {
                    return true;
                }
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (obstacles.contains(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        
        return false; // Agent can't reach next barrier box from parking area
    }

    /**
     * Pre-assigns parking positions for ALL barrier boxes using greedy search with
     * backtracking. At each step, verifies that after placement, the agent can still
     * reach the next barrier box through the gap.
     *
     * This prevents the "self-trapping" failure mode where greedy per-box parking
     * fills narrow corridors, cutting off the agent's return path to the gap.
     *
     * @param clearingOrder barrier boxes in clearing order
     * @param candidates    sorted list of ALL parking candidate positions
     * @param currentState  current game state
     * @param level         level definition
     * @param blockedAgentPos position of the blocked agent (null to skip transit check)
     * @param gapExitCell   cell the blocked agent must reach (null to skip transit check)
     * @return list of parking assignments (may be shorter than clearingOrder if no full assignment exists)
     */
    private List<Position> preAssignParkingSlots(
            List<Position> clearingOrder,
            List<Position> candidates,
            State currentState,
            Level level,
            Position blockedAgentPos,
            Position gapExitCell) {

        // Build obstacle set from current box positions
        Set<Position> obstacles = new HashSet<>();
        for (Position boxPos : currentState.getBoxes().keySet()) {
            obstacles.add(boxPos);
        }

        // Try full assignment first
        List<Position> assignment = new ArrayList<>();
        if (doPreAssignRecursive(0, clearingOrder, candidates, assignment, obstacles, level,
                blockedAgentPos, gapExitCell)) {
            return assignment; // Full assignment found
        }

        // Full assignment failed — find maximum feasible prefix
        // Try decreasing sizes until one works
        for (int maxBoxes = clearingOrder.size() - 1; maxBoxes >= 1; maxBoxes--) {
            assignment.clear();
            // Reset obstacles for each attempt
            Set<Position> resetObstacles = new HashSet<>();
            for (Position boxPos : currentState.getBoxes().keySet()) {
                resetObstacles.add(boxPos);
            }
            List<Position> subOrder = clearingOrder.subList(0, maxBoxes);
            if (doPreAssignRecursive(0, subOrder, candidates, assignment, resetObstacles, level,
                    blockedAgentPos, gapExitCell)) {
                logNormal("[PP] [CLEAR-BARRIER] Pre-assignment: safely clearing " + maxBoxes
                        + "/" + clearingOrder.size() + " boxes");
                return assignment;
            }
        }

        return Collections.emptyList(); // Can't even clear 1 box safely
    }

    /**
     * Recursive greedy search with backtracking for parking slot pre-assignment.
     * For each box in clearingOrder, tries candidates in order; for each candidate,
     * simulates placement and checks connectivity to the next barrier box.
     * Backtracks when a candidate breaks future connectivity.
     *
     * @param idx          current box index in clearingOrder
     * @param clearingOrder barrier boxes to clear
     * @param candidates   all parking candidate positions (sorted)
     * @param assigned     accumulator: assigned parking positions so far
     * @param obstacles    simulated obstacle set (mutated in-place, restored on backtrack)
     * @param level        level definition
     * @return true if all remaining boxes can be assigned valid parking
     */
    private boolean doPreAssignRecursive(
            int idx, List<Position> clearingOrder, List<Position> candidates,
            List<Position> assigned, Set<Position> obstacles, Level level,
            Position blockedAgentPos, Position gapExitCell) {

        if (idx >= clearingOrder.size()) {
            // All boxes assigned — verify blocked agent can still reach the gap.
            // After clearing, parked boxes must not cut off the blocked agent's only
            // path to the gap area. Without this check, parking fills all vertical
            // corridors and traps the blocked agent in the lower area.
            if (blockedAgentPos != null && gapExitCell != null) {
                // Build effective obstacles: current + remove all barrier boxes (they'll be cleared)
                Set<Position> effectiveObs = new HashSet<>(obstacles);
                for (Position bp : clearingOrder) {
                    effectiveObs.remove(bp);
                }
                return bfsCanReach(blockedAgentPos, gapExitCell, effectiveObs, level);
            }
            return true;
        }

        Position boxPos = clearingOrder.get(idx);

        for (Position candidate : candidates) {
            // Skip if already occupied or assigned to an earlier box
            if (obstacles.contains(candidate)) continue;
            if (assigned.contains(candidate)) continue;

            // Simulate: box moves from boxPos to candidate
            obstacles.remove(boxPos);
            obstacles.add(candidate);

            // Connectivity check: can agent reach the next barrier box?
            boolean connected;
            if (idx + 1 < clearingOrder.size()) {
                connected = bfsConnectedAfterParking(
                        candidate, clearingOrder.get(idx + 1),
                        obstacles, clearingOrder, idx, level);
            } else {
                connected = true; // Last box — no connectivity requirement
            }

            if (connected) {
                assigned.add(candidate);
                if (doPreAssignRecursive(idx + 1, clearingOrder, candidates,
                        assigned, obstacles, level,
                        blockedAgentPos, gapExitCell)) {
                    return true;
                }
                assigned.remove(assigned.size() - 1);
            }

            // Undo simulation
            obstacles.add(boxPos);
            obstacles.remove(candidate);
        }

        return false;
    }

    /**
     * BFS connectivity check for parking pre-assignment.
     * After hypothetically parking a box at candidatePos (removed from boxPos),
     * checks if the agent (assumed adjacent to candidatePos) can reach any cell
     * adjacent to nextBox.
     *
     * Future barrier boxes (beyond current index) are treated as removable
     * (they will be cleared in later steps).
     */
    private boolean bfsConnectedAfterParking(
            Position candidatePos, Position nextBox,
            Set<Position> obstacles, List<Position> clearingOrder,
            int currentIdx, Level level) {

        // Build effective obstacles: current obstacles minus future barrier boxes
        Set<Position> effectiveObstacles = new HashSet<>(obstacles);
        for (int j = currentIdx + 1; j < clearingOrder.size(); j++) {
            effectiveObstacles.remove(clearingOrder.get(j));
        }

        // BFS from cells adjacent to candidatePos (where agent would be after parking)
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        for (Direction dir : Direction.values()) {
            Position adj = candidatePos.move(dir);
            if (!level.isWall(adj) && !effectiveObstacles.contains(adj)) {
                visited.add(adj);
                queue.add(adj);
            }
        }

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            // Check if we can reach any cell adjacent to nextBox
            for (Direction dir : Direction.values()) {
                if (current.move(dir).equals(nextBox)) return true;
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (effectiveObstacles.contains(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return false;
    }

    /**
     * Simple BFS reachability check: can we walk from 'start' to any cell adjacent
     * to 'target' without passing through walls or obstacles?
     * Used to verify that the blocked agent can still reach the gap area after
     * all barrier boxes are parked.
     */
    private boolean bfsCanReach(Position start, Position target,
                                 Set<Position> obstacles, Level level) {
        if (start.equals(target)) return true;
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        visited.add(start);
        queue.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (next.equals(target)) return true;
                // Also check adjacency to target (agent needs to be NEAR gap, not ON it)
                for (Direction d2 : Direction.values()) {
                    if (next.move(d2).equals(target)) {
                        if (!level.isWall(next) && !obstacles.contains(next)) return true;
                    }
                }
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (obstacles.contains(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return false;
    }

    private List<Position> computeApproachPath(Position start, Position targetBox,
                                                State state, Level level) {
        Map<Position, Position> parent = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        parent.put(start, null);
        queue.add(start);

        Position reached = null;
        while (!queue.isEmpty() && reached == null) {
            Position current = queue.poll();
            // Check if current is adjacent to the target box
            for (Direction dir : Direction.values()) {
                if (current.move(dir).equals(targetBox)) {
                    reached = current;
                    break;
                }
            }
            if (reached != null) break;

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (parent.containsKey(next)) continue;
                if (level.isWall(next)) continue;
                if (state.hasBoxAt(next)) continue;
                parent.put(next, current);
                queue.add(next);
            }
        }

        if (reached == null) return Collections.emptyList();

        // Reconstruct path
        List<Position> path = new ArrayList<>();
        for (Position p = reached; p != null; p = parent.get(p)) {
            path.add(p);
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * MAPF FIX: Validates plan correctness and removes redundant actions.
     * 
     * 1. Collision check: Ensure no two agents occupy same cell at same time
     * 2. Redundant NoOp removal: Remove trailing NoOps when agents are at goals
     */
    private List<Action[]> validateAndOptimizePlan(List<Action[]> plan, State initialState, 
                                                   Level level, int numAgents) {
        if (plan == null || plan.isEmpty()) return plan;
        
        // Step 1: Validate (detect collisions)
        State state = initialState;
        for (int step = 0; step < plan.size(); step++) {
            Action[] actions = plan.get(step);
            
            // Check for vertex conflicts (two agents at same position)
            Set<Position> nextPositions = new HashSet<>();
            for (int a = 0; a < numAgents; a++) {
                Position nextPos = computeNextPosition(state, a, actions[a], level);
                if (nextPositions.contains(nextPos)) {
                    logVerbose("[PP] WARNING: Collision detected at step " + step + " at " + nextPos);
                }
                nextPositions.add(nextPos);
            }
            
            // Apply actions
            state = applyJointAction(actions, state, level, numAgents);
        }
        
        // Step 2: Optimize - remove trailing NoOps
        int lastMeaningfulStep = plan.size() - 1;
        while (lastMeaningfulStep >= 0) {
            Action[] actions = plan.get(lastMeaningfulStep);
            boolean allNoOp = true;
            for (Action a : actions) {
                if (a.type != Action.ActionType.NOOP) {
                    allNoOp = false;
                    break;
                }
            }
            if (!allNoOp) break;
            lastMeaningfulStep--;
        }
        
        if (lastMeaningfulStep < plan.size() - 1) {
            int removed = plan.size() - lastMeaningfulStep - 1;
            logNormal("[PP] Optimized: removed " + removed + " trailing NoOp steps");
            return plan.subList(0, lastMeaningfulStep + 1);
        }
        
        return plan;
    }
    
    /**
     * Computes the next position of an agent after applying an action.
     */
    private Position computeNextPosition(State state, int agentId, Action action, Level level) {
        Position current = state.getAgentPosition(agentId);
        if (action == null || action.type == Action.ActionType.NOOP) {
            return current;
        }
        if (action.type == Action.ActionType.MOVE) {
            return current.move(action.agentDir);
        }
        // Push/Pull: agent moves
        return current.move(action.agentDir);
    }
    
    /**
     * MAPF FIX: Get cached subgoal order or compute once.
     * Priority is fixed at start - only filter out completed goals.
     * UPDATED: Allows refreshing if list is empty but goal state not reached (Phase 1 -> Phase 2).
     */
    private List<Subgoal> getOrComputeSubgoalOrder(State currentState, Level level) {
        if (cachedSubgoalOrder == null) {
            computeAndCacheSubgoals(currentState, level);
        }
        
        // Filter out completed goals from cached order
        List<Subgoal> remaining = filterUnsatisfiedSubgoals(cachedSubgoalOrder, currentState, level);
        
        // Fail-safe: If ran out of goals but not at goal state, refresh!
        // This handles the transition from Box Goals (Phase 1) to Agent Goals (Phase 2)
        // Also: re-enable barrier-displaced goals so CYAN can return A-boxes.
        if (remaining.isEmpty() && !currentState.isGoalState(level)) {
            logVerbose("[PP] Phase switch: refreshing subgoal list");
            if (!displacedGoals.isEmpty()) {
                logVerbose("[PP] Re-enabling " + displacedGoals.size() + " barrier-displaced goals for return phase");
                displacedGoals.clear();
            }
            // Un-suspend TRANSIT goals when nothing else remains: by now their
            // "corridor" dependencies are fulfilled, so filling them won't cause oscillation.
            if (!suspendedTransitGoals.isEmpty()) {
                logNormal("[PP] [TRANSIT] Un-suspending " + suspendedTransitGoals.size()
                        + " transit goals for final placement");
                for (Position p : new ArrayList<>(suspendedTransitGoals)) {
                    completedBoxGoals.remove(p);
                }
                suspendedTransitGoals.clear();
                subgoalManager.invalidateHungarianCache();
            }
            computeAndCacheSubgoals(currentState, level);
            remaining = filterUnsatisfiedSubgoals(cachedSubgoalOrder, currentState, level);
        }
        
        return remaining;
    }
    
    private void computeAndCacheSubgoals(State state, Level level) {
        // Hungarian (SELECTION: which box fills which goal) and dependency analysis
        // (ORDER: what order to fill goals) are ORTHOGONAL concerns — both always active.
        // Hungarian's cost matrix is pure box-to-goal distance with NO execution order
        // assumptions. planSubgoal() has a box-retry fallback: if Hungarian's globally-
        // optimal pick fails BSP (serial execution mismatch), it automatically falls
        // through to greedy Layer 2 candidates.
        subgoalManager.computeHungarianAssignment(state, level, completedBoxGoals);
        
        cachedSubgoalOrder = subgoalManager.getUnsatisfiedSubgoals(state, level, completedBoxGoals);
        sortSubgoals(cachedSubgoalOrder, state, level);

        // F2: filter out suspended box-goals (NAMO blocker-relief co-suspension).
        // These are the original goals of boxes being relocated to P_temp; pushing
        // the box back to its goal would undo the relief and re-block the protected
        // agent. Skipped only for this attempt; agent goals never suspended.
        if (!suspendedBoxGoals.isEmpty()) {
            int beforeSize = cachedSubgoalOrder.size();
            List<Subgoal> kept = new ArrayList<>(beforeSize);
            int removed = 0;
            for (Subgoal sg : cachedSubgoalOrder) {
                if (!sg.isAgentGoal && suspendedBoxGoals.contains(sg.goalPos)) {
                    removed++;
                } else {
                    kept.add(sg);
                }
            }
            if (removed > 0) {
                cachedSubgoalOrder = kept;
                // Log only once per attempt (not per recompute) \u2014 noisy.
            }
        }
        
        // Physical blocking sort: reorder subgoals where filling goal A would place a box
        // on goal B's box-to-goal path. Goal B should be filled BEFORE goal A.
        applyPhysicalBlockingSort(cachedSubgoalOrder, state, level);

        // P0b: stable-partition deprioritized goals (from prior attempts' FailureReport)
        // to the end. They are still attempted, just last — giving other goals a chance
        // to clear physical blockers first.
        if (!deprioritizedGoals.isEmpty()) {
            int beforeSize = cachedSubgoalOrder.size();
            List<Subgoal> head = new ArrayList<>(beforeSize);
            List<Subgoal> tail = new ArrayList<>();
            for (Subgoal sg : cachedSubgoalOrder) {
                if (deprioritizedGoals.contains(sg.goalPos)) tail.add(sg);
                else head.add(sg);
            }
            if (!tail.isEmpty()) {
                cachedSubgoalOrder = head;
                cachedSubgoalOrder.addAll(tail);
                if (mapf.planning.SearchConfig.isVerbose()) {
                    System.err.println("[PP] P0b: demoted " + tail.size() + " goal(s) to end (from prior FailureReport)");
                }
            }
        }

        // P4 (conflict-driven PP): stable-partition prioritized goals to the FRONT.
        // These are blockers identified from the previous attempt's FailureReport —
        // goals that must be satisfied before the failed subgoal can succeed.
        // Applied AFTER deprioritization so a goal can't be both (rare; if so, FRONT wins).
        if (!prioritizedGoals.isEmpty()) {
            List<Subgoal> front = new ArrayList<>();
            List<Subgoal> rest = new ArrayList<>(cachedSubgoalOrder.size());
            for (Subgoal sg : cachedSubgoalOrder) {
                if (prioritizedGoals.contains(sg.goalPos)) front.add(sg);
                else rest.add(sg);
            }
            if (!front.isEmpty()) {
                List<Subgoal> reordered = new ArrayList<>(cachedSubgoalOrder.size());
                reordered.addAll(front);
                reordered.addAll(rest);
                cachedSubgoalOrder = reordered;
                if (mapf.planning.SearchConfig.isVerbose()) {
                    System.err.println("[PP] P4: promoted " + front.size()
                            + " blocker goal(s) to front (from prior FailureReport blockers)");
                }
            }
        }

        // P1: prepend synthetic escape subgoals (highest priority). These break
        // 2-cycles by parking a blocking box at a P_temp before normal subgoals.
        if (!escapeSubgoals.isEmpty()) {
            List<Subgoal> withEscapes = new ArrayList<>(escapeSubgoals.size() + cachedSubgoalOrder.size());
            withEscapes.addAll(escapeSubgoals);
            withEscapes.addAll(cachedSubgoalOrder);
            cachedSubgoalOrder = withEscapes;
            if (mapf.planning.SearchConfig.isVerbose()) {
                System.err.println("[PP] P1: prepended " + escapeSubgoals.size()
                        + " escape subgoal(s) for cycle-breaking");
            }
        }

        // Step 4 (Layer-3): Sort TRANSIT goals to end — fill them last.
        // TRANSIT goals sit on paths other boxes need for delivery; placing them last
        // means by the time we fill them, all other boxes are already at their goals.
        // Exception: goals already in prioritizedGoals keep their promoted position.
        if (!transitProfiles.isEmpty()) {
            List<Subgoal> transitTail = new ArrayList<>();
            List<Subgoal> nonTransit = new ArrayList<>();
            for (Subgoal sg : cachedSubgoalOrder) {
                if (!sg.isAgentGoal && isTransitGoal(sg.goalPos)
                        && !prioritizedGoals.contains(sg.goalPos)) {
                    transitTail.add(sg);
                } else {
                    nonTransit.add(sg);
                }
            }
            if (!transitTail.isEmpty()) {
                // Among TRANSIT goals: fewer crossings first (less disruptive to fill first)
                transitTail.sort((a, b) -> {
                    GoalTransitAnalyzer.GoalProfile pa = transitProfiles.get(a.goalPos);
                    GoalTransitAnalyzer.GoalProfile pb = transitProfiles.get(b.goalPos);
                    int ca = pa != null ? pa.crossingCount : 0;
                    int cb = pb != null ? pb.crossingCount : 0;
                    return Integer.compare(ca, cb);
                });
                nonTransit.addAll(transitTail);
                cachedSubgoalOrder = nonTransit;
                logVerbose("[PP] Step4: deferred " + transitTail.size() + " TRANSIT goals to end");
            }
        }

        logGoalOrder(cachedSubgoalOrder);
    }
    
    private List<Subgoal> filterUnsatisfiedSubgoals(List<Subgoal> source, State currentState, Level level) {
        List<Subgoal> strictRemaining = new ArrayList<>();
        List<Subgoal> allUnsatisfied = new ArrayList<>();
        
        for (Subgoal sg : source) {
            if (sg.isSyntheticRelief() && !reliefCertificateStillBlocked(sg, currentState, level)) {
                continue;
            }
            // Basic satisfaction check
            if (completedBoxGoals.contains(sg.goalPos)) continue;
            // Barrier-displaced goals: boxes intentionally cleared to open a path
            // for a blocked agent. Defer re-filling until the blocked agent's goals
            // are achieved (Phase switch will clear displacedGoals).
            if (!sg.isAgentGoal && displacedGoals.contains(sg.goalPos)) continue;

            boolean satisfied;
            if (sg.isAgentGoal) {
                Position agentPos = currentState.getAgentPosition(sg.agentId);
                satisfied = agentPos.equals(sg.goalPos);
            } else {
                Character boxAtGoal = currentState.getBoxes().get(sg.goalPos);
                satisfied = (boxAtGoal != null && boxAtGoal == sg.boxType);
            }
            if (satisfied) continue;

            allUnsatisfied.add(sg);
            
            // Strict dependency check
            if (areDependenciesMet(sg.goalPos, level)) {
                strictRemaining.add(sg);
            }
        }
        
        // Normal path: some goals have all dependencies met
        if (!strictRemaining.isEmpty()) {
            moveDeferredGoalsToTail(strictRemaining);
            return strictRemaining;
        }
        
        // Cycle fallback: ALL goals have unmet dependencies (circular dependency deadlock).
        // In push-pull domain, no filled goal is truly permanent — boxes can be pulled away.
        // Graceful degradation: return ALL unsatisfied goals, sorted by fewest unmet deps first.
        if (!allUnsatisfied.isEmpty()) {
            allUnsatisfied.sort((a, b) -> {
                int unmetA = countUnmetDependencies(a.goalPos, level);
                int unmetB = countUnmetDependencies(b.goalPos, level);
                return Integer.compare(unmetA, unmetB);
            });
        }
        
        return allUnsatisfied;
    }
    
    /**
     * Counts how many dependencies for a goal are NOT yet satisfied.
     * Used for cycle-fallback ordering: prefer goals with fewer blockers.
     */
    private int countUnmetDependencies(Position goal, Level level) {
        Set<Position> deps = goalDependsOn.get(goal);
        if (deps == null || deps.isEmpty()) return 0;
        int count = 0;
        for (Position dep : deps) {
            if (deferredBlockedGoals.contains(dep)) continue;
            boolean isBoxGoal = level.getBoxGoal(dep) != '\0';
            if (isBoxGoal) {
                if (!completedBoxGoals.contains(dep)) count++;
            } else {
                if (!completedAgentGoals.contains(dep)) count++;
            }
        }
        return count;
    }

    /**
     * Checks if all dependencies for a goal are satisfied.
     * A dependency is satisfied if the prerequisite goal is completed.
     */
    private boolean areDependenciesMet(Position goal, Level level) {
        Set<Position> deps = goalDependsOn.get(goal);
        if (deps == null || deps.isEmpty()) return true;

        for (Position dep : deps) {
            if (deferredBlockedGoals.contains(dep)) continue;
            boolean isBoxGoal = level.getBoxGoal(dep) != '\0';
            if (isBoxGoal) {
                if (!completedBoxGoals.contains(dep)) return false;
            } else {
                // Agent dependency: less common, but check if satisfied
                // Note: completedAgentGoals tracks *ever* completed, but agent might move.
                // However, for dependency logic, we assume "Completed" means "Done".
                if (!completedAgentGoals.contains(dep)) return false;
            }
        }
        return true;
    }

    private void moveDeferredGoalsToTail(List<Subgoal> subgoals) {
        if (deferredBlockedGoals.isEmpty() || subgoals.size() < 2) return;
        List<Subgoal> active = new ArrayList<>(subgoals.size());
        List<Subgoal> deferred = new ArrayList<>();
        for (Subgoal sg : subgoals) {
            if (deferredBlockedGoals.contains(sg.goalPos)) deferred.add(sg);
            else active.add(sg);
        }
        if (!deferred.isEmpty() && !active.isEmpty()) {
            subgoals.clear();
            subgoals.addAll(active);
            subgoals.addAll(deferred);
        }
    }
    
    /** Sort subgoals based on the current OrderingMode. */
    private void sortSubgoals(List<Subgoal> subgoals, State state, Level level) {
        switch (orderingMode) {
            case REVERSE_TOPOLOGICAL:
                if (precomputedGoalOrder != null && !precomputedGoalOrder.isEmpty()) {
                    Map<Position, Integer> orderMap = new HashMap<>();
                    for (int i = 0; i < precomputedGoalOrder.size(); i++) {
                        orderMap.put(precomputedGoalOrder.get(i), i);
                    }
                    // Reverse: higher index first
                    subgoals.sort((a, b) -> {
                        int orderA = orderMap.getOrDefault(a.goalPos, -1);
                        int orderB = orderMap.getOrDefault(b.goalPos, -1);
                        return Integer.compare(orderB, orderA);
                    });
                    logVerbose("[PP] Sorted subgoals in REVERSE topological order");
                } else {
                    sortByDifficulty(subgoals, state, level);
                }
                break;
                
            case DISTANCE_GREEDY:
                // Sort by estimated difficulty: easiest (nearest) first
                sortByDifficulty(subgoals, state, level);
                logVerbose("[PP] Sorted subgoals by DISTANCE_GREEDY (nearest first)");
                break;
                
            case DISTANCE_FARTHEST:
                // Sort by estimated difficulty: HARDEST (farthest) first
                // Critical for spiral topologies: outer goals must be done before inner goals
                // block narrow corridors with cross-color immovable boxes.
                sortByDifficultyDescending(subgoals, state, level);
                logVerbose("[PP] Sorted subgoals by DISTANCE_FARTHEST (hardest first)");
                break;
                
            case RANDOM:
                Collections.shuffle(subgoals, random);
                logVerbose("[PP] Sorted subgoals in RANDOM order");
                break;
                
            case TOPOLOGICAL:
            default:
                if (precomputedGoalOrder != null && !precomputedGoalOrder.isEmpty()) {
                    Map<Position, Integer> orderMap = new HashMap<>();
                    for (int i = 0; i < precomputedGoalOrder.size(); i++) {
                        orderMap.put(precomputedGoalOrder.get(i), i);
                    }
                    
                    subgoals.sort((a, b) -> {
                        int orderA = orderMap.getOrDefault(a.goalPos, Integer.MAX_VALUE);
                        int orderB = orderMap.getOrDefault(b.goalPos, Integer.MAX_VALUE);
                        return Integer.compare(orderA, orderB);
                    });
                } else {
                    sortByDifficulty(subgoals, state, level);
                }
                break;
        }
    }
    
    /** Sort subgoals by estimated difficulty (easiest first). */
    private void sortByDifficulty(List<Subgoal> subgoals, State state, Level level) {
        final State s = state;
        final Level lv = level;
        subgoals.sort((a, b) -> {
            int diffA = subgoalManager.estimateSubgoalDifficulty(a, s, lv, completedBoxGoals);
            int diffB = subgoalManager.estimateSubgoalDifficulty(b, s, lv, completedBoxGoals);
            return Integer.compare(diffA, diffB);
        });
    }
    
    /** Sort subgoals by estimated difficulty (HARDEST first — for spiral/corridor topologies). */
    private void sortByDifficultyDescending(List<Subgoal> subgoals, State state, Level level) {
        final State s = state;
        final Level lv = level;
        subgoals.sort((a, b) -> {
            int diffA = subgoalManager.estimateSubgoalDifficulty(a, s, lv, completedBoxGoals);
            int diffB = subgoalManager.estimateSubgoalDifficulty(b, s, lv, completedBoxGoals);
            // Handle MAX_VALUE: both infinite → equal, otherwise finite < infinite
            if (diffA == Integer.MAX_VALUE && diffB == Integer.MAX_VALUE) return 0;
            if (diffA == Integer.MAX_VALUE) return 1;
            if (diffB == Integer.MAX_VALUE) return -1;
            return Integer.compare(diffB, diffA); // Descending
        });
    }
    
    /**
     * Computes a dynamic BSP search budget based on estimated box-to-goal distance.
     * Uses heuristic true distance (BFS-precomputed wall-aware) when available,
     * falling back to Manhattan distance. This is critical for non-trivial topologies
     * like spirals where actual path length can be 3-5x Manhattan distance.
     * 
     * Formula: clamp(MIN_BSP_BUDGET + distance * BSP_BUDGET_PER_DISTANCE, MIN, MAX)
     */
    private int computeDynamicBspBudget(Position boxPos, Position goalPos) {
        int distance = boxPos.manhattanDistance(goalPos);
        // Try true distance from heuristic if available (much more accurate for spirals/corridors)
        if (heuristic instanceof mapf.planning.heuristic.TrueDistanceHeuristic) {
            int trueDistance = ((mapf.planning.heuristic.TrueDistanceHeuristic) heuristic).getDistance(boxPos, goalPos);
            if (trueDistance < Integer.MAX_VALUE) {
                distance = trueDistance;
            }
        }
        int budget = SearchConfig.MIN_BSP_BUDGET + distance * SearchConfig.BSP_BUDGET_PER_DISTANCE;
        return Math.max(SearchConfig.MIN_BSP_BUDGET, Math.min(effectiveMaxBspBudget, budget));
    }
    
    /**
     * Physical blocking sort: detects cross-color physical blocking between subgoals
     * and reorders to avoid deadlocks.
     * 
     * For each pair (A, B) of box subgoals handled by DIFFERENT color agents:
     *   If A's goalPos lies on B's box→goal path, then B should be done BEFORE A
     *   (because filling A would block B's path).
     * 
     * Uses bubble-sort style promotion: if B is blocked by A and B comes after A,
     * swap them. Only applies to cross-color pairs (same-color handled by dependency graph).
     * Limited to O(n²) pairwise checks with max n² swaps.
     */
    private void applyPhysicalBlockingSort(List<Subgoal> subgoals, State state, Level level) {
        int n = subgoals.size();
        if (n < 2) return;
        
        // Build box→goal paths for each subgoal (lazily cached)
        Map<Integer, Set<Position>> pathCells = new HashMap<>();
        
        boolean changed = true;
        int passes = 0;
        int maxPasses = n; // limit iterations
        
        while (changed && passes < maxPasses) {
            changed = false;
            passes++;
            
            for (int i = 0; i < n - 1; i++) {
                Subgoal a = subgoals.get(i);
                if (a.isAgentGoal) continue;
                
                Color colorA = level.getAgentColor(a.agentId);
                
                for (int j = i + 1; j < n; j++) {
                    Subgoal b = subgoals.get(j);
                    if (b.isAgentGoal) continue;
                    
                    Color colorB = level.getAgentColor(b.agentId);
                    
                    // Only check cross-color pairs
                    if (colorA != null && colorA.equals(colorB)) continue;
                    
                    // Does A's goalPos block B's box→goal path?
                    Set<Position> bPath = pathCells.computeIfAbsent(j, idx -> {
                        Position boxPos = subgoalManager.findBestBoxForGoal(b, state, level, subgoals, completedBoxGoals);
                        if (boxPos == null) return Collections.emptySet();
                        List<Position> path = pathAnalyzer.findPathIgnoringDynamicObstacles(boxPos, b.goalPos, level);
                        return (path != null) ? new HashSet<>(path) : Collections.emptySet();
                    });
                    
                    if (bPath.contains(a.goalPos)) {
                        // A's goal blocks B's path → B should be done before A
                        // Promote B to position i (move A down)
                        subgoals.remove(j);
                        subgoals.add(i, b);
                        pathCells.clear(); // invalidate path cache after reorder
                        changed = true;
                        logVerbose("[PP] [PHYS-SORT] Promoted " + b.boxType + "->" + b.goalPos 
                                + " before " + a.boxType + "->" + a.goalPos + " (cross-color blocking)");
                        break; // restart inner loop from new i
                    }
                }
                if (changed) break; // restart outer loop
            }
        }
    }
    
    /** Log the goal execution order. Internal planning detail, verbose only. */
    private void logGoalOrder(List<Subgoal> subgoals) {
        if (!SearchConfig.isVerbose()) return;
        System.err.println("[PP] Task Plan (" + subgoals.size() + " subgoals):");
        for (int i = 0; i < subgoals.size(); i++) {
            Subgoal sg = subgoals.get(i);
            String taskType = sg.isAgentGoal ? "AgentGoal" : ("Box " + sg.boxType);
            System.err.println("  " + (i+1) + ". " + taskType + " -> " + sg.goalPos + " [Agent " + sg.agentId + "]");
        }
    }

    /** Try to execute subgoals in priority order. Returns true if progress made. */
    private boolean tryExecuteSubgoals(List<Subgoal> subgoals, List<Action[]> fullPlan, 
            State currentState, Level level, int numAgents, State initialState) {
        
        int skippedByDeps = 0;
        for (Subgoal subgoal : subgoals) {
            // MAPF FIX: Strictly enforce goal dependencies.
            // Even if a lower-priority goal is locally executable, we must not execute it
            // if it depends on a higher-priority goal that is not yet complete.
            if (goalDependsOn.containsKey(subgoal.goalPos)) {
                boolean depMet = true;
                for (Position dep : goalDependsOn.get(subgoal.goalPos)) {
                    if (deferredBlockedGoals.contains(dep)) continue;
                    boolean isBoxG = level.getBoxGoal(dep.row, dep.col) != '\0';
                    if (isBoxG && !completedBoxGoals.contains(dep)) {
                        depMet = false;
                        break;
                    } else if (!isBoxG && !completedAgentGoals.contains(dep)) {
                        depMet = false;
                        break;
                    }
                }
                if (!depMet) {
                    logVerbose("[PP] Skipped " + (subgoal.isAgentGoal ? "Agent" : "Box " + subgoal.boxType)
                            + " -> " + subgoal.goalPos + " (unmet deps)");
                    skippedByDeps++; continue;
                }
            }

            // Goal-level cycle detection: skip goals that have been completed too many times.
            // This breaks push-pull cycles where a box is pushed to goal, pulled off by
            // another agent's recovery, then pushed back again indefinitely.
            if (!subgoal.isAgentGoal) {
                int completions = goalCompletionCount.getOrDefault(subgoal.goalPos, 0);
                if (completions >= MAX_GOAL_COMPLETIONS) {
                    logVerbose("[PP] [CYCLE-SKIP] Goal " + subgoal.goalPos + " (box " + subgoal.boxType
                            + ") already completed " + completions + " times — skipping to break cycle");
                    continue;
                }
            }

            // TRAP blacklist: skip goals that previously triggered agent-trap detection.
            // This prevents the infinite TRAP→rollback→retry→TRAP cycle.
            if (!subgoal.isAgentGoal && trapBlacklist.contains(subgoal.goalPos)) {
                logVerbose("[PP] [TRAP-SKIP] " + subgoal.boxType + " -> " + subgoal.goalPos
                        + " — blacklisted from prior TRAP rollback");
                continue;
            }

            if (isSyntheticBoxTarget(subgoal, level) && syntheticReliefBlacklist.contains(subgoal.goalPos)) {
                logVerbose("[PP] [SYNTHETIC-RELIEF-SKIP] " + subgoal.boxType + " -> " + subgoal.goalPos
                        + " rejected earlier in this attempt");
                continue;
            }

            if (subgoal.isSyntheticRelief() && !reliefCertificateStillBlocked(subgoal, currentState, level)) {
                String cert = reliefCertificateLabel(subgoal);
                logNormalRepeated("synthetic.skip.certificate-resolved " + cert,
                        "[PP][SYNTHETIC-RELIEF-SKIP] subgoal=" + subgoalLabel(subgoal)
                        + " reason=certificate-already-resolved cert=" + reliefCertificateLabel(subgoal));
                continue;
            }

            // REGRESS blacklist: skip subgoals that repeatedly cause regression rollbacks.
            // After MAX_REGRESS_RETRIES failures for the same subgoal, the path through completed goals is
            // structurally unavoidable for this ordering — stop wasting time.
            // (Kept for future use, currently handled by unprotectable goal approach below)

            // P0a: record the subgoal currently being attempted for the failure-report path.
            lastAttemptedSubgoalForReport = subgoal;

            // Task-Aware: Pass the full list of subgoals for global allocation checking
            List<Action> path = planSubgoal(subgoal, currentState, level, subgoals);
            
            if (path != null && !path.isEmpty()) {
                
                // Snapshot plan size for potential rollback (agent-trap detection)
                int planSizeBefore = fullPlan.size();
                SubgoalEval beforeEval = captureSubgoalEval(subgoal, currentState, level);
                
                // Plan parallel subgoals for agents in independent components
                planIndependentAgents(subgoal, subgoals, currentState, level);
                
                // Record agent path in reservation table for space-time collision avoidance
                List<Position> agentPath = extractAgentPath(subgoal.agentId, currentState, path, level);
                boolean permanentEnd = subgoal.isAgentGoal;
                reservationTable.reservePath(subgoal.agentId, agentPath, globalTimeStep, permanentEnd);
                
                // Execute the path
                State tempState = currentState;
                for (Action action : path) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            subgoal.agentId, action, tempState, level, numAgents, subgoal.isAgentGoal, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, subgoal.agentId);
                    fullPlan.add(jointAction);
                    tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    globalTimeStep++;
                    planMerger.updatePlanIndexes(jointAction, numAgents, subgoal.agentId);
                }
                
                // POST-PLAN REGRESSION GUARD: Check if executing this path disturbed
                // any previously completed box goals. If so, either accept or rollback.
                // "Break-then-rebuild" tolerance: if only 1 goal regressed and the current
                // subgoal was successfully placed, accept the regression — the regressed goal
                // will be re-planned in a later iteration. This enables progress through
                // chokepoints where temporarily displacing a completed goal is unavoidable.
                // If 2+ goals regressed or the subgoal wasn't reached, rollback as before.
                if (!completedBoxGoals.isEmpty()) {
                    List<Position> regressedGoals = detectRegressedGoals(tempState, level);
                    // Exclude goals that were intentionally displaced by tryRecovery.
                    // Those positions are expected to be empty — not a regression.
                    regressedGoals.removeAll(displacedGoals);
                    if (!regressedGoals.isEmpty()) {
                        // Check if we can accept this regression (break-then-rebuild)
                        boolean subgoalReached = !subgoal.isAgentGoal && verifyGoalReached(subgoal, tempState, level);
                        int satisfiedBefore = countSatisfiedBoxGoals(currentState, level);
                        int satisfiedAfter = countSatisfiedBoxGoals(tempState, level);
                        boolean netGoalGain = satisfiedAfter > satisfiedBefore;
                        if (regressedGoals.size() <= 1 && subgoalReached && netGoalGain) {
                            // REGRESS-ACCEPT: 1 goal regressed but current subgoal succeeded.
                            // Accept the trade-off — remove regressed goal from completed set
                            // so it gets re-planned in a future iteration.
                            Position regressedPos = regressedGoals.get(0);
                            logNormal(getName() + ": [REGRESS-ACCEPT] " 
                                    + subgoal.boxType + " -> " + subgoal.goalPos
                                    + " disturbed " + regressedPos + " — accepting (will re-plan)");
                            if (isTransitGoal(regressedPos)) {
                                suspendedTransitGoals.add(regressedPos);
                                logVerbose("[PP] [REGRESS-SUSPEND] TRANSIT goal " + regressedPos + " suspended");
                            } else {
                                completedBoxGoals.remove(regressedPos);
                            }
                            int disturbCount = regressDisturbCount.merge(regressedPos, 1, Integer::sum);
                            if (disturbCount >= 2 && !suspendedTransitGoals.contains(regressedPos)) {
                                completedBoxGoals.remove(regressedPos); // ensure un-frozen so it can be re-delivered
                                suspendedTransitGoals.add(regressedPos);
                                logVerbose("[PP] [REGRESS-SUSPEND-EXCESS] goal " + regressedPos
                                        + " suspended after " + disturbCount + " disturbances");
                            }
                            cachedSubgoalOrder = null;
                            subgoalManager.invalidateHungarianCache();
                            // Fall through to normal goal verification and completion
                        } else {
                            // Original rollback: 2+ regressions or subgoal not reached
                            logVerbose(getName() + ": [REGRESS] Path for " 
                                    + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType)
                                    + " -> " + subgoal.goalPos + " disturbed " + regressedGoals.size() 
                                    + " completed goal(s): " + regressedGoals + " — rollback");
                            // Track disturbance count per completed goal.
                            // If a completed goal is disturbed too many times, it's structurally
                            // in a chokepoint — remove it from completedBoxGoals so future
                            // paths aren't blocked by trying to protect it.
                            for (Position rg : regressedGoals) {
                                int count = regressDisturbCount.merge(rg, 1, Integer::sum);
                                if (count >= MAX_REGRESS_PER_GOAL) {
                                    if (isTransitGoal(rg)) {
                                        suspendedTransitGoals.add(rg);
                                        logNormal(getName() + ": [REGRESS-SUSPEND] TRANSIT goal " + rg
                                                + " disturbed " + count + " times — suspending");
                                    } else {
                                        logNormal(getName() + ": [REGRESS-UNPROTECT] Goal " + rg
                                                + " disturbed " + count + " times — marking unprotectable");
                                        completedBoxGoals.remove(rg);
                                    }
                                }
                            }
                            // Rollback: remove the executed actions from fullPlan
                            while (fullPlan.size() > planSizeBefore) {
                                fullPlan.remove(fullPlan.size() - 1);
                                globalTimeStep--;
                            }
                            // Invalidate stored plans — state rolled back
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            continue; // try next subgoal in priority order
                        }
                    }
                }
                
                // Check if any parallel (stored plan) agents completed their goals
                checkStoredPlanCompletions(tempState, level);
                
                // Verify goal was reached
                boolean reached = verifyGoalReached(subgoal, tempState, level);
                if (reached) {
                    // Pukoban agent-trap detection: after filling a box goal, check if the 
                    // agent is trapped in a dead-end disconnected from remaining tasks.
                    // This is a POST-PLAN validation (not static analysis) because it depends
                    // on the actual agent position and world state after execution.
                    if (!subgoal.isAgentGoal && !isSyntheticBoxTarget(subgoal, level)
                            && wouldTrapAgent(subgoal, tempState, level, subgoals, currentState)) {
                        // Agent would be trapped — skip this subgoal, let PP try alternatives
                        logNormal(getName() + ": [TRAP] Skipping " + subgoal.boxType + " -> " + subgoal.goalPos
                                + " — agent " + subgoal.agentId + " would be trapped after filling");
                        // Blacklist this goal to prevent repeated TRAP→rollback→retry cycles
                        trapBlacklist.add(subgoal.goalPos);
                        // Rollback: remove the executed actions from fullPlan
                        while (fullPlan.size() > planSizeBefore) {
                            fullPlan.remove(fullPlan.size() - 1);
                            globalTimeStep--;
                        }
                        // Invalidate stored plans — state rolled back
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                        continue; // try next subgoal in priority order
                    }
                    SealRisk sealRisk = findSealRisk(subgoal, currentState, tempState, level, subgoals);
                    if (sealRisk != null) {
                        rollbackPlanTo(fullPlan, planSizeBefore);
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                        State stagedState = tryStageForSealRisk("NORMAL", subgoal, sealRisk,
                                currentState, level, fullPlan, numAgents);
                        if (stagedState != null) {
                            cachedSubgoalOrder = null;
                            subgoalManager.invalidateHungarianCache();
                            lastProgressWasPhantom = false;
                            return true;
                        }
                        continue;
                    }
                    if (!acceptReachedSubgoal("NORMAL", subgoal, beforeEval, tempState, level,
                            planSizeBefore, fullPlan.size(), subgoals)) {
                        rollbackPlanTo(fullPlan, planSizeBefore);
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                        continue;
                    }
                    logAcceptedSubgoalEval("NORMAL", subgoal, beforeEval, tempState, level,
                            planSizeBefore, fullPlan.size());
                    
                    // Mark box goal as completed
                    if (!subgoal.isAgentGoal) {
                        completedBoxGoals.add(subgoal.goalPos);
                        deferredBlockedGoals.remove(subgoal.goalPos);
                        // NOTE: displacedGoals NOT cleared here. Barrier-displaced goals
                        // stay deferred until the phase switch (all non-displaced goals
                        // satisfied) to prevent PP from undoing barrier clearing.
                        
                        // Goal-level cycle detection: track completion count
                        int count = goalCompletionCount.getOrDefault(subgoal.goalPos, 0) + 1;
                        goalCompletionCount.put(subgoal.goalPos, count);
                        if (count > MAX_GOAL_COMPLETIONS) {
                            logVerbose("[PP] CYCLE DETECTED: goal " + subgoal.goalPos 
                                    + " completed " + count + " times (box " + subgoal.boxType + ")");
                        }
                        
                        // Invalidate Hungarian cache — world state changed, assignment may be stale
                        subgoalManager.invalidateHungarianCache();
                        // Force subgoal list refresh: completing a box goal may make new agent 
                        // goals eligible (agents whose same-color box tasks are now all done).
                        // Without this, filterUnsatisfiedSubgoals can only REMOVE completed goals 
                        // from the cache — it cannot ADD newly-eligible agent goals that were 
                        // excluded during initial cache computation.
                        cachedSubgoalOrder = null;
                    }
                    
                    // Check if any previously completed goals were disturbed (soft-unlock)
                    revalidateCompletedGoals(tempState, level);
                    
                    // Proactive Yielding: park agent at safe position after completing subgoal.
                    // This prevents agents from blocking corridors/chokepoints for later subgoals.
                    // CRITICAL FIX: Do NOT park an agent that just completed its own agent goal.
                    // Parking moves it off the goal position, causing it to be re-added to
                    // unsatisfied subgoals next iteration → infinite loop (park ↔ re-solve).
                    if (!subgoal.isAgentGoal) {
                        // Pass remaining subgoals for task-aware critical path checking
                        List<Subgoal> remainingSubgoals = new ArrayList<>();
                        for (Subgoal sg : subgoals) {
                            if (sg != subgoal) remainingSubgoals.add(sg);
                        }
                        tempState = parkAgentAfterSubgoal(subgoal.agentId, tempState, level, fullPlan, numAgents, remainingSubgoals);
                    }
                    
                    // Track phantom progress: re-solving an agent goal that was already done
                    if (subgoal.isAgentGoal) {
                        lastProgressWasPhantom = completedAgentGoals.contains(subgoal.goalPos);
                        completedAgentGoals.add(subgoal.goalPos);
                    } else {
                        lastProgressWasPhantom = false;
                    }
                    
                    // PHANTOM = re-solving completed agent goal (debug info, verbose only)
                    if (lastProgressWasPhantom) {
                        logVerbose(getName() + ": [OK] " + 
                            (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) +
                            " -> " + subgoal.goalPos + " (" + path.size() + " steps) [PHANTOM]");
                    } else {
                        // Routine subgoal success: verbose only (one line per subgoal
                        // is too noisy for default level). Final "Goal state reached"
                        // is still printed at minimal level.
                        logVerbose(getName() + ": [OK] " + 
                            (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) +
                            " -> " + subgoal.goalPos + " (" + path.size() + " steps)");
                    }
                    return true;
                }
                logNormal(getName() + ": [EXEC-MISMATCH] "
                        + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType)
                        + " -> " + subgoal.goalPos
                        + " path executed but goal not reached - rollback");
                while (fullPlan.size() > planSizeBefore) {
                    fullPlan.remove(fullPlan.size() - 1);
                    globalTimeStep--;
                }
                planMerger.clearAllPlans();
                storedPlanSubgoals.clear();
                continue;
            } else {
                diagPlanSubgoalNull++;
                int supportPlanSizeBefore = -1;
                State supportStateBefore = null;
                Set<Position> supportDisplacedBefore = null;
                // PATH CLEARING: When BSP fails for a box subgoal, other boxes may
                // physically block the path. Try to identify and move them.
                // Example: In MADS, box C at (10,3) blocks box A's path to (10,1).
                // Agent 1 (red) can't push C (pink), so BSP fails. We need agent 2
                // (pink) to move C out of the way first.
                boolean allowSupportForSubgoal = !isSyntheticBoxTarget(subgoal, level)
                        || isEscapeSubgoal(subgoal);
                if (allowSupportForSubgoal) {
                    int reliefPlanSizeBefore = fullPlan.size();
                    State reliefStateBefore = currentState;
                    Set<Position> reliefDisplacedBefore = new HashSet<>(displacedGoals);
                    State relievedState = tryTaskConditionedBlockerRelief(subgoal, currentState, level,
                            fullPlan, numAgents, subgoals);
                    if (relievedState != null) {
                        path = planSubgoal(subgoal, relievedState, level, subgoals);
                        if (path != null && !path.isEmpty()) {
                            currentState = relievedState;
                            supportPlanSizeBefore = reliefPlanSizeBefore;
                            supportStateBefore = reliefStateBefore;
                            supportDisplacedBefore = reliefDisplacedBefore;
                        } else if (fullPlan.size() > reliefPlanSizeBefore) {
                            // Treat accepted blocker movement as a first-class support subgoal.
                            // The parent task may still need another support move in the next
                            // iteration, but rolling this back loses real monotonic progress.
                            lastProgressWasPhantom = false;
                            return true;
                        } else {
                            rollbackPlanTo(fullPlan, reliefPlanSizeBefore);
                            restoreDisplacedGoals(reliefDisplacedBefore);
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                        }
                    }
                }

                if (!isSyntheticBoxTarget(subgoal, level) && (path == null || path.isEmpty())) {
                    int reliefPlanSizeBefore = fullPlan.size();
                    State reliefStateBefore = currentState;
                    Set<Position> reliefDisplacedBefore = new HashSet<>(displacedGoals);
                    State relievedState = tryScopedSyntheticBlockerRelief(subgoal, currentState, level,
                            fullPlan, numAgents, subgoals);
                    if (relievedState != null) {
                        if (fullPlan.size() > reliefPlanSizeBefore) {
                            // Scoped NAMO relief can be a useful support transaction even
                            // when the parent task still needs another clearing step.
                            lastProgressWasPhantom = false;
                            return true;
                        }
                        path = planSubgoal(subgoal, relievedState, level, subgoals);
                        if (path != null && !path.isEmpty()) {
                            currentState = relievedState;
                            supportPlanSizeBefore = reliefPlanSizeBefore;
                            supportStateBefore = reliefStateBefore;
                            supportDisplacedBefore = reliefDisplacedBefore;
                        } else {
                            rollbackPlanTo(fullPlan, reliefPlanSizeBefore);
                            restoreDisplacedGoals(reliefDisplacedBefore);
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                        }
                    }
                }

                if (!subgoal.isAgentGoal && (path == null || path.isEmpty())) {
                    int clearingPlanSizeBefore = fullPlan.size();
                    State clearingStateBefore = currentState;
                    Set<Position> clearingDisplacedBefore = new HashSet<>(displacedGoals);
                    State clearedState = tryPathClearing(subgoal, currentState, level, 
                            fullPlan, numAgents, subgoals);
                    if (clearedState != null) {
                        // Clearing moved some blocking boxes. Retry planSubgoal.
                        path = planSubgoal(subgoal, clearedState, level, subgoals);
                        if (path != null && !path.isEmpty()) {
                            diagPathClearingRescued++;
                            logVerbose("[PP] [CLEAR] Path found after clearing: " + path.size() + " steps");
                            currentState = clearedState;
                            supportPlanSizeBefore = clearingPlanSizeBefore;
                            supportStateBefore = clearingStateBefore;
                            supportDisplacedBefore = clearingDisplacedBefore;
                            // Fall through to the execution block below
                        } else {
                            rollbackPlanTo(fullPlan, clearingPlanSizeBefore);
                            restoreDisplacedGoals(clearingDisplacedBefore);
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                        }
                    }
                }
                
                // Execute path if clearing succeeded
                if (path != null && !path.isEmpty()) {
                    int planSizeBefore = fullPlan.size();
                    int transactionPlanSizeBefore = supportPlanSizeBefore >= 0
                            ? supportPlanSizeBefore : planSizeBefore;
                    State transactionStateBefore = supportStateBefore != null
                            ? supportStateBefore : currentState;
                    Set<Position> transactionDisplacedBefore = supportDisplacedBefore != null
                            ? supportDisplacedBefore : new HashSet<>(displacedGoals);
                    Set<Position> transactionCompletedBefore = new HashSet<>(completedBoxGoals);
                    Map<Position, Integer> transactionGoalCountBefore = new HashMap<>(goalCompletionCount);
                    SubgoalEval beforeEval = captureSubgoalEval(subgoal, currentState, level);
                    List<Position> agentPath = extractAgentPath(subgoal.agentId, currentState, path, level);
                    reservationTable.reservePath(subgoal.agentId, agentPath, globalTimeStep, subgoal.isAgentGoal);
                    
                    State tempState = currentState;
                    for (Action action : path) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                                subgoal.agentId, action, tempState, level, numAgents, subgoal.isAgentGoal, completedBoxGoals);
                        jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, subgoal.agentId);
                        fullPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                        globalTimeStep++;
                        planMerger.updatePlanIndexes(jointAction, numAgents, subgoal.agentId);
                    }
                    
                    // Same post-plan checks as normal path
                    if (!completedBoxGoals.isEmpty()) {
                        List<Position> regressedGoals = detectRegressedGoals(tempState, level);
                        regressedGoals.removeAll(displacedGoals);
                        if (!regressedGoals.isEmpty()) {
                            logVerbose(getName() + ": [REGRESS] Cleared path disturbed " 
                                    + regressedGoals.size() + " goal(s) — rollback");
                            rollbackPlanTo(fullPlan, transactionPlanSizeBefore);
                            restoreTransactionBookkeeping(transactionDisplacedBefore,
                                    transactionCompletedBefore, transactionGoalCountBefore);
                            currentState = transactionStateBefore;
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            continue;
                        }
                    }
                    
                    boolean reached = verifyGoalReached(subgoal, tempState, level);
                    if (reached) {
                        SealRisk sealRisk = findSealRisk(subgoal, transactionStateBefore, tempState, level, subgoals);
                        if (sealRisk != null) {
                            rollbackPlanTo(fullPlan, transactionPlanSizeBefore);
                            restoreTransactionBookkeeping(transactionDisplacedBefore,
                                    transactionCompletedBefore, transactionGoalCountBefore);
                            currentState = transactionStateBefore;
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            State stagedState = tryStageForSealRisk("CLEARED", subgoal, sealRisk,
                                    transactionStateBefore, level, fullPlan, numAgents);
                            if (stagedState != null) {
                                cachedSubgoalOrder = null;
                                subgoalManager.invalidateHungarianCache();
                                lastProgressWasPhantom = false;
                                return true;
                            }
                            continue;
                        }
                        if (!acceptReachedSubgoal("CLEARED", subgoal, beforeEval, tempState, level,
                                planSizeBefore, fullPlan.size(), subgoals)) {
                            rollbackPlanTo(fullPlan, transactionPlanSizeBefore);
                            restoreTransactionBookkeeping(transactionDisplacedBefore,
                                    transactionCompletedBefore, transactionGoalCountBefore);
                            currentState = transactionStateBefore;
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            continue;
                        }
                        logAcceptedSubgoalEval("CLEARED", subgoal, beforeEval, tempState, level,
                                planSizeBefore, fullPlan.size());
                        if (!subgoal.isAgentGoal) {
                            completedBoxGoals.add(subgoal.goalPos);
                            deferredBlockedGoals.remove(subgoal.goalPos);
                            // NOTE: displacedGoals NOT cleared here — see phase switch.
                            int count = goalCompletionCount.getOrDefault(subgoal.goalPos, 0) + 1;
                            goalCompletionCount.put(subgoal.goalPos, count);
                            subgoalManager.invalidateHungarianCache();
                            cachedSubgoalOrder = null;
                        }
                        // Borrow-and-return is part of the same transaction:
                        // the parent subgoal is not committed unless every
                        // borrowed completed goal is restored.
                        List<Position> borrowedGoalsToRestore = new ArrayList<>(lastClearingDisplacedGoals);
                        if (!borrowedGoalsToRestore.isEmpty()) {
                            tempState = returnDisplacedGoals(tempState, level, fullPlan, numAgents);
                            if (!borrowedGoalsRestored(borrowedGoalsToRestore, tempState, level)) {
                                logNormal(getName() + ": [BORROW-RETURN] Failed to restore "
                                        + borrowedGoalsToRestore + " after "
                                        + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType)
                                        + " -> " + subgoal.goalPos + " - rollback");
                                rollbackPlanTo(fullPlan, transactionPlanSizeBefore);
                                restoreTransactionBookkeeping(transactionDisplacedBefore,
                                        transactionCompletedBefore, transactionGoalCountBefore);
                                currentState = transactionStateBefore;
                                planMerger.clearAllPlans();
                                storedPlanSubgoals.clear();
                                continue;
                            }
                        }
                        revalidateCompletedGoals(tempState, level);
                        
                        if (!subgoal.isAgentGoal) {
                            List<Subgoal> rem = new ArrayList<>();
                            for (Subgoal sg : subgoals) { if (sg != subgoal) rem.add(sg); }
                            parkAgentAfterSubgoal(subgoal.agentId, tempState, level, fullPlan, numAgents, rem);
                        }
                        lastProgressWasPhantom = subgoal.isAgentGoal && completedAgentGoals.contains(subgoal.goalPos);
                        if (subgoal.isAgentGoal) completedAgentGoals.add(subgoal.goalPos);
                        else lastProgressWasPhantom = false;
                        logVerbose(getName() + ": [OK] " + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) 
                                + " -> " + subgoal.goalPos + " (" + path.size() + " steps) [CLEARED]");
                        return true;
                    }
                    logNormal(getName() + ": [EXEC-MISMATCH] Cleared path for "
                            + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType)
                            + " -> " + subgoal.goalPos
                            + " executed but goal not reached - rollback");
                    rollbackPlanTo(fullPlan, transactionPlanSizeBefore);
                    restoreTransactionBookkeeping(transactionDisplacedBefore,
                            transactionCompletedBefore, transactionGoalCountBefore);
                    currentState = transactionStateBefore;
                    planMerger.clearAllPlans();
                    storedPlanSubgoals.clear();
                    continue;
                }

                deferBlockedGoal(subgoal, currentState, level, subgoals);
            }
        }
        // ============ CYCLE-AWARE DEPENDENCY RELAXATION ============
        // When ALL remaining subgoals are blocked by unmet dependencies, we have a
        // dependency cycle (every goal depends on some other unfulfilled goal).
        // In this case, strict enforcement creates a complete deadlock.
        // Break the cycle by doing a second pass that IGNORES dependency constraints,
        // allowing the "cheapest" subgoal to proceed. This is safe because:
        // - The dependency is about transient blocking (agent body / box in corridor)
        // - Once one goal completes, it breaks the cycle for others
        // - The subgoal execution itself handles conflicts via BSP + conflict resolver
        if (skippedByDeps > 0 && skippedByDeps >= subgoals.size()) {
            logVerbose("[PP] [CYCLE-BREAK] All " + skippedByDeps + " subgoals blocked by deps — relaxing constraints");
            for (Subgoal subgoal : subgoals) {
                // Skip cycle/trap/completion blacklists (still enforced)
                if (!subgoal.isAgentGoal) {
                    int completions = goalCompletionCount.getOrDefault(subgoal.goalPos, 0);
                    if (completions >= MAX_GOAL_COMPLETIONS) continue;
                }
                if (!subgoal.isAgentGoal && trapBlacklist.contains(subgoal.goalPos)) continue;
                if (isSyntheticBoxTarget(subgoal, level) && syntheticReliefBlacklist.contains(subgoal.goalPos)) continue;

                List<Action> path = planSubgoal(subgoal, currentState, level, subgoals);
                if (path != null && !path.isEmpty()) {
                    int planSizeBefore = fullPlan.size();
                    SubgoalEval beforeEval = captureSubgoalEval(subgoal, currentState, level);
                    planIndependentAgents(subgoal, subgoals, currentState, level);
                    List<Position> agentPath = extractAgentPath(subgoal.agentId, currentState, path, level);
                    reservationTable.reservePath(subgoal.agentId, agentPath, globalTimeStep, subgoal.isAgentGoal);

                    State tempState = currentState;
                    for (Action action : path) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                                subgoal.agentId, action, tempState, level, numAgents, subgoal.isAgentGoal, completedBoxGoals);
                        jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, subgoal.agentId);
                        fullPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                        globalTimeStep++;
                        planMerger.updatePlanIndexes(jointAction, numAgents, subgoal.agentId);
                    }

                    // Regression check (same logic as normal path)
                    if (!completedBoxGoals.isEmpty()) {
                        List<Position> regressedGoals = detectRegressedGoals(tempState, level);
                        regressedGoals.removeAll(displacedGoals);
                        if (!regressedGoals.isEmpty()) {
                            boolean subgoalReached = !subgoal.isAgentGoal && verifyGoalReached(subgoal, tempState, level);
                            int satisfiedBefore = countSatisfiedBoxGoals(currentState, level);
                            int satisfiedAfter = countSatisfiedBoxGoals(tempState, level);
                            boolean netGoalGain = satisfiedAfter > satisfiedBefore;
                            if (regressedGoals.size() <= 1 && subgoalReached && netGoalGain) {
                                Position regressedPos = regressedGoals.get(0);
                                logNormal(getName() + ": [CYCLE-BREAK][REGRESS-ACCEPT] " 
                                        + subgoal.boxType + " -> " + subgoal.goalPos
                                        + " disturbed " + regressedPos + " — accepting");
                                if (isTransitGoal(regressedPos)) {
                                    suspendedTransitGoals.add(regressedPos);
                                    logVerbose("[PP] [CYCLE-BREAK][REGRESS-SUSPEND] TRANSIT goal " + regressedPos + " suspended");
                                } else {
                                    completedBoxGoals.remove(regressedPos);
                                }
                                int cbDisturbCount = regressDisturbCount.merge(regressedPos, 1, Integer::sum);
                                if (cbDisturbCount >= 2 && !suspendedTransitGoals.contains(regressedPos)) {
                                    completedBoxGoals.remove(regressedPos);
                                    suspendedTransitGoals.add(regressedPos);
                                }
                                cachedSubgoalOrder = null;
                                subgoalManager.invalidateHungarianCache();
                            } else {
                                logVerbose("[PP] [CYCLE-BREAK] Regression rollback for " + subgoal.goalPos
                                        + (subgoalReached && !netGoalGain
                                                ? " (no net satisfied-goal gain)" : ""));
                                for (Position rg : regressedGoals) {
                                    int count = regressDisturbCount.merge(rg, 1, Integer::sum);
                                    if (count >= MAX_REGRESS_PER_GOAL) {
                                        if (isTransitGoal(rg)) {
                                            suspendedTransitGoals.add(rg);
                                        } else {
                                            completedBoxGoals.remove(rg);
                                        }
                                    }
                                }
                                while (fullPlan.size() > planSizeBefore) {
                                    fullPlan.remove(fullPlan.size() - 1);
                                    globalTimeStep--;
                                }
                                planMerger.clearAllPlans();
                                storedPlanSubgoals.clear();
                                continue;
                            }
                        }
                    }

                    checkStoredPlanCompletions(tempState, level);
                    boolean reached = verifyGoalReached(subgoal, tempState, level);
                    if (reached) {
                        if (!subgoal.isAgentGoal && !isSyntheticBoxTarget(subgoal, level)
                                && wouldTrapAgent(subgoal, tempState, level, subgoals, currentState)) {
                            logNormal(getName() + ": [CYCLE-BREAK][TRAP] " + subgoal.boxType + " -> " + subgoal.goalPos);
                            trapBlacklist.add(subgoal.goalPos);
                            while (fullPlan.size() > planSizeBefore) {
                                fullPlan.remove(fullPlan.size() - 1);
                                globalTimeStep--;
                            }
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            continue;
                        }
                        SealRisk sealRisk = findSealRisk(subgoal, currentState, tempState, level, subgoals);
                        if (sealRisk != null) {
                            rollbackPlanTo(fullPlan, planSizeBefore);
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            State stagedState = tryStageForSealRisk("CYCLE-BREAK", subgoal, sealRisk,
                                    currentState, level, fullPlan, numAgents);
                            if (stagedState != null) {
                                cachedSubgoalOrder = null;
                                subgoalManager.invalidateHungarianCache();
                                lastProgressWasPhantom = false;
                                return true;
                            }
                            continue;
                        }
                        if (!acceptReachedSubgoal("CYCLE-BREAK", subgoal, beforeEval, tempState, level,
                                planSizeBefore, fullPlan.size(), subgoals)) {
                            rollbackPlanTo(fullPlan, planSizeBefore);
                            planMerger.clearAllPlans();
                            storedPlanSubgoals.clear();
                            continue;
                        }
                        logAcceptedSubgoalEval("CYCLE-BREAK", subgoal, beforeEval, tempState, level,
                                planSizeBefore, fullPlan.size());
                        if (!subgoal.isAgentGoal) {
                            completedBoxGoals.add(subgoal.goalPos);
                            int count = goalCompletionCount.getOrDefault(subgoal.goalPos, 0) + 1;
                            goalCompletionCount.put(subgoal.goalPos, count);
                            subgoalManager.invalidateHungarianCache();
                            cachedSubgoalOrder = null;
                        }
                        revalidateCompletedGoals(tempState, level);
                        if (!subgoal.isAgentGoal) {
                            List<Subgoal> rem = new ArrayList<>();
                            for (Subgoal sg : subgoals) { if (sg != subgoal) rem.add(sg); }
                            parkAgentAfterSubgoal(subgoal.agentId, tempState, level, fullPlan, numAgents, rem);
                        }
                        lastProgressWasPhantom = subgoal.isAgentGoal && completedAgentGoals.contains(subgoal.goalPos);
                        if (subgoal.isAgentGoal) completedAgentGoals.add(subgoal.goalPos);
                        else lastProgressWasPhantom = false;
                        logVerbose(getName() + ": [OK] " + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType) 
                                + " -> " + subgoal.goalPos + " (" + path.size() + " steps) [CYCLE-BREAK]");
                        return true;
                    }
                    logNormal(getName() + ": [CYCLE-BREAK][EXEC-MISMATCH] "
                            + (subgoal.isAgentGoal ? "Agent " + subgoal.agentId : "Box " + subgoal.boxType)
                            + " -> " + subgoal.goalPos
                            + " executed but goal not reached - rollback");
                    while (fullPlan.size() > planSizeBefore) {
                        fullPlan.remove(fullPlan.size() - 1);
                        globalTimeStep--;
                    }
                    planMerger.clearAllPlans();
                    storedPlanSubgoals.clear();
                    continue;
                }
            }
            logVerbose("[PP] [CYCLE-BREAK] Second pass also failed — no executable subgoal found");
        }

        logVerbose("[PP] tryExecuteSubgoals: no progress (skippedByDeps=" + skippedByDeps 
                + ", total=" + subgoals.size() + ")");
        return false;
    }

    private void deferBlockedGoal(Subgoal subgoal, State state, Level level,
                                  List<Subgoal> allSubgoals) {
        if (subgoal == null || subgoal.goalPos == null) return;
        if (subgoal.isSyntheticRelief() || isSyntheticBoxTarget(subgoal, level)) return;
        if (deferredBlockedGoals.add(subgoal.goalPos)) {
            List<Position> blockers = findAccessBlockersForTask(subgoal, state, level, allSubgoals);
            recordFailureSignal(subgoal.isAgentGoal
                            ? FailureReport.Cause.AGENT_GOAL_BLOCKED
                            : FailureReport.Cause.BOX_GOAL_BLOCKED,
                    Collections.singletonList(subgoal.goalPos), blockers);
            logVerbose("[PP][DEFER] " + subgoalLabel(subgoal)
                    + " remains blocked after relief/clearing; softening as prerequisite"
                    + (blockers.isEmpty() ? "" : " blockers=" + blockers));
            cachedSubgoalOrder = null;
        }
    }
    
    private State tryTaskConditionedBlockerRelief(Subgoal blockedSubgoal, State state, Level level,
            List<Action[]> fullPlan, int numAgents, List<Subgoal> allSubgoals) {
        if (isSyntheticBoxTarget(blockedSubgoal, level) && !isEscapeSubgoal(blockedSubgoal)) {
            return null;
        }

        int initialPlanSize = fullPlan.size();
        int initialTime = globalTimeStep;
        State current = state;
        int moved = 0;
        String supportKey = taskSupportKey(blockedSubgoal);
        Set<Position> usedTemps = new HashSet<>(
                taskSupportForbidden.getOrDefault(supportKey, Collections.emptySet()));
        Set<Position> taskCritical = taskAccessCells(blockedSubgoal, current, level, allSubgoals);
        List<Position> initialBlockers = findAccessBlockersForTask(
                blockedSubgoal, current, level, allSubgoals);

        for (int round = 0; round < MAX_TASK_RELIEF_MOVES; round++) {
            List<Position> blockers = findAccessBlockersForTask(blockedSubgoal, current, level, allSubgoals);
            if (blockers.isEmpty()) {
                break;
            }

            recordFailureSignal(blockedSubgoal.isAgentGoal
                            ? FailureReport.Cause.AGENT_GOAL_BLOCKED
                            : FailureReport.Cause.BOX_GOAL_BLOCKED,
                    Collections.singletonList(blockedSubgoal.goalPos), blockers);
            logVerbose("[PP][TASK-RELIEF] " + subgoalLabel(blockedSubgoal)
                    + " blocked by " + blockers + " - trying local relief"
                    + " (round=" + (round + 1) + ")");

            boolean movedThisRound = false;
            int blockerCountBefore = blockers.size();

            for (Position blockerPos : blockers) {
                if (moved >= MAX_TASK_RELIEF_MOVES) break;
                Character blockerType = current.getBoxes().get(blockerPos);
                if (blockerType == null) {
                    int blockingAgent = current.getAgentAt(blockerPos);
                    if (blockingAgent >= 0 && blockingAgent != blockedSubgoal.agentId) {
                        int planSizeBefore = fullPlan.size();
                        int timeBefore = globalTimeStep;
                        State parked = parkSupportAgentOffCritical(
                                blockingAgent, current, level, fullPlan, numAgents, taskCritical, usedTemps);
                        if (parked != current) {
                            int blockerCountAfter = findAccessBlockersForTask(
                                    blockedSubgoal, parked, level, allSubgoals).size();
                            boolean madeTaskProgress = blockerCountAfter < blockerCountBefore
                                    || hasTaskAccess(blockedSubgoal, parked, level, allSubgoals);
                            if (madeTaskProgress) {
                                current = parked;
                                moved++;
                                movedThisRound = true;
                                logVerbose("[PP][TASK-RELIEF] parked agent" + blockingAgent
                                        + " off " + subgoalLabel(blockedSubgoal)
                                        + " blockers=" + blockerCountBefore + "->" + blockerCountAfter);
                                break;
                            }
                        }
                        rollbackPlanTo(fullPlan, planSizeBefore);
                        globalTimeStep = timeBefore;
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                    }
                    continue;
                }

                int helper = findHelperAgentForBox(blockerType, current, level, blockerPos);
                if (helper < 0) continue;

                int reliefBudget = Math.min(effectiveMaxBspBudget * 2,
                        Math.max(SearchConfig.MIN_BSP_BUDGET * 4,
                                computeDynamicBspBudget(blockerPos, blockedSubgoal.goalPos) * 2));
                Set<Position> unfreeze = new HashSet<>();
                unfreeze.add(blockerPos);
                Set<Position> releaseForbidden = new HashSet<>(taskCritical);
                releaseForbidden.addAll(allGoalCells(level));
                releaseForbidden.addAll(usedTemps);
                releaseForbidden.add(blockerPos);

                int attempted = 0;
                int bspFailed = 0;
                int validationFailed = 0;
                int skippedNogood = 0;
                boolean movedThisBlocker = false;

                int planSizeBeforeRelease = fullPlan.size();
                int timeBeforeRelease = globalTimeStep;
                List<Action> releasePath = boxSearchPlanner.planBoxReleaseFromForbidden(
                        helper, blockerPos, blockerType, current, level,
                        releaseForbidden, unfreeze, reliefBudget, taskCritical);
                attempted++;
                if (releasePath != null && !releasePath.isEmpty()) {
                    State beforeRelease = current;
                    State trial = appendReliefPath(releasePath, helper, current, level, fullPlan, numAgents);
                    Position releasedPos = findRelocatedBoxPosition(beforeRelease, trial, blockerType, blockerPos);
                    int blockerCountAfter = findAccessBlockersForTask(
                            blockedSubgoal, trial, level, allSubgoals).size();
                    boolean movedBlocker = releasedPos != null && !releasedPos.equals(blockerPos)
                            && !trial.getBoxes().containsKey(blockerPos);
                    boolean madeTaskProgress = blockerCountAfter < blockerCountBefore
                            || hasTaskAccess(blockedSubgoal, trial, level, allSubgoals);

                    if (movedBlocker && madeTaskProgress) {
                        current = trial;
                        usedTemps.add(releasedPos);
                        usedTemps.add(blockerPos);
                        moved++;
                        movedThisBlocker = true;
                        movedThisRound = true;
                        logVerbose("[PP][TASK-RELIEF] released blocker "
                                + blockerType + " " + blockerPos + " -> " + releasedPos
                                + " for " + subgoalLabel(blockedSubgoal)
                                + " blockers=" + blockerCountBefore + "->" + blockerCountAfter);
                    } else {
                        rollbackPlanTo(fullPlan, planSizeBeforeRelease);
                        globalTimeStep = timeBeforeRelease;
                        planMerger.clearAllPlans();
                        storedPlanSubgoals.clear();
                        validationFailed++;
                    }
                } else {
                    bspFailed++;
                }

                if (movedThisBlocker) {
                    if (movedThisRound) break;
                    continue;
                }

                List<Position> parkingCandidates = findTaskReliefParkingCandidates(
                        blockerPos, current, level, taskCritical, usedTemps);
                for (Position pTemp : parkingCandidates) {
                    String nogoodKey = taskReliefNogoodKey(blockedSubgoal, blockerPos, pTemp);
                    if (taskReliefNogoods.contains(nogoodKey)) {
                        skippedNogood++;
                        continue;
                    }

                    int planSizeBefore = fullPlan.size();
                    int timeBefore = globalTimeStep;
                    List<Action> reliefPath = boxSearchPlanner.planBoxDisplacementWithUnfreeze(
                            helper, blockerPos, pTemp, blockerType, current, level,
                            unfreeze, reliefBudget, taskCritical);
                    attempted++;
                    if (reliefPath == null || reliefPath.isEmpty()) {
                        taskReliefNogoods.add(nogoodKey);
                        bspFailed++;
                        continue;
                    }

                    State trial = appendReliefPath(reliefPath, helper, current, level, fullPlan, numAgents);

                    Character atTemp = trial.getBoxes().get(pTemp);
                    int blockerCountAfter = findAccessBlockersForTask(
                            blockedSubgoal, trial, level, allSubgoals).size();
                    boolean movedBlocker = atTemp != null && atTemp == blockerType
                            && !trial.getBoxes().containsKey(blockerPos);
                    boolean madeTaskProgress = blockerCountAfter < blockerCountBefore
                            || hasTaskAccess(blockedSubgoal, trial, level, allSubgoals);

                    if (movedBlocker && madeTaskProgress) {
                        current = trial;
                        usedTemps.add(pTemp);
                        usedTemps.add(blockerPos);
                        moved++;
                        movedThisBlocker = true;
                        movedThisRound = true;
                        logVerbose("[PP][TASK-RELIEF] moved blocker "
                                + blockerType + " " + blockerPos + " -> " + pTemp
                                + " for " + subgoalLabel(blockedSubgoal)
                                + " blockers=" + blockerCountBefore + "->" + blockerCountAfter);
                        break;
                    }

                    rollbackPlanTo(fullPlan, planSizeBefore);
                    globalTimeStep = timeBefore;
                    planMerger.clearAllPlans();
                    storedPlanSubgoals.clear();
                    taskReliefNogoods.add(nogoodKey);
                    validationFailed++;
                }

                if (!movedThisBlocker) {
                    State helperAccessState = tryClearHelperAccessForBlocker(
                            blockedSubgoal, blockerPos, blockerType, helper, current,
                            level, fullPlan, numAgents, taskCritical, usedTemps);
                    if (helperAccessState != null) {
                        current = helperAccessState;
                        moved++;
                        movedThisBlocker = true;
                        movedThisRound = true;
                        logNormal("[PP][SUPPORT-CHAIN] cleared helper access for "
                                + blockerType + " " + blockerPos
                                + " toward " + subgoalLabel(blockedSubgoal));
                    }
                }

                if (!movedThisBlocker) {
                    logVerbose("[PP][TASK-RELIEF] failed blocker "
                            + blockerType + " at " + blockerPos
                            + " for " + subgoalLabel(blockedSubgoal)
                            + " candidates=" + parkingCandidates.size()
                            + " attempted=" + attempted
                            + " bspFailed=" + bspFailed
                            + " validationFailed=" + validationFailed
                            + " skippedNogood=" + skippedNogood);
                }

                if (movedThisRound) break;
            }

            if (!movedThisRound) {
                break;
            }
            taskCritical = taskAccessCells(blockedSubgoal, current, level, allSubgoals);
        }

        if (moved == 0) return null;
        List<Position> remainingBlockers = findAccessBlockersForTask(blockedSubgoal, current, level, allSubgoals);
        if (!remainingBlockers.isEmpty()) {
            boolean reducedBlockers = remainingBlockers.size() < initialBlockers.size();
            boolean accessOpened = hasTaskAccess(blockedSubgoal, current, level, allSubgoals);
            if (!reducedBlockers && !accessOpened) {
                rollbackPlanTo(fullPlan, initialPlanSize);
                globalTimeStep = initialTime;
                planMerger.clearAllPlans();
                storedPlanSubgoals.clear();
                logNormal("[PP][TASK-RELIEF] incomplete relief for " + subgoalLabel(blockedSubgoal)
                        + " remaining=" + remainingBlockers + " -- rollback partial relief");
                return null;
            }
            logNormal("[PP][SUPPORT-PROGRESS] " + subgoalLabel(blockedSubgoal)
                    + " blockers=" + initialBlockers.size() + "->" + remainingBlockers.size()
                    + " remaining=" + remainingBlockers);
        }
        subgoalManager.invalidateHungarianCache();
        taskSupportForbidden.put(supportKey, new HashSet<>(usedTemps));
        cachedSubgoalOrder = null;
        return current;
    }

    private State tryClearHelperAccessForBlocker(Subgoal parentTask, Position targetBoxPos,
            char targetBoxType, int targetAgent, State current, Level level,
            List<Action[]> fullPlan, int numAgents, Set<Position> parentCritical,
            Set<Position> usedTemps) {
        return tryClearHelperAccessForBlocker(parentTask, targetBoxPos, targetBoxType,
                targetAgent, current, level, fullPlan, numAgents, parentCritical,
                usedTemps, 0);
    }

    private State tryClearHelperAccessForBlocker(Subgoal parentTask, Position targetBoxPos,
            char targetBoxType, int targetAgent, State current, Level level,
            List<Action[]> fullPlan, int numAgents, Set<Position> parentCritical,
            Set<Position> usedTemps, int depth) {
        if (depth >= MAX_SUPPORT_CHAIN_DEPTH) return null;

        List<Position> initialAccessBlockers = findHelperAccessBlockersForBox(
                targetAgent, targetBoxPos, current, level);
        if (initialAccessBlockers.isEmpty()) {
            return null;
        }

        int entryPlanSize = fullPlan.size();
        int entryTime = globalTimeStep;
        State working = current;

        for (int pass = 0; pass < MAX_TASK_RELIEF_MOVES; pass++) {
            Position targetAgentPos = working.getAgentPosition(targetAgent);
            if (targetAgentPos == null) break;

            Set<Position> targetReachable = strictAgentReachable(targetAgentPos, working, level);
            if (hasReachableNeighbor(targetBoxPos, targetReachable, level)) {
                return working == current ? null : working;
            }

            Color targetAgentColor = level.getAgentColor(targetAgent);
            LinkedHashSet<Position> supportCritical = new LinkedHashSet<>(parentCritical);
            LinkedHashSet<Position> accessBlockers = new LinkedHashSet<>();

            for (Position accessCell : operationCells(targetBoxPos, level)) {
                supportCritical.addAll(findStaticPathCellsToAccessCell(
                        targetAgentPos, accessCell, working, level));
                List<Position> cellBlockers = findAgentPathBlockersToAccessCell(
                        targetAgentPos, accessCell, working, level, targetAgentColor, targetBoxPos);
                accessBlockers.addAll(cellBlockers);
            }
            accessBlockers.remove(targetBoxPos);

            LinkedHashSet<Integer> agentBlockers = findHelperAccessAgentBlockers(
                    targetAgent, targetBoxPos, working, level);
            boolean advancedPrecondition = false;
            for (int blockingAgent : agentBlockers) {
                State parked = parkSupportAgentOffCritical(
                        blockingAgent, working, level, fullPlan, numAgents, supportCritical, usedTemps);
                if (parked != working) {
                    working = parked;
                    advancedPrecondition = true;
                    break;
                }
            }
            if (advancedPrecondition) continue;

            if (accessBlockers.isEmpty()) break;

            for (Position accessBlocker : accessBlockers) {
                Character accessBlockerType = working.getBoxes().get(accessBlocker);
                if (accessBlockerType == null) continue;

                int accessHelper = findHelperAgentForBox(accessBlockerType, working, level, accessBlocker);
                if (accessHelper < 0) continue;

                int planSizeBefore = fullPlan.size();
                int timeBefore = globalTimeStep;
                State before = working;

                Set<Position> forbidden = new HashSet<>(supportCritical);
                forbidden.addAll(allGoalCells(level));
                forbidden.addAll(usedTemps);
                forbidden.add(accessBlocker);
                forbidden.add(targetBoxPos);

                Set<Position> unfreeze = new HashSet<>();
                unfreeze.add(accessBlocker);

                int budget = Math.min(effectiveMaxBspBudget * 2,
                        Math.max(SearchConfig.MIN_BSP_BUDGET * 4,
                                computeDynamicBspBudget(accessBlocker, targetBoxPos) * 2));
                List<Action> releasePath = boxSearchPlanner.planBoxReleaseFromForbidden(
                        accessHelper, accessBlocker, accessBlockerType, working, level,
                        forbidden, unfreeze, budget, supportCritical);
                if (releasePath == null || releasePath.isEmpty()) {
                    State nested = tryClearHelperAccessForBlocker(parentTask, accessBlocker,
                            accessBlockerType, accessHelper, working, level, fullPlan,
                            numAgents, supportCritical, usedTemps, depth + 1);
                    if (nested != null) {
                        working = nested;
                        advancedPrecondition = true;
                        break;
                    }
                    continue;
                }

                State trial = appendReliefPath(releasePath, accessHelper, working, level, fullPlan, numAgents);
                Position releasedPos = findRelocatedBoxPosition(before, trial, accessBlockerType, accessBlocker);
                trial = parkSupportAgentOffCritical(
                        accessHelper, trial, level, fullPlan, numAgents, supportCritical, usedTemps);
                boolean movedAccessBlocker = releasedPos != null && !releasedPos.equals(accessBlocker)
                        && !trial.getBoxes().containsKey(accessBlocker);
                boolean helperCanNowReach = hasReachableNeighbor(targetBoxPos,
                        strictAgentReachable(trial.getAgentPosition(targetAgent), trial, level), level);
                int remainingAccessBlockers = findHelperAccessBlockersForBox(
                        targetAgent, targetBoxPos, trial, level).size();
                boolean reducedHelperBlockers = remainingAccessBlockers < accessBlockers.size();

                if (movedAccessBlocker && (helperCanNowReach || reducedHelperBlockers)) {
                    if (releasedPos != null) usedTemps.add(releasedPos);
                    usedTemps.add(accessBlocker);
                    return trial;
                }

                rollbackPlanTo(fullPlan, planSizeBefore);
                globalTimeStep = timeBefore;
                planMerger.clearAllPlans();
                storedPlanSubgoals.clear();
            }

            if (!advancedPrecondition) break;
        }

        if (working != current) {
            boolean helperCanNowReach = hasReachableNeighbor(targetBoxPos,
                    strictAgentReachable(working.getAgentPosition(targetAgent), working, level), level);
            int remainingAccessBlockers = findHelperAccessBlockersForBox(
                    targetAgent, targetBoxPos, working, level).size();
            if (helperCanNowReach || remainingAccessBlockers < initialAccessBlockers.size()) {
                return working;
            }

            rollbackPlanTo(fullPlan, entryPlanSize);
            globalTimeStep = entryTime;
            planMerger.clearAllPlans();
            storedPlanSubgoals.clear();
        }
        return null;
    }

    private List<Position> findHelperAccessBlockersForBox(int targetAgent, Position targetBoxPos,
            State state, Level level) {
        Position targetAgentPos = state.getAgentPosition(targetAgent);
        Color targetAgentColor = level.getAgentColor(targetAgent);
        LinkedHashSet<Position> accessBlockers = new LinkedHashSet<>();
        for (Position accessCell : operationCells(targetBoxPos, level)) {
            List<Position> cellBlockers = findAgentPathBlockersToAccessCell(
                    targetAgentPos, accessCell, state, level, targetAgentColor, targetBoxPos);
            accessBlockers.addAll(cellBlockers);
        }
        accessBlockers.remove(targetBoxPos);
        return new ArrayList<>(accessBlockers);
    }

    private LinkedHashSet<Integer> findHelperAccessAgentBlockers(int targetAgent, Position targetBoxPos,
            State state, Level level) {
        Position targetAgentPos = state.getAgentPosition(targetAgent);
        LinkedHashSet<Integer> agentBlockers = new LinkedHashSet<>();
        for (Position accessCell : operationCells(targetBoxPos, level)) {
            List<Integer> cellBlockers = findAgentPathAgentBlockersToAccessCell(
                    targetAgentPos, accessCell, state, level, targetAgent);
            Collections.reverse(cellBlockers);
            agentBlockers.addAll(cellBlockers);
        }
        agentBlockers.remove(targetAgent);
        return agentBlockers;
    }

    private List<Integer> findAgentPathAgentBlockersToAccessCell(Position start, Position target,
            State state, Level level, int targetAgent) {
        if (start == null || target == null || level.isWall(target)) return Collections.emptyList();

        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Position> parent = new HashMap<>();
        queue.add(start);
        visited.add(start);

        boolean found = false;
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            if (current.equals(target)) {
                found = true;
                break;
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || immovableBoxes.contains(next)) continue;
                visited.add(next);
                parent.put(next, current);
                queue.add(next);
            }
        }

        if (!found) return Collections.emptyList();

        LinkedHashSet<Integer> blockers = new LinkedHashSet<>();
        Position current = target;
        while (current != null && !current.equals(start)) {
            int agentAt = state.getAgentAt(current);
            if (agentAt >= 0 && agentAt != targetAgent) {
                blockers.add(agentAt);
            }
            current = parent.get(current);
        }
        return new ArrayList<>(blockers);
    }

    private State parkSupportAgentOffCritical(int agentId, State state, Level level,
            List<Action[]> fullPlan, int numAgents, Set<Position> critical,
            Set<Position> usedTemps) {
        Position agentPos = state.getAgentPosition(agentId);
        if (agentPos == null || !critical.contains(agentPos)) return state;

        Set<Position> parkingForbidden = new HashSet<>(critical);
        parkingForbidden.addAll(completedBoxGoals);
        parkingForbidden.addAll(completedAgentGoals);
        parkingForbidden.addAll(usedTemps);

        Position parking = findSupportParkingPosition(
                agentId, state, level, numAgents, parkingForbidden);
        if (parking == null || parking.equals(agentPos)) return state;

        List<Action> parkPath = pathAnalyzer.planAgentPathWithFallback(
                agentId, parking, state, level, numAgents, boxSearchPlanner);
        if (parkPath == null || parkPath.isEmpty() || parkPath.size() > 20) return state;

        State parked = appendReliefPath(parkPath, agentId, state, level, fullPlan, numAgents);
        logVerbose("[PP][SUPPORT-PARK] agent" + agentId + " " + agentPos + " -> " + parking);
        return parked;
    }

    private Position findSupportParkingPosition(int agentId, State state, Level level,
            int numAgents, Set<Position> forbidden) {
        Position agentPos = state.getAgentPosition(agentId);
        if (agentPos == null) return null;

        Set<Position> occupiedAgents = new HashSet<>();
        for (int i = 0; i < numAgents; i++) {
            occupiedAgents.add(state.getAgentPosition(i));
        }

        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Integer> dist = new HashMap<>();
        queue.add(agentPos);
        visited.add(agentPos);
        dist.put(agentPos, 0);

        Position best = null;
        int bestScore = Integer.MIN_VALUE;
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            int d = dist.get(current);
            if (d > 30) continue;

            if (!current.equals(agentPos)
                    && !level.isWall(current)
                    && !state.hasBoxAt(current)
                    && !occupiedAgents.contains(current)
                    && !forbidden.contains(current)) {
                int score = -d * 3;
                int passable = pathAnalyzer.countPassableNeighbors(current, level);
                if (passable == 1) score += 8;
                else if (passable == 2) score -= 3;
                if (pathAnalyzer.getArticulationPoints().contains(current)) score -= 5;
                if (score > bestScore) {
                    bestScore = score;
                    best = current;
                }
            }

            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || state.hasBoxAt(next)) continue;
                if (occupiedAgents.contains(next) && !next.equals(agentPos)) continue;
                visited.add(next);
                dist.put(next, d + 1);
                queue.add(next);
            }
        }
        return best;
    }

    private State tryScopedSyntheticBlockerRelief(Subgoal blockedSubgoal, State state, Level level,
            List<Action[]> fullPlan, int numAgents, List<Subgoal> allSubgoals) {
        if (isSyntheticBoxTarget(blockedSubgoal, level)) {
            return null;
        }

        int initialPlanSize = fullPlan.size();
        int initialTime = globalTimeStep;
        State current = state;
        int moved = 0;
        boolean madeAnyTaskProgress = false;

        BlockerReliefSynthesizer.ReliefResult reliefResult =
                BlockerReliefSynthesizer.synthesizeWithMeta(current, level, immovableBoxes);
        if (reliefResult.reliefs.isEmpty()) {
            return null;
        }

        Set<Subgoal> selected = new LinkedHashSet<>();
        for (Subgoal relief : reliefResult.reliefs) {
            if (reliefTargetsSubgoal(relief, blockedSubgoal, current, level, allSubgoals)) {
                selected.add(relief);
            }
        }
        boolean added;
        do {
            added = false;
            for (Subgoal relief : reliefResult.reliefs) {
                if (selected.contains(relief)) continue;
                for (Subgoal parentRelief : selected) {
                    if (reliefSupportsRelief(relief, parentRelief)) {
                        selected.add(relief);
                        added = true;
                        break;
                    }
                }
            }
        } while (added);

        List<Subgoal> candidates = new ArrayList<>(reliefResult.reliefs);
        if (candidates.isEmpty()) {
            return null;
        }
        int maxScopedMoves = Math.max(MAX_TASK_RELIEF_MOVES, Math.min(12, candidates.size()));

        logVerbose("[PP][SCOPED-RELIEF] " + subgoalLabel(blockedSubgoal)
                + " trying " + candidates.size() + " NAMO relief candidate(s)"
                + " (directChain=" + selected.size() + ", moveCap=" + maxScopedMoves + ")");

        for (Subgoal relief : candidates) {
            if (moved >= maxScopedMoves) break;
            if (syntheticReliefBlacklist.contains(relief.goalPos)) continue;
            if (relief.isSyntheticRelief() && !reliefCertificateStillBlocked(relief, current, level)) {
                continue;
            }

            List<Position> blockersBefore = findAccessBlockersForTask(blockedSubgoal, current, level, allSubgoals);
            boolean hadAccessBefore = hasTaskAccess(blockedSubgoal, current, level, allSubgoals);
            int planSizeBefore = fullPlan.size();
            int timeBefore = globalTimeStep;
            SubgoalEval beforeEval = captureSubgoalEval(relief, current, level);

            Position blockerPos = relief.reliefCertificate.blockerStart;
            Character blockerType = current.getBoxes().get(blockerPos);
            if (blockerType == null || blockerType != relief.boxType) {
                continue;
            }
            Set<Position> taskCritical = taskAccessCells(blockedSubgoal, current, level, allSubgoals);
            Set<Position> releaseForbidden = new HashSet<>(taskCritical);
            releaseForbidden.addAll(allGoalCells(level));
            releaseForbidden.add(blockerPos);
            Set<Position> unfreeze = new HashSet<>();
            unfreeze.add(blockerPos);
            int reliefBudget = Math.min(effectiveMaxBspBudget * 2,
                    Math.max(SearchConfig.MIN_BSP_BUDGET * 4,
                            computeDynamicBspBudget(blockerPos, blockedSubgoal.goalPos) * 2));
            List<Action> reliefPath = planSubgoal(relief, current, level, allSubgoals);
            boolean usedFixedTargetFallback = reliefPath != null && !reliefPath.isEmpty();
            if (reliefPath == null || reliefPath.isEmpty()) {
                reliefPath = boxSearchPlanner.planBoxReleaseFromForbidden(
                        relief.agentId, blockerPos, relief.boxType, current, level,
                        releaseForbidden, unfreeze, reliefBudget, taskCritical);
                usedFixedTargetFallback = false;
            }
            if (reliefPath == null || reliefPath.isEmpty()) {
                logVerbose("[PP][SCOPED-RELIEF] failed to plan " + subgoalLabel(relief)
                        + " for " + subgoalLabel(blockedSubgoal)
                        + " cert=" + reliefCertificateLabel(relief));
                continue;
            }

            State beforeRelease = current;
            State trial = appendReliefPath(reliefPath, relief.agentId, current, level, fullPlan, numAgents);
            if (!acceptReachedSubgoal("SCOPED-RELIEF", relief, beforeEval, trial, level,
                    planSizeBefore, fullPlan.size(), allSubgoals)) {
                rollbackPlanTo(fullPlan, planSizeBefore);
                globalTimeStep = timeBefore;
                planMerger.clearAllPlans();
                storedPlanSubgoals.clear();
                continue;
            }

            Position releasedPos = findRelocatedBoxPosition(beforeRelease, trial, relief.boxType, blockerPos);
            List<Position> blockersAfter = findAccessBlockersForTask(blockedSubgoal, trial, level, allSubgoals);
            boolean hasAccessAfter = hasTaskAccess(blockedSubgoal, trial, level, allSubgoals);
            boolean madeTaskProgress = blockersAfter.size() < blockersBefore.size()
                    || (!hadAccessBefore && hasAccessAfter);
            madeAnyTaskProgress |= madeTaskProgress;

            current = trial;
            moved++;
            String acceptedMsg = "[PP][SCOPED-RELIEF] accepted " + subgoalLabel(relief)
                    + " for " + subgoalLabel(blockedSubgoal)
                    + " released=" + blockerPos + "->" + releasedPos
                    + " blockers=" + blockersBefore.size() + "->" + blockersAfter.size()
                    + " parentProgress=" + madeTaskProgress
                    + " mode=" + (usedFixedTargetFallback ? "fixed-target" : "release")
                    + " cert=" + reliefCertificateLabel(relief);
            if (madeTaskProgress) {
                logNormal(acceptedMsg);
            } else {
                logVerbose(acceptedMsg);
            }

            List<Action> parentPath = planSubgoal(blockedSubgoal, current, level, allSubgoals);
            if (parentPath != null && !parentPath.isEmpty()) {
                subgoalManager.invalidateHungarianCache();
                cachedSubgoalOrder = null;
                return current;
            }
        }

        if (moved > 0 && madeAnyTaskProgress) {
            subgoalManager.invalidateHungarianCache();
            cachedSubgoalOrder = null;
            return current;
        }

        if (moved > 0) {
            rollbackPlanTo(fullPlan, initialPlanSize);
            globalTimeStep = initialTime;
            planMerger.clearAllPlans();
            storedPlanSubgoals.clear();
            logVerbose("[PP][SCOPED-RELIEF] incomplete relief for " + subgoalLabel(blockedSubgoal)
                    + " -- rollback partial relief");
        }
        return null;
    }

    private boolean reliefTargetsSubgoal(Subgoal relief, Subgoal blockedSubgoal, State state,
                                         Level level, List<Subgoal> allSubgoals) {
        if (!relief.isSyntheticRelief()) return false;
        ReliefCertificate cert = relief.reliefCertificate;

        if (cert.blockedAgentId == blockedSubgoal.agentId) {
            return true;
        }
        if (blockedSubgoal.goalPos != null && blockedSubgoal.goalPos.equals(cert.secondary)) {
            return true;
        }
        if (!blockedSubgoal.isAgentGoal && blockedSubgoal.goalPos != null
                && blockedSubgoal.goalPos.equals(cert.primary)) {
            return true;
        }
        if (!blockedSubgoal.isAgentGoal && cert.primary != null) {
            for (Position boxPos : sameTypeBoxesForSubgoal(blockedSubgoal, state, level, allSubgoals)) {
                if (boxPos.equals(cert.primary)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean reliefSupportsRelief(Subgoal candidate, Subgoal parentRelief) {
        if (!candidate.isSyntheticRelief() || !parentRelief.isSyntheticRelief()) {
            return false;
        }
        ReliefCertificate candidateCert = candidate.reliefCertificate;
        ReliefCertificate parentCert = parentRelief.reliefCertificate;
        if (candidateCert.blockedAgentId != parentRelief.agentId) {
            return false;
        }
        return parentCert.blockerStart != null
                && (parentCert.blockerStart.equals(candidateCert.primary)
                || parentCert.blockerStart.equals(candidateCert.secondary));
    }

    private State appendReliefPath(List<Action> reliefPath, int helper, State current,
                                   Level level, List<Action[]> fullPlan, int numAgents) {
        State trial = current;
        for (Action action : reliefPath) {
            Action[] jointAction = planMerger.createJointActionWithMerging(
                    helper, action, trial, level, numAgents, false, completedBoxGoals);
            jointAction = conflictResolver.resolveConflicts(jointAction, trial, level, helper);
            fullPlan.add(jointAction);
            trial = applyJointAction(jointAction, trial, level, numAgents);
            globalTimeStep++;
            planMerger.updatePlanIndexes(jointAction, numAgents, helper);
        }
        return trial;
    }

    private Position findRelocatedBoxPosition(State before, State after, char boxType, Position oldPos) {
        Character stillThere = after.getBoxes().get(oldPos);
        if (stillThere != null && stillThere == boxType) return oldPos;

        Set<Position> beforePositions = new HashSet<>();
        for (Map.Entry<Position, Character> entry : before.getBoxes().entrySet()) {
            if (entry.getValue() == boxType) beforePositions.add(entry.getKey());
        }
        for (Map.Entry<Position, Character> entry : after.getBoxes().entrySet()) {
            if (entry.getValue() != boxType) continue;
            if (!beforePositions.contains(entry.getKey())) return entry.getKey();
        }
        return null;
    }

    private Set<Position> allGoalCells(Level level) {
        Set<Position> cells = new HashSet<>();
        for (List<Position> goals : level.getBoxGoalsByType().values()) cells.addAll(goals);
        cells.addAll(level.getAgentGoalPositionMap().values());
        return cells;
    }

    private List<Position> findAccessBlockersForTask(Subgoal subgoal, State state, Level level,
                                                     List<Subgoal> allSubgoals) {
        Color agentColor = level.getAgentColor(subgoal.agentId);
        Set<Position> reachable = strictAgentReachable(state.getAgentPosition(subgoal.agentId), state, level);
        if (subgoal.isAgentGoal) {
            if (reachable.contains(subgoal.goalPos)) {
                return Collections.emptyList();
            }
            return findAgentGoalBlockersForTask(subgoal, state, level, agentColor);
        }

        List<Position> candidateBoxes = sameTypeBoxesForSubgoal(subgoal, state, level, allSubgoals);
        Set<Position> blockers = new LinkedHashSet<>();

        for (Position boxPos : candidateBoxes) {
            if (hasReachableNeighbor(boxPos, reachable, level)) continue;

            for (Direction dir : Direction.values()) {
                Position adj = boxPos.move(dir);
                if (level.isWall(adj)) continue;
                if (reachable.contains(adj)) continue;
                Character adjBox = state.getBoxes().get(adj);
                if (adjBox != null) {
                    Color adjColor = level.getBoxColor(adjBox);
                    if (adjColor != null && !immovableBoxes.contains(adj)
                            && (!adjColor.equals(agentColor) || adjBox != subgoal.boxType)) {
                        blockers.add(adj);
                    }
                } else {
                    blockers.addAll(findAgentPathBlockersToAccessCell(
                            state.getAgentPosition(subgoal.agentId), adj, state, level,
                            agentColor, boxPos));
                    List<Integer> agentBlockers = findAgentPathAgentBlockersToAccessCell(
                            state.getAgentPosition(subgoal.agentId), adj, state, level, subgoal.agentId);
                    Collections.reverse(agentBlockers);
                    for (int agentBlocker : agentBlockers) {
                        Position agentPos = state.getAgentPosition(agentBlocker);
                        if (agentPos != null) blockers.add(agentPos);
                    }
                }
            }
        }
        return new ArrayList<>(blockers);
    }

    private List<Position> findAgentGoalBlockersForTask(Subgoal subgoal, State state, Level level,
                                                        Color agentColor) {
        LinkedHashSet<Position> blockers = new LinkedHashSet<>();
        List<Position> path = findAgentGoalStaticPathCells(
                state.getAgentPosition(subgoal.agentId), subgoal.goalPos, state, level);
        for (Position cell : path) {
            if (cell.equals(state.getAgentPosition(subgoal.agentId))) continue;
            Character box = state.getBoxes().get(cell);
            if (box != null && !immovableBoxes.contains(cell)) {
                blockers.add(cell);
            }
            int agentAt = state.getAgentAt(cell);
            if (agentAt >= 0 && agentAt != subgoal.agentId) {
                blockers.add(cell);
            }
        }
        return new ArrayList<>(blockers);
    }

    private boolean hasTaskAccess(Subgoal subgoal, State state, Level level,
                                  List<Subgoal> allSubgoals) {
        Set<Position> reachable = strictAgentReachable(state.getAgentPosition(subgoal.agentId), state, level);
        if (subgoal.isAgentGoal) {
            return reachable.contains(subgoal.goalPos);
        }
        for (Position boxPos : sameTypeBoxesForSubgoal(subgoal, state, level, allSubgoals)) {
            if (hasReachableNeighbor(boxPos, reachable, level)) {
                return true;
            }
        }
        return false;
    }

    private String taskReliefNogoodKey(Subgoal task, Position blockerPos, Position pTemp) {
        return taskSupportKey(task)
                + "|blocker=" + blockerPos + "|park=" + pTemp;
    }

    private String taskSupportKey(Subgoal task) {
        return task.isAgentGoal
                ? "agent" + task.agentId + "@" + task.goalPos
                : task.agentId + ":" + task.boxType + "@" + task.goalPos;
    }

    /**
     * Finds boxes on a shortest static route from the task agent to an operation
     * cell. Normal reachability treats boxes as walls; this pass treats movable
     * boxes as passable only to identify which ones explain the failed access.
     */
    private List<Position> findAgentPathBlockersToAccessCell(Position start, Position target,
            State state, Level level, Color taskAgentColor, Position targetBoxPos) {
        if (start == null || target == null || level.isWall(target)) return Collections.emptyList();

        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Position> parent = new HashMap<>();
        queue.add(start);
        visited.add(start);

        boolean found = false;
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            if (current.equals(target)) {
                found = true;
                break;
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || immovableBoxes.contains(next)) continue;
                visited.add(next);
                parent.put(next, current);
                queue.add(next);
            }
        }

        if (!found) return Collections.emptyList();

        List<Position> path = new ArrayList<>();
        Position pathCell = target;
        while (pathCell != null) {
            path.add(pathCell);
            if (pathCell.equals(start)) break;
            pathCell = parent.get(pathCell);
        }
        Collections.reverse(path);

        LinkedHashSet<Position> blockers = new LinkedHashSet<>();
        for (Position current : path) {
            if (current.equals(start) || current.equals(targetBoxPos)) continue;
            Character box = state.getBoxes().get(current);
            if (box != null) {
                Color boxColor = level.getBoxColor(box);
                if (boxColor != null) {
                    blockers.add(current);
                }
            }
        }
        return new ArrayList<>(blockers);
    }

    private Set<Position> taskAccessCells(Subgoal subgoal, State state, Level level,
                                          List<Subgoal> allSubgoals) {
        Set<Position> cells = new HashSet<>();
        cells.add(subgoal.goalPos);
        if (subgoal.isAgentGoal) {
            cells.addAll(findAgentGoalStaticPathCells(
                    state.getAgentPosition(subgoal.agentId), subgoal.goalPos, state, level));
            for (Direction dir : Direction.values()) {
                Position adj = subgoal.goalPos.move(dir);
                if (!level.isWall(adj)) cells.add(adj);
            }
            return cells;
        }
        for (Position boxPos : sameTypeBoxesForSubgoal(subgoal, state, level, allSubgoals)) {
            cells.add(boxPos);
            for (Direction dir : Direction.values()) {
                Position adj = boxPos.move(dir);
                if (!level.isWall(adj)) {
                    cells.add(adj);
                    cells.addAll(findStaticPathCellsToAccessCell(
                            state.getAgentPosition(subgoal.agentId), adj, state, level));
                }
            }
        }
        return cells;
    }

    private List<Position> findAgentGoalStaticPathCells(Position start, Position target,
            State state, Level level) {
        List<Position> withoutBoxes = findStaticPathCellsAvoidingBoxes(start, target, state, level);
        if (!withoutBoxes.isEmpty() || (start != null && start.equals(target))) {
            return withoutBoxes;
        }
        return findStaticPathCellsToAccessCell(start, target, state, level);
    }

    private List<Position> findStaticPathCellsAvoidingBoxes(Position start, Position target,
            State state, Level level) {
        if (start == null || target == null || level.isWall(target)) return Collections.emptyList();

        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Position> parent = new HashMap<>();
        queue.add(start);
        visited.add(start);

        boolean found = false;
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            if (current.equals(target)) {
                found = true;
                break;
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || immovableBoxes.contains(next) || state.hasBoxAt(next)) continue;
                visited.add(next);
                parent.put(next, current);
                queue.add(next);
            }
        }

        if (!found) return Collections.emptyList();

        List<Position> path = new ArrayList<>();
        Position current = target;
        while (current != null) {
            path.add(current);
            if (current.equals(start)) break;
            current = parent.get(current);
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * Returns one static access corridor from start to target, treating movable
     * boxes as passable resources. Support subgoals should avoid parking boxes on
     * this corridor; otherwise they can reduce the immediate blocker count while
     * recreating the same access failure one cell later.
     */
    private List<Position> findStaticPathCellsToAccessCell(Position start, Position target,
            State state, Level level) {
        if (start == null || target == null || level.isWall(target)) return Collections.emptyList();

        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Position> parent = new HashMap<>();
        queue.add(start);
        visited.add(start);

        boolean found = false;
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            if (current.equals(target)) {
                found = true;
                break;
            }
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || immovableBoxes.contains(next)) continue;
                visited.add(next);
                parent.put(next, current);
                queue.add(next);
            }
        }

        if (!found) return Collections.emptyList();

        List<Position> path = new ArrayList<>();
        Position current = target;
        while (current != null) {
            path.add(current);
            if (current.equals(start)) break;
            current = parent.get(current);
        }
        Collections.reverse(path);
        return path;
    }

    private List<Position> sameTypeBoxesForSubgoal(Subgoal subgoal, State state, Level level,
                                                   List<Subgoal> allSubgoals) {
        Position preferred = subgoalManager.findBestBoxForGoal(
                subgoal, state, level, allSubgoals, completedBoxGoals);
        List<Position> boxes = new ArrayList<>();
        if (preferred != null) boxes.add(preferred);
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == subgoal.boxType
                    && !entry.getKey().equals(preferred)
                    && level.getBoxGoal(entry.getKey()) != subgoal.boxType) {
                boxes.add(entry.getKey());
            }
        }
        return boxes;
    }

    private Set<Position> strictAgentReachable(Position start, State state, Level level) {
        Set<Position> visited = new HashSet<>();
        if (start == null || level.isWall(start) || state.hasBoxAt(start)) return visited;
        Queue<Position> queue = new LinkedList<>();
        visited.add(start);
        queue.add(start);
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || state.hasBoxAt(next) || state.hasAgentAt(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    private Set<Position> agentGoalReachableWithFrozen(int agentId, State state,
            Level level, Set<Position> frozen) {
        Position start = state.getAgentPosition(agentId);
        Set<Position> visited = new HashSet<>();
        if (start == null || level.isWall(start) || state.hasBoxAt(start)
                || frozen.contains(start)) {
            return visited;
        }

        Queue<Position> queue = new LinkedList<>();
        visited.add(start);
        queue.add(start);
        // Seal-risk checks hard topology only: other agents can be coordinated away later.
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || frozen.contains(next) || state.hasBoxAt(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    private boolean hasReachableNeighbor(Position pos, Set<Position> reachable, Level level) {
        for (Direction dir : Direction.values()) {
            Position adj = pos.move(dir);
            if (!level.isWall(adj) && reachable.contains(adj)) return true;
        }
        return false;
    }

    private List<Position> operationCells(Position boxPos, Level level) {
        List<Position> cells = new ArrayList<>();
        if (boxPos == null) return cells;
        for (Direction dir : Direction.values()) {
            Position adj = boxPos.move(dir);
            if (!level.isWall(adj)) {
                cells.add(adj);
            }
        }
        return cells;
    }

    private int findHelperAgentForBox(char boxType, State state, Level level, Position boxPos) {
        Color color = level.getBoxColor(boxType);
        if (color == null) return -1;
        int bestAgent = -1;
        int bestDist = Integer.MAX_VALUE;
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            if (!color.equals(level.getAgentColor(agentId))) continue;
            Position agentPos = state.getAgentPosition(agentId);
            if (agentPos == null) continue;
            int d = agentPos.manhattanDistance(boxPos);
            if (d < bestDist) {
                bestDist = d;
                bestAgent = agentId;
            }
        }
        return bestAgent;
    }

    private List<Position> findTaskReliefParkingCandidates(Position blockerPos, State state, Level level,
                                                           Set<Position> taskCritical,
                                                           Set<Position> usedTemps) {
        Set<Position> goalCells = new HashSet<>();
        for (List<Position> goals : level.getBoxGoalsByType().values()) goalCells.addAll(goals);
        goalCells.addAll(level.getAgentGoalPositionMap().values());

        List<Position> safe = new ArrayList<>();
        List<Position> fallback = new ArrayList<>();
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(blockerPos);
        visited.add(blockerPos);

        int expansions = 0;
        while (!queue.isEmpty() && expansions++ < 160) {
            Position p = queue.poll();
            if (!p.equals(blockerPos)
                    && !level.isWall(p)
                    && !state.hasBoxAt(p)
                    && !immovableBoxes.contains(p)
                    && !goalCells.contains(p)
                    && !taskCritical.contains(p)
                    && !usedTemps.contains(p)) {
                if (countFreeNeighborsForTaskRelief(p, state, level) <= 1) {
                    safe.add(p);
                } else {
                    fallback.add(p);
                }
            }

            for (Direction dir : Direction.values()) {
                Position next = p.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || immovableBoxes.contains(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }

        Comparator<Position> byDistance = Comparator
                .comparingInt((Position p) -> p.manhattanDistance(blockerPos))
                .thenComparingInt(p -> p.row)
                .thenComparingInt(p -> p.col);
        safe.sort(byDistance);
        fallback.sort(byDistance);
        safe.addAll(fallback);
        return safe;
    }

    private int countFreeNeighborsForTaskRelief(Position p, State state, Level level) {
        int count = 0;
        for (Direction dir : Direction.values()) {
            Position adj = p.move(dir);
            if (!level.isWall(adj) && !state.hasBoxAt(adj) && !immovableBoxes.contains(adj)) {
                count++;
            }
        }
        return count;
    }

    /**
     * Path Clearing: when BSP fails because OTHER boxes physically block the target
     * box's path to its goal, this method identifies those blockers and uses the
     * appropriate same-color agent to push them out of the way.
     * 
     * Algorithm:
     * 1. Compute the static BFS path from box to goal (ignoring all dynamic obstacles)
     * 2. Find OTHER boxes that sit on this path (or on required agent push positions)
     * 3. For each blocker: find same-color agent, find safe clearing position via BFS,
     *    use BSP to plan displacement, execute
     * 4. Return updated state if any clearing was done, null otherwise
     * 
     * This handles the classic multi-agent box-blocking pattern: e.g., in MADS,
     * box C (pink) at (10,3) blocks red agent's path for box A to goal (10,1).
     */
    private State tryPathClearing(Subgoal subgoal, State state, Level level,
            List<Action[]> fullPlan, int numAgents, List<Subgoal> allSubgoals) {
        
        Position boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
        if (boxPos == null) return null;
        
        // Step 1: Find a structural path from box to goal. Completed goals are
        // stable resources, so prefer a route that does not pass through them.
        Set<Position> protectedGoals = new HashSet<>(completedBoxGoals);
        protectedGoals.remove(boxPos);
        protectedGoals.remove(subgoal.goalPos);
        List<Position> idealPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                boxPos, subgoal.goalPos, level, protectedGoals);
        boolean pathUsesProtectedGoals = idealPath == null;
        if (idealPath == null) {
            idealPath = pathAnalyzer.findPathIgnoringDynamicObstacles(boxPos, subgoal.goalPos, level);
        }
        if (idealPath == null || idealPath.isEmpty()) return null;
        
        // Build the "danger zone": all positions on the path + agent push positions
        Set<Position> dangerZone = new HashSet<>(idealPath);
        for (int i = 0; i < idealPath.size() - 1; i++) {
            Position from = idealPath.get(i);
            Position to = idealPath.get(i + 1);
            int dr = to.row - from.row;
            int dc = to.col - from.col;
            // Agent push position: opposite side of push direction
            Position agentPush = new Position(from.row - dr, from.col - dc);
            if (!level.isWall(agentPush)) {
                dangerZone.add(agentPush);
            }
        }
        
        // Step 2: Find boxes that are in the danger zone (excluding the target box itself)
        List<Position> blockerPositions = new ArrayList<>();
        LinkedHashSet<Position> borrowBlockerPositions = new LinkedHashSet<>();
        int skippedProtectedBlockers = 0;
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position bPos = entry.getKey();
            if (bPos.equals(boxPos)) continue;
            if (dangerZone.contains(bPos)) {
                if (completedBoxGoals.contains(bPos)) {
                    if (pathUsesProtectedGoals) {
                        borrowBlockerPositions.add(bPos);
                    } else {
                        skippedProtectedBlockers++;
                    }
                    continue;
                }
                blockerPositions.add(bPos);
            }
        }
        
        if (blockerPositions.isEmpty() && borrowBlockerPositions.isEmpty()) {
            if (skippedProtectedBlockers > 0) {
                logVerbose("[PP] [CLEAR] Static path for " + subgoal.boxType + " -> " + subgoal.goalPos
                        + (pathUsesProtectedGoals ? " requires" : " touched")
                        + " completed goal(s); leaving them protected");
            }
            return null;
        }
        
        logVerbose("[PP] [CLEAR] Found " + blockerPositions.size() + " ordinary blocker(s)"
                + (borrowBlockerPositions.isEmpty() ? "" : " and "
                        + borrowBlockerPositions.size() + " borrowable completed-goal blocker(s)")
                + " for " + subgoal.boxType + " -> " + subgoal.goalPos + ": "
                + blockerPositions + (borrowBlockerPositions.isEmpty() ? "" : " borrow=" + borrowBlockerPositions));
        
        // Step 3: For each blocker, find its agent and a safe clearing position
        State currentState = state;
        boolean anyCleared = false;
        lastClearingDisplacedGoals.clear();
        lastBorrowedGoalBoxPositions.clear();
        int displacementCount = 0;
        
        for (Position blockerPos : blockerPositions) {
            Character blockerType = currentState.getBoxes().get(blockerPos);
            if (blockerType == null) continue; // already moved
            
            Color blockerColor = level.getBoxColor(blockerType);
            if (blockerColor == null) continue;
            
            // Find agent that can push this box
            int clearingAgent = -1;
            for (int a = 0; a < level.getNumAgents(); a++) {
                if (blockerColor.equals(level.getAgentColor(a))) {
                    clearingAgent = a;
                    break;
                }
            }
            if (clearingAgent < 0) continue;
            
            // Find safe clearing position: BFS from blocker to find nearest cell NOT in dangerZone
            Position clearTarget = findClearingPosition(blockerPos, dangerZone, currentState, level);
            if (clearTarget == null) {
                logVerbose("[PP] [CLEAR] No safe clearing position for " + blockerType + " at " + blockerPos);
                continue;
            }
            
            logVerbose("[PP] [CLEAR] Moving " + blockerType + " from " + blockerPos + " to " + clearTarget
                    + " (agent " + clearingAgent + ")");
            
            // Plan displacement
            List<Action> displacePath = boxSearchPlanner.planBoxDisplacement(
                    clearingAgent, blockerPos, clearTarget, blockerType, currentState, level);
            
            if (displacePath != null && !displacePath.isEmpty()) {
                // Check if this displacement would exceed the cap
                // Count goals that would be displaced by moving this blocker
                boolean blockerOnCompletedGoal = completedBoxGoals.contains(blockerPos);
                if (blockerOnCompletedGoal) {
                    displacementCount++;
                    if (displacementCount > MAX_CLEARING_DISPLACEMENTS) {
                        logVerbose("[PP] [CLEAR] Displacement cap reached (" + MAX_CLEARING_DISPLACEMENTS + "), skipping remaining");
                        break;
                    }
                }
                
                // Execute the clearing path
                for (Action action : displacePath) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            clearingAgent, action, currentState, level, numAgents, false, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, clearingAgent);
                    fullPlan.add(jointAction);
                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    globalTimeStep++;
                }
                anyCleared = true;
                
                // Track displaced goals for borrow-and-return
                if (blockerOnCompletedGoal) {
                    lastClearingDisplacedGoals.add(blockerPos);
                    displacedGoals.add(blockerPos);
                    logVerbose("[PP] [CLEAR] Displaced completed goal at " + blockerPos);
                }
                
                logVerbose("[PP] [CLEAR] Cleared " + blockerType + " in " + displacePath.size() + " steps");
            } else {
                logVerbose("[PP] [CLEAR] BSP can't plan displacement for " + blockerType + " at " + blockerPos);
            }
        }
        
        // Phase B: Agent→box approach path clearing
        // Only runs if box→goal clearing didn't make progress AND the agent actually
        // can't reach the box (BFS through real obstacles fails).
        if (!anyCleared) {
            Position agentPos = currentState.getAgentPosition(subgoal.agentId);
            Color agentColor = level.getAgentColor(subgoal.agentId);
            
            // Check if agent can actually reach the box
            List<Action> agentPath = pathAnalyzer.planAgentPath(
                    subgoal.agentId, boxPos, currentState, level, numAgents);
            
            if (agentPath == null) {
                // Agent can't reach box — find different-color boxes blocking the path
                List<Position> agentToBoxPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        agentPos, boxPos, level);
                
                if (agentToBoxPath != null) {
                    for (Position pathCell : agentToBoxPath) {
                        Character boxAtCell = currentState.getBoxes().get(pathCell);
                        if (boxAtCell != null && level.getBoxColor(boxAtCell) != agentColor) {
                            if (completedBoxGoals.contains(pathCell)) {
                                borrowBlockerPositions.add(pathCell);
                                continue;
                            }
                            // Find same-color agent for this blocker
                            Color bColor = level.getBoxColor(boxAtCell);
                            int clearAgent = -1;
                            for (int a = 0; a < level.getNumAgents(); a++) {
                                if (bColor.equals(level.getAgentColor(a))) {
                                    clearAgent = a;
                                    break;
                                }
                            }
                            if (clearAgent < 0) continue;
                            
                            // Clear it away from both dangerZone and agent→box path
                            Set<Position> excludeZone = new HashSet<>(dangerZone);
                            excludeZone.addAll(agentToBoxPath);
                            Position clearTarget = findClearingPosition(pathCell, excludeZone, currentState, level);
                            if (clearTarget == null) {
                                clearTarget = findClearingPosition(pathCell, dangerZone, currentState, level);
                            }
                            if (clearTarget == null) continue;
                            
                            logVerbose("[PP] [CLEAR-A2B] Moving " + boxAtCell + " from " + pathCell 
                                    + " to " + clearTarget + " (agent " + clearAgent + ")");
                            
                            List<Action> displacePath = boxSearchPlanner.planBoxDisplacement(
                                    clearAgent, pathCell, clearTarget, boxAtCell, currentState, level);
                            
                            if (displacePath != null && !displacePath.isEmpty()) {
                                // Check displacement cap
                                boolean onGoal = completedBoxGoals.contains(pathCell);
                                if (onGoal) {
                                    displacementCount++;
                                    if (displacementCount > MAX_CLEARING_DISPLACEMENTS) {
                                        logVerbose("[PP] [CLEAR-A2B] Displacement cap reached");
                                        break;
                                    }
                                }
                                
                                for (Action action : displacePath) {
                                    Action[] jointAction = planMerger.createJointActionWithMerging(
                                            clearAgent, action, currentState, level, numAgents, false, completedBoxGoals);
                                    jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, clearAgent);
                                    fullPlan.add(jointAction);
                                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                                    globalTimeStep++;
                                }
                                anyCleared = true;
                                
                                if (onGoal) {
                                    lastClearingDisplacedGoals.add(pathCell);
                                    displacedGoals.add(pathCell);
                                    logVerbose("[PP] [CLEAR-A2B] Displaced completed goal at " + pathCell);
                                }
                                
                                logVerbose("[PP] [CLEAR-A2B] Cleared " + boxAtCell + " in " + displacePath.size() + " steps");
                            }
                        }
                    }
                }
            }
        }

        if (!anyCleared && !borrowBlockerPositions.isEmpty()) {
            for (Position borrowPos : borrowBlockerPositions) {
                State borrowed = tryBorrowCompletedGoalForClearing(
                        borrowPos, dangerZone, currentState, level, fullPlan, numAgents);
                if (borrowed != null) {
                    currentState = borrowed;
                    anyCleared = true;
                    break;
                }
            }
        }
        
        return anyCleared ? currentState : null;
    }

    private State tryBorrowCompletedGoalForClearing(Position borrowPos, Set<Position> dangerZone,
            State state, Level level, List<Action[]> fullPlan, int numAgents) {
        if (!completedBoxGoals.contains(borrowPos)) return null;

        char goalType = level.getBoxGoal(borrowPos);
        Character blockerType = state.getBoxes().get(borrowPos);
        if (goalType == '\0' || blockerType == null || blockerType != goalType) return null;

        int helper = findHelperAgentForBox(blockerType, state, level, borrowPos);
        if (helper < 0) return null;

        Set<Position> forbidden = new HashSet<>(dangerZone);
        forbidden.addAll(allGoalCells(level));
        forbidden.add(borrowPos);

        Set<Position> unfreeze = new HashSet<>();
        unfreeze.add(borrowPos);

        Set<Position> protectedPositions = new HashSet<>(completedBoxGoals);
        protectedPositions.remove(borrowPos);
        protectedPositions.addAll(dangerZone);

        int budget = Math.min(effectiveMaxBspBudget * 2,
                Math.max(SearchConfig.MIN_BSP_BUDGET * 4,
                        computeDynamicBspBudget(borrowPos, borrowPos) * 2));

        int planSizeBefore = fullPlan.size();
        int timeBefore = globalTimeStep;
        List<Action> releasePath = boxSearchPlanner.planBoxReleaseFromForbidden(
                helper, borrowPos, blockerType, state, level,
                forbidden, unfreeze, budget, protectedPositions);
        if (releasePath == null || releasePath.isEmpty()) {
            return null;
        }

        State trial = appendReliefPath(releasePath, helper, state, level, fullPlan, numAgents);
        Position releasedPos = findRelocatedBoxPosition(state, trial, blockerType, borrowPos);
        boolean moved = releasedPos != null
                && !releasedPos.equals(borrowPos)
                && !trial.getBoxes().containsKey(borrowPos)
                && !forbidden.contains(releasedPos)
                && level.getBoxGoal(releasedPos) == '\0'
                && level.getAgentGoal(releasedPos.row, releasedPos.col) < 0;
        if (!moved) {
            rollbackPlanTo(fullPlan, planSizeBefore);
            globalTimeStep = timeBefore;
            planMerger.clearAllPlans();
            storedPlanSubgoals.clear();
            return null;
        }

        lastClearingDisplacedGoals.add(borrowPos);
        lastBorrowedGoalBoxPositions.put(borrowPos, releasedPos);
        displacedGoals.add(borrowPos);
        logNormal("[PP][BORROW] borrowed completed goal " + blockerType
                + " " + borrowPos + " -> " + releasedPos
                + " (return required)");
        return trial;
    }
    
    /**
     * BFS to find the nearest safe clearing position for a blocker box.
     * The position must be:
     * - Not in the danger zone (path + agent push positions)
     * - Not a wall
     * - Not occupied by another box
     * - Not an agent position
     * - Preferably NOT on a completed goal (borrow-and-return: minimize displacements)
     */
    private Position findClearingPosition(Position blockerPos, Set<Position> dangerZone,
            State state, Level level) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        visited.add(blockerPos);
        queue.add(blockerPos);
        
        Position firstGoalFallback = null; // first valid pos that's on a completed goal
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next) || level.isWall(next)) continue;
                visited.add(next);
                
                // Valid clearing position: not in danger zone, not occupied
                if (!dangerZone.contains(next) 
                        && !state.getBoxes().containsKey(next)
                        && !isAgentAt(next, state, level)) {
                    // Prefer positions NOT on completed goals
                    if (!completedBoxGoals.contains(next)) {
                        return next;
                    }
                    // Remember first goal-position as fallback
                    if (firstGoalFallback == null) {
                        firstGoalFallback = next;
                    }
                }
                queue.add(next);
            }
        }
        return firstGoalFallback; // may be null if nothing found at all
    }
    
    /** Check if any agent is at the given position. */
    private boolean isAgentAt(Position pos, State state, Level level) {
        for (int a = 0; a < state.getNumAgents(); a++) {
            if (state.getAgentPosition(a).equals(pos)) return true;
        }
        return false;
    }
    
    /**
     * Borrow-and-return: after path clearing + BSP succeeds, try to push
     * displaced boxes back to their original goal positions.
     * Best-effort: if any return fails, the goal will be re-planned normally
     * in a later iteration (revalidateCompletedGoals removed it from completedBoxGoals).
     */
    private State returnDisplacedGoals(State state, Level level, List<Action[]> fullPlan, int numAgents) {
        State currentState = state;
        int returned = 0;
        
        for (Position goalPos : lastClearingDisplacedGoals) {
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') continue;
            
            // Check if the goal is actually still unsatisfied
            Character boxAtGoal = currentState.getBoxes().get(goalPos);
            if (boxAtGoal != null && boxAtGoal == goalType) {
                displacedGoals.remove(goalPos);
                lastBorrowedGoalBoxPositions.remove(goalPos);
                continue; // already back
            }
            
            // Find the nearest box of this type that can be pushed back
            Color goalColor = level.getBoxColor(goalType);
            if (goalColor == null) continue;
            
            int returnAgent = -1;
            for (int a = 0; a < level.getNumAgents(); a++) {
                if (goalColor.equals(level.getAgentColor(a))) {
                    returnAgent = a;
                    break;
                }
            }
            if (returnAgent < 0) continue;
            
            // Find the displaced box — nearest box of the right type not on its goal
            Position bestBoxPos = lastBorrowedGoalBoxPositions.get(goalPos);
            if (bestBoxPos != null && currentState.getBoxAt(bestBoxPos) != goalType) {
                bestBoxPos = null;
            }
            int bestDist = Integer.MAX_VALUE;
            if (bestBoxPos == null) {
                for (Map.Entry<Position, Character> entry : currentState.getBoxes().entrySet()) {
                    if (entry.getValue() != goalType) continue;
                    Position bPos = entry.getKey();
                    // Skip boxes that are already on a satisfied goal
                    char bGoal = level.getBoxGoal(bPos.row, bPos.col);
                    if (bGoal == entry.getValue()) continue;
                    int dist = goalPos.manhattanDistance(bPos);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestBoxPos = bPos;
                    }
                }
            }
            if (bestBoxPos == null) continue;
            
            // Plan BSP to push box back to goal
            Set<Position> frozenForReturn = new HashSet<>(completedBoxGoals);
            frozenForReturn.removeAll(displacedGoals);
            frozenForReturn.remove(goalPos); // target must not be frozen
            List<Action> returnPath = boxSearchPlanner.searchForSubgoal(
                    returnAgent, bestBoxPos, goalPos, goalType, currentState, level, frozenForReturn);
            
            if (returnPath != null && !returnPath.isEmpty()) {
                for (Action action : returnPath) {
                    Action[] jointAction = planMerger.createJointActionWithMerging(
                            returnAgent, action, currentState, level, numAgents, false, completedBoxGoals);
                    jointAction = conflictResolver.resolveConflicts(jointAction, currentState, level, returnAgent);
                    fullPlan.add(jointAction);
                    currentState = applyJointAction(jointAction, currentState, level, numAgents);
                    globalTimeStep++;
                }
                
                // Verify the box actually reached the goal
                Character atGoal = currentState.getBoxes().get(goalPos);
                if (atGoal != null && atGoal == goalType) {
                    completedBoxGoals.add(goalPos);
                    displacedGoals.remove(goalPos);
                    lastBorrowedGoalBoxPositions.remove(goalPos);
                    returned++;
                    logVerbose("[PP] [RETURN] Returned " + goalType + " to " + goalPos 
                            + " (" + returnPath.size() + " steps)");
                }
            }
        }
        
        if (returned > 0) {
            logNormal("[PP] [RETURN] Returned " + returned + "/" 
                    + lastClearingDisplacedGoals.size() + " displaced goals");
        }
        lastClearingDisplacedGoals.clear();
        return currentState;
    }

    private boolean borrowedGoalsRestored(List<Position> borrowedGoals, State state, Level level) {
        for (Position goalPos : borrowedGoals) {
            char goalType = level.getBoxGoal(goalPos);
            if (goalType == '\0') return false;
            Character atGoal = state.getBoxes().get(goalPos);
            if (atGoal == null || atGoal != goalType) return false;
        }
        return true;
    }

    /**
     * Agent-trap detection: checks if committing to this subgoal would trap the agent.
     * 
     * IMPORTANT: In a pull-supporting domain, the agent can always pull same-color
     * boxes out of the way to create paths. This means boxes of the agent's color
     * are NOT permanent obstacles. Only treat walls and different-color/immovable
     * boxes as obstacles. Also, the agent can push/pull boxes it's adjacent to,
     * so we include cells reachable after moving a same-color box.
     * 
     * Pre-execution comparison: If the agent was ALREADY unable to reach remaining
     * tasks before executing this subgoal (e.g., blocked by cross-color boxes),
     * then filling the goal doesn't make things worse — don't flag as trapped.
     * This prevents false positives in dense levels like ZOOM where A-boxes
     * permanently separate map regions.
     */
    private boolean wouldTrapAgent(Subgoal completedSubgoal, State stateAfterExecution, 
            Level level, List<Subgoal> allSubgoals, State stateBeforeExecution) {
        Color agentColor = level.getAgentColor(completedSubgoal.agentId);
        
        List<Subgoal> remainingSameColor = new ArrayList<>();
        for (Subgoal sg : allSubgoals) {
            if (sg == completedSubgoal) continue;
            if (sg.isAgentGoal) continue;
            if (completedBoxGoals.contains(sg.goalPos)) continue;
            Color sgColor = level.getBoxColor(sg.boxType);
            if (agentColor != null && agentColor.equals(sgColor)) {
                remainingSameColor.add(sg);
            }
        }
        
        if (remainingSameColor.isEmpty()) return false;
        
        if (teamCanReachAnyRemainingTask(agentColor, remainingSameColor, stateAfterExecution, level)) {
            return false;
        }
        
        boolean couldReachBefore = teamCanReachAnyRemainingTask(agentColor, remainingSameColor,
                stateBeforeExecution, level);
        if (!couldReachBefore) {
            logVerbose("[PP] [TRAP-BYPASS] Color team " + agentColor
                    + " already couldn't reach remaining " + remainingSameColor.size()
                    + " tasks before " + completedSubgoal.boxType
                    + " -> " + completedSubgoal.goalPos + " - allowing");
            return false;
        }
        
        return true;
    }

    private boolean teamCanReachAnyRemainingTask(Color teamColor, List<Subgoal> remainingSameColor,
                                                 State state, Level level) {
        if (teamColor == null || remainingSameColor.isEmpty()) return false;

        for (int agent = 0; agent < state.getNumAgents(); agent++) {
            if (level.getAgentColor(agent) != teamColor) continue;
            Position agentPos = state.getAgentPosition(agent);
            Set<Position> reachable = agentReachabilityBFS(agentPos, state, level, teamColor);
            for (Subgoal sg : remainingSameColor) {
                for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
                    if (entry.getValue() != sg.boxType) continue;
                    Position boxPos = entry.getKey();
                    for (Direction dir : Direction.values()) {
                        if (reachable.contains(boxPos.move(dir))) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
    
    /**
     * BFS from a position to find all cells reachable by the agent.
     * Pull-aware: same-color boxes can be pulled out of the way, so they are
     * NOT treated as permanent obstacles. Only walls, immovable boxes, and
     * different-color boxes block the agent.
     */
    private Set<Position> agentReachabilityBFS(Position start, State state, Level level, Color agentColor) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        
        visited.add(start);
        queue.add(start);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                
                // Check if there's a box at this position
                Character boxAtNext = state.getBoxes().get(next);
                if (boxAtNext != null) {
                    // Same-color box: agent can pull it out of the way → passable
                    Color boxColor = level.getBoxColor(boxAtNext);
                    if (boxColor != null && boxColor.equals(agentColor)) {
                        // Box is same color — agent can interact (push/pull) to clear path
                        visited.add(next);
                        queue.add(next);
                        continue;
                    }
                    // Different-color or immovable box → obstacle
                    continue;
                }
                
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }
    
    // ====================== Connected Component Parallel Execution ======================
    
    /**
     * Computes connected components for agents. Agents in different components
     * are separated by walls/immovable boxes and cannot interact — their plans
     * can execute in parallel, reducing total plan steps from O(sum) to O(max).
     */
    private Map<Integer, Integer> computeAgentComponents(State state, Level level) {
        int numAgents = state.getNumAgents();
        if (numAgents <= 1) {
            Map<Integer, Integer> single = new HashMap<>();
            single.put(0, 0);
            return single;
        }
        
        // BFS reachability per agent (movable boxes are passable — agent can push/pull them)
        Map<Integer, Set<Position>> reachable = new HashMap<>();
        for (int a = 0; a < numAgents; a++) {
            Set<Position> area = new HashSet<>();
            Queue<Position> queue = new LinkedList<>();
            Position start = state.getAgentPosition(a);
            queue.add(start);
            area.add(start);
            while (!queue.isEmpty()) {
                Position current = queue.poll();
                for (Direction dir : Direction.values()) {
                    Position next = current.move(dir);
                    if (!level.isWall(next) && !immovableBoxes.contains(next) && !area.contains(next)) {
                        area.add(next);
                        queue.add(next);
                    }
                }
            }
            reachable.put(a, area);
        }
        
        // Union-Find: agents whose reachable sets overlap are in the same component
        int[] parent = new int[numAgents];
        for (int i = 0; i < numAgents; i++) parent[i] = i;
        
        for (int a = 0; a < numAgents; a++) {
            for (int b = a + 1; b < numAgents; b++) {
                // Fast check: is agent b's position in agent a's reachable set?
                if (reachable.get(a).contains(state.getAgentPosition(b))) {
                    unionFind(parent, a, b);
                }
            }
        }
        
        Map<Integer, Integer> result = new HashMap<>();
        for (int a = 0; a < numAgents; a++) {
            result.put(a, findRoot(parent, a));
        }
        
        // Log independent groups
        Map<Integer, List<Integer>> groups = new HashMap<>();
        for (var entry : result.entrySet()) {
            groups.computeIfAbsent(entry.getValue(), k -> new ArrayList<>()).add(entry.getKey());
        }
        if (groups.size() > 1) {
            logNormal("[PP] Detected " + groups.size() + " independent agent groups: " + groups.values());
        }
        
        return result;
    }
    
    private int findRoot(int[] parent, int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]]; // path compression
            x = parent[x];
        }
        return x;
    }
    
    private void unionFind(int[] parent, int a, int b) {
        int pa = findRoot(parent, a);
        int pb = findRoot(parent, b);
        parent[pa] = pb;
    }
    
    /**
     * Plans subgoals for agents in independent connected components.
     * When the primary agent's subgoal is being executed, agents in other
     * components can execute their own subgoals in parallel via stored plans.
     * The existing addOtherAgentMoves() in PlanMerger merges them at each step.
     */
    private void planIndependentAgents(Subgoal primarySubgoal, List<Subgoal> allSubgoals,
            State state, Level level) {
        if (agentComponentId.size() <= 1) return;
        
        int primaryComponent = agentComponentId.getOrDefault(primarySubgoal.agentId, -1);
        Set<Integer> plannedComponents = new HashSet<>();
        plannedComponents.add(primaryComponent);
        
        for (Subgoal sg : allSubgoals) {
            if (sg == primarySubgoal) continue;
            int sgComponent = agentComponentId.getOrDefault(sg.agentId, -1);
            if (sgComponent == primaryComponent || plannedComponents.contains(sgComponent)) continue;
            
            // Skip if agent already has a stored plan being executed
            if (planMerger.hasPlanRemaining(sg.agentId)) {
                plannedComponents.add(sgComponent);
                continue;
            }
            
            // Check dependencies
            if (!areDependenciesMet(sg.goalPos, level)) continue;
            
            // Skip goals completed too many times (cycle detection)
            if (!sg.isAgentGoal) {
                int completions = goalCompletionCount.getOrDefault(sg.goalPos, 0);
                if (completions >= MAX_GOAL_COMPLETIONS) continue;
            }
            
            // Plan this subgoal
            List<Action> path = planSubgoal(sg, state, level, allSubgoals);
            if (path != null && !path.isEmpty()) {
                planMerger.storePlan(sg.agentId, path, globalTimeStep);
                storedPlanSubgoals.put(sg.agentId, sg);
                plannedComponents.add(sgComponent);
                logNormal("[PP] [PARALLEL] Stored plan for Agent " + sg.agentId 
                        + " (" + (sg.isAgentGoal ? "AgentGoal" : "Box " + sg.boxType) + " -> " + sg.goalPos
                        + ", " + path.size() + " steps) in component " + sgComponent);
            }
        }
    }
    
    /**
     * Checks if any agents executing via stored plans (parallel execution)
     * have completed their goals. Updates completion tracking accordingly.
     */
    private void checkStoredPlanCompletions(State state, Level level) {
        if (storedPlanSubgoals.isEmpty()) return;
        
        Iterator<Map.Entry<Integer, Subgoal>> it = storedPlanSubgoals.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Subgoal> entry = it.next();
            int agentId = entry.getKey();
            Subgoal sg = entry.getValue();
            
            // Check if plan is exhausted
            if (!planMerger.hasPlanRemaining(agentId)) {
                boolean reached = verifyGoalReached(sg, state, level);
                if (reached) {
                    if (!sg.isAgentGoal) {
                        completedBoxGoals.add(sg.goalPos);
                        int count = goalCompletionCount.getOrDefault(sg.goalPos, 0) + 1;
                        goalCompletionCount.put(sg.goalPos, count);
                        subgoalManager.invalidateHungarianCache();
                        cachedSubgoalOrder = null;
                    }
                    if (sg.isAgentGoal) {
                        completedAgentGoals.add(sg.goalPos);
                    }
                    logVerbose(getName() + ": [OK] " 
                            + (sg.isAgentGoal ? "Agent " + agentId : "Box " + sg.boxType)
                            + " -> " + sg.goalPos + " [PARALLEL]");
                }
                planMerger.invalidatePlan(agentId);
                it.remove();
            }
        }
    }

    /** Extract agent position path from action sequence. */
    private List<Position> extractAgentPath(int agentId, State startState, List<Action> actions, Level level) {
        List<Position> path = new ArrayList<>();
        State state = startState;
        path.add(state.getAgentPosition(agentId));
        
        for (Action action : actions) {
            if (action.type != Action.ActionType.NOOP && state.isApplicable(action, agentId, level)) {
                state = state.apply(action, agentId);
            }
            path.add(state.getAgentPosition(agentId));
        }
        return path;
    }
    
    /**
     * Park agent at a safe position after completing a subgoal.
     * 
     * Pukoban standard yielding: after an agent finishes its task, it must move out of
     * corridors and chokepoints so that subsequent agents can use those passages.
     * Uses PathAnalyzer.findParkingPosition which avoids:
     * - Corridors (positions with only 2 passable neighbors)
     * - Box goal positions
     * - Agent goal positions  
     * - Satisfied goal positions (frozen boxes)
     * - Other agent positions
     */
    private State parkAgentAfterSubgoal(int agentId, State state, Level level, 
            List<Action[]> fullPlan, int numAgents, List<Subgoal> pendingSubgoals) {
        Position agentPos = state.getAgentPosition(agentId);
        
        // Check if agent is already at a safe position:
        // Not on a goal, not in a corridor (2 passable neighbors = corridor)
        // TASK-AWARE FIX: Also check if agent is on the critical path of any pending subgoal
        boolean onBoxGoal = level.getBoxGoal(agentPos) != '\0';
        boolean onAgentGoal = level.getAgentGoal(agentPos.row, agentPos.col) >= 0;
        boolean inCorridor = pathAnalyzer.isInCorridor(agentPos, level);
        boolean onCriticalPath = isOnPendingSubgoalCriticalPath(agentPos, agentId, pendingSubgoals, state, level);
        
        if (!onBoxGoal && !onAgentGoal && !inCorridor && !onCriticalPath) {
            return state; // Already in a safe parking spot
        }
        
        logVerbose("[PP] Yielding agent " + agentId + " from " + agentPos
                + " (onBoxGoal=" + onBoxGoal + ", onAgentGoal=" + onAgentGoal 
                + ", inCorridor=" + inCorridor + ", onCriticalPath=" + onCriticalPath + ")");
        
        // Compute positions to avoid: all frozen/completed goal positions
        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(state, level);
        satisfiedGoals.addAll(completedBoxGoals);
        
        // Critical positions: other agents' current positions + pending subgoal paths
        Set<Position> criticalPositions = new HashSet<>();
        for (int i = 0; i < numAgents; i++) {
            if (i == agentId) continue;
            criticalPositions.add(state.getAgentPosition(i));
        }
        
        // Also add critical positions from pending subgoals so parking doesn't block them
        for (Subgoal sg : pendingSubgoals) {
            if (sg.isAgentGoal) {
                criticalPositions.add(sg.goalPos);
            } else {
                criticalPositions.add(sg.goalPos);
                // Task-Aware: Use globally valid box allocation for critical path checking
                Position boxPos = subgoalManager.findBestBoxForGoal(sg, state, level, pendingSubgoals, completedBoxGoals);
                if (boxPos != null) {
                    List<Position> boxPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                            boxPos, sg.goalPos, level);
                    if (boxPath != null) {
                        criticalPositions.addAll(boxPath);
                        // Add adjacent cells (agent push positions)
                        for (Position p : boxPath) {
                            for (Direction dir : Direction.values()) {
                                Position adj = p.move(dir);
                                if (!level.isWall(adj)) criticalPositions.add(adj);
                            }
                        }
                    }
                }
            }
        }
        
        // Find parking position using PathAnalyzer's existing logic
        Position parkingPos = pathAnalyzer.findParkingPosition(
                agentId, state, level, numAgents, criticalPositions, satisfiedGoals);
        
        if (parkingPos == null || parkingPos.equals(agentPos)) {
            return state; // No better parking position found
        }
        
        // Plan path to parking position (with push/pull fallback through same-color boxes)
        List<Action> parkPath = pathAnalyzer.planAgentPathWithFallback(
                agentId, parkingPos, state, level, numAgents, boxSearchPlanner);
        if (parkPath == null || parkPath.isEmpty()) {
            return state;
        }
        
        // Limit parking path length to avoid wasting too many steps
        int maxParkSteps = 10;
        if (parkPath.size() > maxParkSteps) {
            parkPath = parkPath.subList(0, maxParkSteps);
        }
        
        // Execute the parking path
        State tempState = state;
        for (Action moveAction : parkPath) {
            if (!tempState.isApplicable(moveAction, agentId, level)) break;
            Action[] jointAction = new Action[numAgents];
            for (int i = 0; i < numAgents; i++) {
                jointAction[i] = (i == agentId) ? moveAction : Action.noOp();
            }
            fullPlan.add(jointAction);
            tempState = tempState.apply(moveAction, agentId);
            globalTimeStep++;
        }
        
        logVerbose("[PP] Parked agent " + agentId + " at " + tempState.getAgentPosition(agentId)
                + " (was " + agentPos + ")");
        return tempState;
    }
    
    /**
     * Task-aware critical path check: determines if an agent's current position
     * lies on the critical path of any pending (not yet completed) subgoal.
     * 
     * This is the key improvement over position-based-only yielding: even if an
     * agent is in an open area (not on a goal, not in a corridor), it may still
     * block a future box-to-goal or agent-to-box path. By checking against all
     * remaining subgoals, we proactively move the agent before BSP even attempts
     * to plan around it.
     * 
     * @param agentPos       Current position of the agent
     * @param agentId        ID of the agent being checked
     * @param pendingSubgoals List of subgoals not yet completed
     * @param state          Current world state
     * @param level          Level definition
     * @return true if the agent is on the critical path of at least one pending subgoal
     */
    private boolean isOnPendingSubgoalCriticalPath(Position agentPos, int agentId,
            List<Subgoal> pendingSubgoals, State state, Level level) {
        if (pendingSubgoals == null || pendingSubgoals.isEmpty()) {
            return false;
        }
        
        for (Subgoal sg : pendingSubgoals) {
            if (sg.isAgentGoal) {
                // Agent goal: check if position is on agent's path to its goal
                Position otherAgentPos = state.getAgentPosition(sg.agentId);
                List<Position> path = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        otherAgentPos, sg.goalPos, level);
                if (path != null && path.contains(agentPos)) {
                    return true;
                }
            } else {
                // Box goal: check both agent-to-box path AND box-to-goal path
                Position boxPos = subgoalManager.findBestBoxForGoal(sg, state, level, completedBoxGoals);
                if (boxPos == null) continue;
                
                // Check box-to-goal path (and adjacent cells, since agent pushes from beside)
                List<Position> boxPath = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        boxPos, sg.goalPos, level);
                if (boxPath != null) {
                    for (Position p : boxPath) {
                        if (p.equals(agentPos)) {
                            return true;
                        }
                        for (Direction dir : Direction.values()) {
                            if (p.move(dir).equals(agentPos)) {
                                return true;
                            }
                        }
                    }
                }
                
                // Check agent-to-box path (the agent assigned to this subgoal must reach the box)
                Position sgAgentPos = state.getAgentPosition(sg.agentId);
                List<Position> agentToBox = pathAnalyzer.findPathIgnoringDynamicObstacles(
                        sgAgentPos, boxPos, level);
                if (agentToBox != null && agentToBox.contains(agentPos)) {
                    return true;
                }
            }
        }
        return false;
    }

    /** Get direction from one position to adjacent position. */
    private Direction getDirection(Position from, Position to) {
        int dRow = to.row - from.row;
        int dCol = to.col - from.col;
        for (Direction dir : Direction.values()) {
            if (dir.dRow == dRow && dir.dCol == dCol) return dir;
        }
        return null;
    }
    
    /**
     * Plan path for a single subgoal.
     * 
     * Frozen = wall semantics: completedBoxGoals are treated as impassable walls.
     * Caller-managed retry sequence with progressive relaxation:
     *   Round 1: All completedBoxGoals frozen (wall) — ST-A* then 2D A*
     *   Round 2: Self-blockers unlocked (targeted) — ST-A* then 2D A*
     *   Round 3: No frozen at all (desperate last resort) — 2D A*
     * 
     * For agent goals: same wall semantics, 2-round retry.
     */
    private List<Action> planSubgoal(Subgoal subgoal, State state, Level level, List<Subgoal> allSubgoals) {
        // Frozen = all completed box goals treated as walls,
        // EXCEPT goals that were intentionally displaced by tryRecovery.
        // Displaced goals remain in completedBoxGoals for dependency satisfaction
        // but must not block BSP pathfinding (the box is physically gone).
        Set<Position> frozen = new HashSet<>(completedBoxGoals);
        frozen.removeAll(displacedGoals);
        
        if (subgoal.isAgentGoal) {
            // Round 1: Agent goal with frozen protection (don't push completed goals)
            List<Action> path = boxSearchPlanner.searchForAgentGoal(
                    subgoal.agentId, subgoal.goalPos, state, level, frozen);
            if (path != null) return path;
            
            // Round 2: Without frozen (allow pushing if path is blocked by completed goals)
            if (!frozen.isEmpty()) {
                path = boxSearchPlanner.searchForAgentGoal(
                        subgoal.agentId, subgoal.goalPos, state, level, Collections.emptySet());
            }
            return path;
        } else {
            // Box-retry mechanism: try BSP with the best box candidate. If ALL rounds
            // fail AND Hungarian was used for selection, invalidate the cache and retry
            // with greedy fallback (which may pick a different, BSP-friendlier box).
            //
            // WHY: Hungarian optimizes for SIMULTANEOUS fulfillment (global minimum total
            // distance), but PP executes SERIALLY. A globally-optimal box can be locally
            // terrible for BSP's limited search budget (far away, complex path). The retry
            // gives greedy Layer 2 a chance to pick a closer, easier box.
            boolean fixedReliefBox = subgoal.isSyntheticRelief()
                    && subgoal.reliefCertificate.blockerStart != null;
            boolean usedHungarian = !fixedReliefBox && subgoalManager.hasHungarianCache();
            Position boxPos = fixedReliefBox
                    ? subgoal.reliefCertificate.blockerStart
                    : subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
            if (fixedReliefBox && state.getBoxAt(boxPos) != subgoal.boxType) {
                String cert = reliefCertificateLabel(subgoal);
                logNormalRepeated("synthetic.skip.blocker-moved " + cert,
                        "[PP][SYNTHETIC-RELIEF-SKIP] subgoal=" + subgoalLabel(subgoal)
                                + " reason=blocker-not-at-certificate-start cert=" + cert);
                return null;
            }
            
            for (int attempt = 0; attempt < 2; attempt++) {
                if (boxPos == null) {
                    if (attempt == 0 && usedHungarian) {
                        // Hungarian may have constrained the feasibility check too strictly.
                        // Retry with greedy-only.
                        logVerbose("[PP] findBestBoxForGoal returned null (attempt " + attempt 
                                + "), retrying without Hungarian");
                        subgoalManager.invalidateHungarianCache();
                        boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
                        continue;
                    }
                    logVerbose("[PP] findBestBoxForGoal returned null for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    return null;
                }
                
                logVerbose("[PP] Box " + subgoal.boxType + " at " + boxPos + " -> goal " + subgoal.goalPos
                        + " (agent " + subgoal.agentId + " at " + state.getAgentPosition(subgoal.agentId) 
                        + ", frozen=" + frozen + ", attempt=" + attempt + ")");

                
                // Dynamic BSP budget: scale search budget based on box-to-goal distance
                int dynamicBudget = computeDynamicBspBudget(boxPos, subgoal.goalPos);
                boxSearchPlanner.setMaxStatesOverride(dynamicBudget);
                
                try {
                // P2: Round 0 — IW(1) for escape subgoals only.
                // Per qanda.txt 75: "escape subgoal is exactly the sweet spot for IW(1):
                // no need for a good heuristic, only need to reach any reachable P_temp."
                boolean isEscapeSg = escapeGoalPositions.contains(subgoal.goalPos);
                int boxToGoalManhattan = boxPos.manhattanDistance(subgoal.goalPos);
                if (isEscapeSg) {
                    diagIw1Invoked++;
                    int iwBudget = Math.min(2000, dynamicBudget);
                    if (iw1Planner == null) {
                        iw1Planner = new IW1Planner(iwBudget);
                    }
                    List<Action> iwPath = iw1Planner.search(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, frozen);
                    if (iwPath != null) {
                        diagIw1Succeeded++;
                        if (mapf.planning.SearchConfig.isVerbose()) {
                            System.err.println("[PP] P2: IW(1) solved escape subgoal "
                                    + subgoal.boxType + " -> " + subgoal.goalPos
                                    + " in " + iwPath.size() + " steps");
                        }
                        return iwPath;
                    }
                }

                // Round 1: ST-A* with all frozen as walls
                List<Action> path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                        subgoal.goalPos, subgoal.boxType, state, level, frozen,
                        reservationTable, globalTimeStep);
                
                // Round 1b: 2D A* with all frozen as walls
                if (path == null) {
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, frozen);
                }
                if (path != null) {
                    return path;
                }
                logVerbose("[PP] Round 1 (frozen) failed for " + subgoal.boxType + " -> " + subgoal.goalPos);

                // P4b: Round 1.5 — IW(1) FIND-ROUTE FALLBACK (per claudeopus47 §1.2.2 / §3.2)
                // for moderate-distance find-route subgoals when A*/BSP failed under frozen.
                // Per qanda.txt §3.3 / §4.4: IW(1) is the sweet spot for single-box find-route
                // where A*+Manhattan badly underestimates (long detours around walls).
                // Used as a FALLBACK (not Round 0 speculation) to preserve the fast A* path
                // on simple subgoals. Skip very short paths (A* trivially solves them)
                // and very long paths (IW(1) state explosion).
                if (!isEscapeSg
                        && boxToGoalManhattan >= IW1_FINDROUTE_MIN_MANHATTAN
                        && boxToGoalManhattan <= IW1_FINDROUTE_MAX_MANHATTAN) {
                    diagIw1Invoked++;
                    int iwBudget = Math.min(800, dynamicBudget);
                    if (iw1Planner == null) {
                        iw1Planner = new IW1Planner(iwBudget);
                    }
                    List<Action> iwPath = iw1Planner.search(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, frozen);
                    if (iwPath != null) {
                        diagIw1Succeeded++;
                        if (mapf.planning.SearchConfig.isMinimal()) {
                            System.err.println("[PP] P4b: IW(1) fallback solved find-route(d="
                                    + boxToGoalManhattan + ") subgoal "
                                    + subgoal.boxType + " -> " + subgoal.goalPos
                                    + " in " + iwPath.size() + " steps");
                        }
                        return iwPath;
                    }
                }
                
                // Round 2a: Same-color self-blocking recovery — find frozen goals of the
                // same color that block the agent's path to the target box.
                if (!frozen.isEmpty()) {
                    Color agentColor = level.getAgentColor(subgoal.agentId);
                    Set<Position> selfBlockers = findPathBlockers(
                            subgoal.agentId, boxPos, state, level, frozen, agentColor, true);
                    
                    if (!selfBlockers.isEmpty()) {
                        Set<Position> relaxedFrozen = new HashSet<>(frozen);
                        relaxedFrozen.removeAll(selfBlockers);
                        
                        logVerbose("[PP] Targeted unlock (same-color): " + selfBlockers.size() 
                                + " blockers for " + subgoal.boxType + " -> " + subgoal.goalPos);
                        
                        path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen,
                                reservationTable, globalTimeStep);
                        if (path == null) {
                            path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                    subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen);
                        }
                        if (path != null) return path;
                    }
                }
                
                // Round 2b: Cross-color blocking recovery — find ANY frozen goal (regardless
                // of color) that blocks the path. This handles multi-agent scenarios where
                // e.g. a blue box on its goal blocks a red agent's path (MADS-level pattern).
                // More permissive than Round 2a but still targeted (not blanket removal).
                if (!frozen.isEmpty()) {
                    Set<Position> crossColorBlockers = findPathBlockers(
                            subgoal.agentId, boxPos, state, level, frozen, null, false);
                    
                    if (!crossColorBlockers.isEmpty()) {
                        Set<Position> relaxedFrozen = new HashSet<>(frozen);
                        relaxedFrozen.removeAll(crossColorBlockers);
                        
                        logVerbose("[PP] Targeted unlock (cross-color): " + crossColorBlockers.size() 
                                + " blockers for " + subgoal.boxType + " -> " + subgoal.goalPos);
                        
                        path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen,
                                reservationTable, globalTimeStep);
                        if (path == null) {
                            path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                    subgoal.goalPos, subgoal.boxType, state, level, relaxedFrozen);
                        }
                        if (path != null) return path;
                    }
                }
                
                // Round 3: No frozen at all (desperate — allows disturbing any completed goal)
                // Progressive budget: increase by 50% for desperate search (harder subgoal)
                if (!frozen.isEmpty()) {
                    boxSearchPlanner.setMaxStatesOverride(Math.min(dynamicBudget * 3 / 2, effectiveMaxBspBudget * 2));
                    logVerbose("[PP] Round 3 (no frozen, budget=" + (dynamicBudget * 3 / 2) + ") for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, Collections.emptySet());
                    if (path != null) {
                        return path;
                    }
                }
                
                // Round 4: Weighted A* escalation — use w=3 for faster (suboptimal) search
                // Progressive budget: double budget for weighted A* (consumes states faster)
                if (boxSearchPlanner.getWeight() < 2.0) {
                    boxSearchPlanner.setMaxStatesOverride(Math.min(dynamicBudget * 2, effectiveMaxBspBudget * 2));
                    double savedWeight = boxSearchPlanner.getWeight();
                    boxSearchPlanner.setWeight(3.0);
                    logVerbose("[PP] Round 4 (weighted A* w=3, budget=" + (dynamicBudget * 2) + ") for " + subgoal.boxType + " -> " + subgoal.goalPos);
                    path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                            subgoal.goalPos, subgoal.boxType, state, level, frozen);
                    if (path == null && !frozen.isEmpty()) {
                        path = boxSearchPlanner.searchForSubgoal(subgoal.agentId, boxPos,
                                subgoal.goalPos, subgoal.boxType, state, level, Collections.emptySet());
                    }
                    boxSearchPlanner.setWeight(savedWeight);
                    if (path != null) return path;
                }
                
                // All BSP rounds failed for this box. If Hungarian was used on the first
                // attempt, the globally-optimal pick may be BSP-hard for serial execution.
                // Invalidate cache and retry — greedy Layer 2 may pick a different box.
                if (attempt == 0 && usedHungarian) {
                    Position failedBox = boxPos;
                    subgoalManager.invalidateHungarianCache();
                    boxPos = subgoalManager.findBestBoxForGoal(subgoal, state, level, allSubgoals, completedBoxGoals);
                    // If greedy picks the SAME box, no point retrying BSP rounds
                    if (boxPos != null && boxPos.equals(failedBox)) {
                        logVerbose("[PP] Greedy fallback picked same box " + boxPos + " — no retry");
                        return null;
                    }
                    if (boxPos != null) {
                        logVerbose("[PP] All rounds failed for Hungarian box " + failedBox 
                                + " — retrying with greedy pick " + boxPos);
                    }
                    continue; // retry BSP rounds with new box
                }
                
                logVerbose("[PP] All rounds exhausted for " + subgoal.boxType + " -> " + subgoal.goalPos);
                return null;
                } finally {
                    boxSearchPlanner.clearMaxStatesOverride();
                }
            }
            return null; // should not reach here, but safety
        }
    }
    
    /**
     * Finds frozen goals that block the agent's path to the target box.
     * 
     * Two modes:
     * - sameColorOnly=true: Only considers frozen goals whose box color matches agentColor.
     *   These are "self-blockers" — the agent filled them and now can't pass through.
     * - sameColorOnly=false: Considers ALL frozen goals regardless of color.
     *   Handles cross-color blocking (e.g., blue box on its goal blocks red agent's path).
     *   This targeted approach avoids falling through to Round 3's blanket frozen removal.
     * 
     * Algorithm: BFS from agent position to find reachable cells. If target box is
     * unreachable, identify frozen goals on the boundary between reachable and unreachable
     * regions. Verify that removing candidates actually unblocks the path.
     * 
     * @param agentId       The agent that needs to reach the target box
     * @param targetBox     Position of the box to be moved
     * @param state         Current world state
     * @param level         Level definition
     * @param hardFrozen    Set of positions treated as frozen (wall-like)
     * @param agentColor    Color of the agent (used when sameColorOnly=true; nullable when false)
     * @param sameColorOnly If true, only return blockers matching agentColor
     * @return Set of frozen positions whose unlock would unblock the agent's path
     */
    private Set<Position> findPathBlockers(int agentId, Position targetBox, State state,
            Level level, Set<Position> hardFrozen, Color agentColor, boolean sameColorOnly) {
        Set<Position> blockers = new HashSet<>();
        if (sameColorOnly && agentColor == null) return blockers;
        
        Position agentPos = state.getAgentPosition(agentId);
        
        // BFS from agent position to find reachable cells (agent walk, ignoring box pushability)
        Set<Position> reachable = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        reachable.add(agentPos);
        queue.add(agentPos);
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (reachable.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (state.getBoxes().containsKey(next)) continue; // boxes block walking
                reachable.add(next);
                queue.add(next);
            }
        }
        
        // Check if agent can reach any neighbor of the target box (to push/pull)
        boolean canReachBox = false;
        for (Direction dir : Direction.values()) {
            if (reachable.contains(targetBox.move(dir))) {
                canReachBox = true;
                break;
            }
        }
        
        if (canReachBox) return blockers; // agent CAN reach box, no blocking
        
        // Agent can't reach box. Find frozen goals on the boundary of reachable region.
        Set<Position> candidates = new HashSet<>();
        for (Position frozen : hardFrozen) {
            char frozenBoxType = level.getBoxGoal(frozen.row, frozen.col);
            if (frozenBoxType == '\0') continue;
            
            // Color filter: in same-color mode, skip different-color frozen goals
            if (sameColorOnly) {
                Color frozenColor = level.getBoxColor(frozenBoxType);
                if (!agentColor.equals(frozenColor)) continue;
            }
            
            // Check if this frozen goal has a box and is adjacent to the reachable region
            Character boxAtFrozen = state.getBoxes().get(frozen);
            if (boxAtFrozen == null) continue; // no box here, not blocking
            
            boolean isOnBoundary = false;
            for (Direction dir : Direction.values()) {
                if (reachable.contains(frozen.move(dir))) {
                    isOnBoundary = true;
                    break;
                }
            }
            if (isOnBoundary) {
                candidates.add(frozen);
            }
        }
        
        if (candidates.isEmpty()) return blockers;
        
        // VERIFICATION: re-run BFS excluding candidate blockers to confirm
        // that removing them actually makes the target box reachable.
        // This prevents false positives where other obstacles are the real cause.
        Set<Position> verifyReachable = new HashSet<>();
        Queue<Position> verifyQueue = new LinkedList<>();
        verifyReachable.add(agentPos);
        verifyQueue.add(agentPos);
        
        while (!verifyQueue.isEmpty()) {
            Position current = verifyQueue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (verifyReachable.contains(next)) continue;
                if (level.isWall(next)) continue;
                // Allow walking through candidate blocker positions
                if (state.getBoxes().containsKey(next) && !candidates.contains(next)) continue;
                verifyReachable.add(next);
                verifyQueue.add(next);
            }
        }
        
        boolean canReachAfterRelax = false;
        for (Direction dir : Direction.values()) {
            if (verifyReachable.contains(targetBox.move(dir))) {
                canReachAfterRelax = true;
                break;
            }
        }
        
        // Only return blockers if removing them actually unblocks the path
        if (canReachAfterRelax) {
            blockers.addAll(candidates);
        }
        
        return blockers;
    }
    
    /** Verify that a subgoal was actually reached. */
    private boolean verifyGoalReached(Subgoal subgoal, State state, Level level) {
        if (subgoal.isAgentGoal) {
            return state.getAgentPosition(subgoal.agentId).equals(subgoal.goalPos);
        } else {
            return state.getBoxAt(subgoal.goalPos) == subgoal.boxType;
        }
    }

    private SubgoalEval captureSubgoalEval(Subgoal subgoal, State state, Level level) {
        Set<Position> reachable = bfsReachableForEval(state.getAgentPosition(subgoal.agentId), state, level);
        return new SubgoalEval(
                stateSignature(state),
                reachable.size(),
                totalReachableCells(state, level),
                adjacentReachable(subgoal.goalPos, reachable, level),
                subgoal.isAgentGoal ? Collections.emptyList() : boxPositionsOfType(state, subgoal.boxType));
    }

    private boolean acceptReachedSubgoal(String phase, Subgoal subgoal, SubgoalEval before,
                                         State afterState, Level level, int planStart, int planEnd,
                                         List<Subgoal> allSubgoals) {
        if (!isSyntheticBoxTarget(subgoal, level)) return true;

        SubgoalEval after = captureSubgoalEval(subgoal, afterState, level);
        boolean sameState = before.stateSignature.equals(after.stateSignature);
        if (subgoal.isSyntheticRelief()) {
            boolean resolved = reliefCertificateResolved(subgoal, afterState, level);
            if (!sameState && resolved) {
                Subgoal blockedFuture = syntheticParkingBlocksRemainingTask(subgoal, afterState, level, allSubgoals);
                if (blockedFuture != null) {
                    syntheticReliefBlacklist.add(subgoal.goalPos);
                    logNormal("[PP][SYNTHETIC-RELIEF-REJECT] phase=" + phase
                            + " steps=" + planStart + ".." + Math.max(planStart, planEnd - 1)
                            + " actions=" + Math.max(0, planEnd - planStart)
                            + " subgoal=" + subgoalLabel(subgoal)
                            + " reason=blocks-future-task"
                            + " blockedFuture=" + subgoalLabel(blockedFuture)
                            + " cert=" + reliefCertificateLabel(subgoal)
                            + " " + subgoal.boxType + "=" + before.boxPositions + "->" + after.boxPositions);
                    return false;
                }
                return true;
            }
            syntheticReliefBlacklist.add(subgoal.goalPos);
            logNormal("[PP][SYNTHETIC-RELIEF-REJECT] phase=" + phase
                    + " steps=" + planStart + ".." + Math.max(planStart, planEnd - 1)
                    + " actions=" + Math.max(0, planEnd - planStart)
                    + " subgoal=" + subgoalLabel(subgoal)
                    + " reason=" + (sameState ? "same-state" : "certificate-not-resolved")
                    + " cert=" + reliefCertificateLabel(subgoal)
                    + " " + subgoal.boxType + "=" + before.boxPositions + "->" + after.boxPositions);
            return false;
        }

        int agentReachDelta = after.agentReachable - before.agentReachable;
        int totalReachDelta = after.totalReachable - before.totalReachable;
        boolean openedTargetAdjacency = after.goalAdjacentReachable && !before.goalAdjacentReachable;
        boolean reachGain = agentReachDelta > 0 || totalReachDelta > 0;

        if (!sameState && (reachGain || openedTargetAdjacency)) {
            Subgoal blockedFuture = syntheticParkingBlocksRemainingTask(subgoal, afterState, level, allSubgoals);
            if (blockedFuture != null) {
                syntheticReliefBlacklist.add(subgoal.goalPos);
                logNormal("[PP][SYNTHETIC-RELIEF-REJECT] phase=" + phase
                        + " steps=" + planStart + ".." + Math.max(planStart, planEnd - 1)
                        + " actions=" + Math.max(0, planEnd - planStart)
                        + " subgoal=" + subgoalLabel(subgoal)
                        + " reason=blocks-future-task"
                        + " blockedFuture=" + subgoalLabel(blockedFuture)
                        + " agentReach=" + before.agentReachable + "->" + after.agentReachable
                        + " totalReach=" + before.totalReachable + "->" + after.totalReachable
                        + " " + subgoal.boxType + "=" + before.boxPositions + "->" + after.boxPositions);
                return false;
            }
            return true;
        }

        syntheticReliefBlacklist.add(subgoal.goalPos);
        logNormal("[PP][SYNTHETIC-RELIEF-REJECT] phase=" + phase
                + " steps=" + planStart + ".." + Math.max(planStart, planEnd - 1)
                + " actions=" + Math.max(0, planEnd - planStart)
                + " subgoal=" + subgoalLabel(subgoal)
                + " reason=" + (sameState ? "same-state" : "no-reach-gain")
                + " agentReach=" + before.agentReachable + "->" + after.agentReachable
                + " totalReach=" + before.totalReachable + "->" + after.totalReachable
                + " goalAdj=" + before.goalAdjacentReachable + "->" + after.goalAdjacentReachable
                + " " + subgoal.boxType + "=" + before.boxPositions + "->" + after.boxPositions);
        return false;
    }

    private Subgoal syntheticParkingBlocksRemainingTask(Subgoal synthetic, State state, Level level,
                                                        List<Subgoal> allSubgoals) {
        Position parked = synthetic.goalPos;
        Character parkedBox = state.getBoxAt(parked);
        if (parkedBox == null || parkedBox != synthetic.boxType) return null;

        List<Subgoal> goalsToCheck = cachedSubgoalOrder != null ? cachedSubgoalOrder : allSubgoals;
        for (Subgoal remaining : goalsToCheck) {
            if (remaining == synthetic) continue;
            if (isSyntheticBoxTarget(remaining, level)) continue;
            if (remaining.isAgentGoal) {
                if (remaining.goalPos.equals(state.getAgentPosition(remaining.agentId))) continue;
            } else {
                if (completedBoxGoals.contains(remaining.goalPos)) continue;
                Character atGoal = state.getBoxAt(remaining.goalPos);
                if (atGoal != null && atGoal == remaining.boxType) continue;
            }

            List<Position> blockers = findAccessBlockersForTask(remaining, state, level, allSubgoals);
            if (blockers.contains(parked)) {
                return remaining;
            }
        }
        return null;
    }

    private SealRisk findSealRisk(Subgoal completedSubgoal, State beforeState, State afterState,
                                  Level level, List<Subgoal> allSubgoals) {
        if (completedSubgoal.isAgentGoal || isSyntheticBoxTarget(completedSubgoal, level)) {
            return null;
        }
        if (afterState.getBoxAt(completedSubgoal.goalPos) != completedSubgoal.boxType) {
            return null;
        }

        Set<Position> frozen = physicallySatisfiedBoxGoals(afterState, level);
        frozen.add(completedSubgoal.goalPos);

        for (Subgoal future : collectFutureTasksForSeal(completedSubgoal, afterState, level, allSubgoals)) {
            if (canServiceFutureWithFrozen(future, afterState, level, frozen)) {
                continue;
            }
            SealStagingPlan staging = null;
            boolean allowStaging = future.isAgentGoal;
            if (allowStaging) {
                staging = findSealStagingPlan(completedSubgoal, future,
                        beforeState, afterState, level, frozen);
            }
            return new SealRisk(future, frozen, staging, allowStaging);
        }
        return null;
    }

    private List<Subgoal> collectFutureTasksForSeal(Subgoal completedSubgoal, State state,
                                                    Level level, List<Subgoal> allSubgoals) {
        Map<Position, Subgoal> future = new LinkedHashMap<>();
        if (allSubgoals != null) {
            for (Subgoal sg : allSubgoals) {
                addFutureSealTask(future, sg, completedSubgoal, state, level);
            }
        }
        for (Position goalPos : level.getAllBoxGoalPositions()) {
            char goalType = level.getBoxGoal(goalPos);
            addFutureSealGoal(future, goalType, goalPos, completedSubgoal, state, level);
        }
        for (Map.Entry<Integer, Position> entry : level.getAgentGoalPositionMap().entrySet()) {
            int agentId = entry.getKey();
            Position goalPos = entry.getValue();
            if (agentId >= state.getNumAgents()) continue;
            if (goalPos.equals(state.getAgentPosition(agentId))) continue;
            future.putIfAbsent(goalPos, new Subgoal(agentId, '\0', goalPos, true));
        }
        return new ArrayList<>(future.values());
    }

    private void addFutureSealTask(Map<Position, Subgoal> future, Subgoal sg,
                                   Subgoal completedSubgoal, State state, Level level) {
        if (sg == null || sg.goalPos.equals(completedSubgoal.goalPos)) return;
        if (sg.isSyntheticRelief() || isSyntheticBoxTarget(sg, level)) return;
        if (sg.isAgentGoal) {
            Position agentPos = state.getAgentPosition(sg.agentId);
            if (agentPos != null && !agentPos.equals(sg.goalPos)) {
                future.putIfAbsent(sg.goalPos, sg);
            }
            return;
        }
        addFutureSealGoal(future, sg.boxType, sg.goalPos, completedSubgoal, state, level);
    }

    private void addFutureSealGoal(Map<Position, Subgoal> future, char goalType, Position goalPos,
                                   Subgoal completedSubgoal, State state, Level level) {
        if (goalType == '\0' || goalPos.equals(completedSubgoal.goalPos)) return;
        if (state.getBoxAt(goalPos) == goalType) return;
        int agentId = findHelperAgentForBox(goalType, state, level, goalPos);
        if (agentId < 0) return;
        future.putIfAbsent(goalPos, new Subgoal(agentId, goalType, goalPos, false));
    }

    private Set<Position> physicallySatisfiedBoxGoals(State state, Level level) {
        Set<Position> satisfied = new HashSet<>();
        for (Position goalPos : level.getAllBoxGoalPositions()) {
            char goalType = level.getBoxGoal(goalPos);
            if (goalType != '\0' && state.getBoxAt(goalPos) == goalType) {
                satisfied.add(goalPos);
            }
        }
        return satisfied;
    }

    private boolean canServiceFutureWithFrozen(Subgoal goal, State state, Level level,
                                               Set<Position> frozen) {
        if (goal.isAgentGoal) {
            Set<Position> reachable = agentGoalReachableWithFrozen(
                    goal.agentId, state, level, frozen);
            return reachable.contains(goal.goalPos);
        }

        Set<Position> serviceRegion = serviceRegionForGoal(goal, state, level, frozen);
        if (serviceRegion.isEmpty()) return false;

        Color goalColor = level.getBoxColor(goal.boxType);
        if (goalColor == null) return true;
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            if (!goalColor.equals(level.getAgentColor(agentId))) continue;
            Set<Position> reachable = bfsReachableWithFrozenGoals(
                    state.getAgentPosition(agentId), level, frozen);
            if (!Collections.disjoint(reachable, serviceRegion)) {
                return true;
            }
        }
        return false;
    }

    private Set<Position> serviceRegionForGoal(Subgoal goal, State state, Level level,
                                               Set<Position> frozen) {
        Set<Position> seeds = new LinkedHashSet<>();
        if (!level.isWall(goal.goalPos) && !frozen.contains(goal.goalPos)) {
            seeds.add(goal.goalPos);
        }
        for (Direction dir : Direction.values()) {
            Position adj = goal.goalPos.move(dir);
            if (!level.isWall(adj) && !frozen.contains(adj)) {
                seeds.add(adj);
            }
        }

        Set<Position> region = bfsReachableWithFrozenGoals(seeds, level, frozen);
        if (region.isEmpty()) return Collections.emptySet();

        boolean hasCandidateBox = false;
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() != goal.boxType) continue;
            Position boxPos = entry.getKey();
            if (state.getBoxAt(boxPos) == level.getBoxGoal(boxPos)) continue;
            if (region.contains(boxPos)) {
                hasCandidateBox = true;
                break;
            }
            for (Direction dir : Direction.values()) {
                if (region.contains(boxPos.move(dir))) {
                    hasCandidateBox = true;
                    break;
                }
            }
            if (hasCandidateBox) break;
        }
        return hasCandidateBox ? region : Collections.emptySet();
    }

    private SealStagingPlan findSealStagingPlan(Subgoal sealingSubgoal, Subgoal future,
                                                State beforeState, State afterState,
                                                Level level, Set<Position> frozenAfterSeal) {
        Color futureColor = future.isAgentGoal
                ? level.getAgentColor(future.agentId)
                : level.getBoxColor(future.boxType);
        if (futureColor == null) return null;

        Set<Position> serviceRegion = future.isAgentGoal
                ? terminalAgentRegionForGoal(future, level, frozenAfterSeal)
                : serviceRegionForGoal(future, afterState, level, frozenAfterSeal);
        if (serviceRegion.isEmpty()) return null;

        Set<Position> stagingExclusion = sealStagingExclusion(sealingSubgoal, afterState, level);
        Set<Position> preferredTargets = new HashSet<>();
        Set<Position> fallbackTargets = new HashSet<>();
        if (future.isAgentGoal) {
            Position terminal = future.goalPos;
            if (beforeState.isFree(terminal, level) && afterState.isFree(terminal, level)) {
                preferredTargets.add(terminal);
                fallbackTargets.add(terminal);
            }
        }
        if (!future.isAgentGoal) {
            for (Position p : serviceRegion) {
                if (!beforeState.isFree(p, level)) continue;
                if (!afterState.isFree(p, level)) continue;
                if (stagingExclusion.contains(p)) {
                    continue;
                }
                fallbackTargets.add(p);
                if (!level.hasBoxGoal(p) && !level.hasAgentGoal(p)) {
                    preferredTargets.add(p);
                }
            }
        }
        if (fallbackTargets.isEmpty()) return null;

        SealStagingPlan best = null;
        for (int agentId = 0; agentId < beforeState.getNumAgents(); agentId++) {
            if (future.isAgentGoal && agentId != future.agentId) continue;
            if (agentId == sealingSubgoal.agentId) continue;
            if (!futureColor.equals(level.getAgentColor(agentId))) continue;

            String nogood = sealStagingNogoodKey(sealingSubgoal, future, agentId);
            if (sealStagingNogoods.contains(nogood)) continue;

            List<Action> path = findMoveOnlyPathToAny(agentId, beforeState, level, preferredTargets);
            if ((path == null || path.isEmpty()) && !preferredTargets.equals(fallbackTargets)) {
                path = findMoveOnlyPathToAny(agentId, beforeState, level, fallbackTargets);
            }
            if (path == null || path.isEmpty()) continue;

            Position target = replayAgentPath(beforeState.getAgentPosition(agentId), path);
            if (best == null || path.size() < best.path.size()) {
                best = new SealStagingPlan(agentId, target, path);
            }
        }
        return best;
    }

    private Set<Position> terminalAgentRegionForGoal(Subgoal agentGoal, Level level,
                                                     Set<Position> frozen) {
        if (!agentGoal.isAgentGoal) return Collections.emptySet();
        return bfsReachableWithFrozenGoals(agentGoal.goalPos, level, frozen);
    }

    private Set<Position> sealStagingExclusion(Subgoal sealingSubgoal, State afterState, Level level) {
        Set<Position> excluded = new HashSet<>();
        excluded.add(sealingSubgoal.goalPos);
        for (Direction dir : Direction.values()) {
            Position one = sealingSubgoal.goalPos.move(dir);
            if (!level.isWall(one)) excluded.add(one);
            Position two = one.move(dir);
            if (!level.isWall(two)) excluded.add(two);
        }
        Position sealingAgent = afterState.getAgentPosition(sealingSubgoal.agentId);
        if (sealingAgent != null) {
            excluded.add(sealingAgent);
            for (Direction dir : Direction.values()) {
                Position adj = sealingAgent.move(dir);
                if (!level.isWall(adj)) excluded.add(adj);
            }
        }
        return excluded;
    }

    private State tryStageForSealRisk(String phase, Subgoal sealingSubgoal, SealRisk risk,
                                      State beforeState, Level level, List<Action[]> fullPlan,
                                      int numAgents) {
        if (!risk.allowStaging) {
            logNormal("[PP][SEAL-REJECT] phase=" + phase
                    + " sealing=" + subgoalLabel(sealingSubgoal)
                    + " would-seal=" + subgoalLabel(risk.future)
                    + " reason=blocks-future-task frozen=" + formatPositions(risk.frozen));
            return null;
        }

        if (risk.staging == null) {
            logNormal("[PP][SEAL-REJECT] phase=" + phase
                    + " sealing=" + subgoalLabel(sealingSubgoal)
                    + " would-seal=" + subgoalLabel(risk.future)
                    + " reason=no-staging-path frozen=" + formatPositions(risk.frozen));
            return null;
        }

        State staged = appendSerializedAgentPath(risk.staging.agentId, risk.staging.path,
                beforeState, level, fullPlan, numAgents);
        if (staged == null) {
            sealStagingNogoods.add(sealStagingNogoodKey(sealingSubgoal, risk.future, risk.staging.agentId));
            logNormal("[PP][SEAL-REJECT] phase=" + phase
                    + " sealing=" + subgoalLabel(sealingSubgoal)
                    + " would-seal=" + subgoalLabel(risk.future)
                    + " reason=staging-exec-failed agent=" + risk.staging.agentId
                    + " target=" + risk.staging.target);
            return null;
        }

        logNormal("[PP][SEAL-STAGE] phase=" + phase
                + " sealing=" + subgoalLabel(sealingSubgoal)
                + " future=" + subgoalLabel(risk.future)
                + " agent=" + risk.staging.agentId
                + " target=" + risk.staging.target
                + " actions=" + risk.staging.path.size());
        return staged;
    }

    private List<Action> findMoveOnlyPathToAny(int agentId, State state, Level level,
                                               Set<Position> targets) {
        if (targets == null || targets.isEmpty()) return null;
        Position start = state.getAgentPosition(agentId);
        if (start == null) return null;
        if (targets.contains(start)) return Collections.emptyList();

        Queue<Position> queue = new ArrayDeque<>();
        Set<Position> visited = new HashSet<>();
        Map<Position, Position> parent = new HashMap<>();
        Map<Position, Action> parentAction = new HashMap<>();
        visited.add(start);
        queue.add(start);

        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (!level.isFree(next)) continue;
                if (state.hasBoxAt(next)) continue;
                if (state.hasAgentAt(next)) continue;

                visited.add(next);
                parent.put(next, current);
                parentAction.put(next, Action.move(dir));
                if (targets.contains(next)) {
                    return reconstructMoveOnlyPath(start, next, parent, parentAction);
                }
                queue.add(next);
            }
        }
        return null;
    }

    private List<Action> reconstructMoveOnlyPath(Position start, Position end,
                                                 Map<Position, Position> parent,
                                                 Map<Position, Action> parentAction) {
        List<Action> path = new ArrayList<>();
        Position current = end;
        while (!current.equals(start)) {
            Action action = parentAction.get(current);
            Position prev = parent.get(current);
            if (action == null || prev == null) return null;
            path.add(action);
            current = prev;
        }
        Collections.reverse(path);
        return path;
    }

    private Position replayAgentPath(Position start, List<Action> path) {
        Position current = start;
        if (current == null || path == null) return current;
        for (Action action : path) {
            if (action.type == Action.ActionType.MOVE) {
                current = current.move(action.agentDir);
            }
        }
        return current;
    }

    private State appendSerializedAgentPath(int agentId, List<Action> path, State state,
                                            Level level, List<Action[]> fullPlan, int numAgents) {
        State current = state;
        for (Action action : path) {
            if (action.type != Action.ActionType.MOVE) return null;
            if (!current.isApplicable(action, agentId, level)) return null;
            Action[] jointAction = new Action[numAgents];
            Arrays.fill(jointAction, Action.noOp());
            jointAction[agentId] = action;
            fullPlan.add(jointAction);
            current = applyJointAction(jointAction, current, level, numAgents);
            globalTimeStep++;
        }
        return current;
    }

    private String sealStagingNogoodKey(Subgoal sealingSubgoal, Subgoal future, int agentId) {
        return sealingSubgoal.boxType + "@" + sealingSubgoal.goalPos
                + "|future=" + future.boxType + "@" + future.goalPos
                + "|agent=" + agentId;
    }

    private Set<Position> bfsReachableWithFrozenGoals(Position start, Level level, Set<Position> frozen) {
        if (start == null) return Collections.emptySet();
        return bfsReachableWithFrozenGoals(Collections.singleton(start), level, frozen);
    }

    private Set<Position> bfsReachableWithFrozenGoals(Collection<Position> starts, Level level, Set<Position> frozen) {
        Set<Position> visited = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        for (Position start : starts) {
            if (start == null || level.isWall(start) || frozen.contains(start)) continue;
            if (visited.add(start)) {
                queue.add(start);
            }
        }
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next) || frozen.contains(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    private String formatPositions(Set<Position> positions) {
        List<String> out = new ArrayList<>();
        for (Position p : positions) out.add(p.row + "," + p.col);
        Collections.sort(out);
        return out.toString();
    }

    private boolean isSyntheticBoxTarget(Subgoal subgoal, Level level) {
        return !subgoal.isAgentGoal && level.getBoxGoal(subgoal.goalPos) == '\0';
    }

    private boolean isEscapeSubgoal(Subgoal subgoal) {
        return subgoal != null && !subgoal.isAgentGoal
                && escapeGoalPositions.contains(subgoal.goalPos);
    }

    private boolean reliefCertificateStillBlocked(Subgoal subgoal, State state, Level level) {
        if (!subgoal.isSyntheticRelief()) return false;
        ReliefCertificate cert = subgoal.reliefCertificate;
        return state.hasBoxAt(cert.blockerStart);
    }

    private boolean reliefCertificateResolved(Subgoal subgoal, State state, Level level) {
        if (!subgoal.isSyntheticRelief()) return false;
        ReliefCertificate cert = subgoal.reliefCertificate;
        return !state.hasBoxAt(cert.blockerStart);
    }

    private String reliefCertificateLabel(Subgoal subgoal) {
        if (!subgoal.isSyntheticRelief()) return "-";
        ReliefCertificate cert = subgoal.reliefCertificate;
        return cert.kind
                + " blocker=" + cert.blockerStart
                + " blockedAgent=" + cert.blockedAgentId
                + " primary=" + cert.primary
                + " secondary=" + cert.secondary;
    }

    private boolean hasOpenGoalTransaction(State state, Level level) {
        return !openGoalDebtPositions(state, level).isEmpty();
    }

    private Set<Position> openGoalDebtPositions(State state, Level level) {
        LinkedHashSet<Position> debt = new LinkedHashSet<>();
        addOpenGoalDebt(debt, lastClearingDisplacedGoals, state, level);
        addOpenGoalDebt(debt, displacedGoals, state, level);
        addOpenGoalDebt(debt, suspendedTransitGoals, state, level);
        addOpenGoalDebt(debt, completedBoxGoals, state, level);
        return debt;
    }

    private void addOpenGoalDebt(Set<Position> debt, Collection<Position> positions,
                                 State state, Level level) {
        if (positions == null || state == null) return;
        for (Position goalPos : positions) {
            if (goalPos != null && !isSatisfiedBoxGoal(goalPos, state, level)) {
                debt.add(goalPos);
            }
        }
    }

    private boolean isSatisfiedBoxGoal(Position goalPos, State state, Level level) {
        if (goalPos == null || state == null) return false;
        char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
        if (goalType == '\0') return true;
        Character actual = state.getBoxAt(goalPos);
        return actual != null && actual == goalType;
    }

    private State rollbackToStablePartialIfNeeded(State initialState, State currentState,
                                                  Level level, List<Action[]> fullPlan,
                                                  int numAgents, int stablePlanSize,
                                                  String reason) {
        Set<Position> debt = openGoalDebtPositions(currentState, level);
        if (debt.isEmpty() || stablePlanSize >= fullPlan.size()) {
            return currentState;
        }

        int from = fullPlan.size();
        rollbackPlanTo(fullPlan, stablePlanSize);
        State stableState = recomputeState(initialState, fullPlan, level, numAgents);
        displacedGoals.clear();
        lastClearingDisplacedGoals.clear();
        lastBorrowedGoalBoxPositions.clear();
        revalidateCompletedGoals(stableState, level);
        planMerger.clearAllPlans();
        storedPlanSubgoals.clear();
        logNormal("[PP][STABLE-PARTIAL] reason=" + reason
                + " rollback=" + from + "->" + stablePlanSize
                + " openDebt=" + formatPositions(debt));
        return stableState;
    }

    private void rollbackPlanTo(List<Action[]> fullPlan, int planSizeBefore) {
        while (fullPlan.size() > planSizeBefore) {
            fullPlan.remove(fullPlan.size() - 1);
            globalTimeStep--;
        }
    }

    private void restoreDisplacedGoals(Set<Position> snapshot) {
        if (snapshot == null) return;
        displacedGoals.clear();
        displacedGoals.addAll(snapshot);
        lastClearingDisplacedGoals.clear();
        lastBorrowedGoalBoxPositions.clear();
    }

    private void restoreTransactionBookkeeping(Set<Position> displacedSnapshot,
                                               Set<Position> completedSnapshot,
                                               Map<Position, Integer> completionCountSnapshot) {
        restoreDisplacedGoals(displacedSnapshot);
        if (completedSnapshot != null) {
            completedBoxGoals.clear();
            completedBoxGoals.addAll(completedSnapshot);
        }
        if (completionCountSnapshot != null) {
            goalCompletionCount.clear();
            goalCompletionCount.putAll(completionCountSnapshot);
        }
        cachedSubgoalOrder = null;
        subgoalManager.invalidateHungarianCache();
    }

    private void logAcceptedSubgoalEval(String phase, Subgoal subgoal, SubgoalEval before,
                                        State afterState, Level level, int planStart, int planEnd) {
        if (!SearchConfig.isNormal()) return;
        SubgoalEval after = captureSubgoalEval(subgoal, afterState, level);
        boolean sameState = before.stateSignature.equals(after.stateSignature);
        boolean syntheticBoxTarget = isSyntheticBoxTarget(subgoal, level);
        int agentReachDelta = after.agentReachable - before.agentReachable;
        int totalReachDelta = after.totalReachable - before.totalReachable;
        String verdict;
        if (sameState) {
            verdict = "CYCLE";
        } else if (subgoal.isSyntheticRelief() && reliefCertificateResolved(subgoal, afterState, level)) {
            verdict = "CERTIFICATE_RESOLVED";
        } else if (syntheticBoxTarget && totalReachDelta <= 0 && agentReachDelta <= 0) {
            verdict = "SYNTHETIC_NO_REACH_GAIN";
        } else if (after.goalAdjacentReachable && !before.goalAdjacentReachable) {
            verdict = "TARGET_ADJ_OPENED";
        } else {
            verdict = "ACCEPTED";
        }

        String msg = "[PP][SUBGOAL-EVAL] phase=" + phase
                + " steps=" + planStart + ".." + Math.max(planStart, planEnd - 1)
                + " actions=" + Math.max(0, planEnd - planStart)
                + " subgoal=" + subgoalLabel(subgoal)
                + " synthetic=" + syntheticBoxTarget
                + " verdict=" + verdict
                + " sameState=" + sameState
                + " agentReach=" + before.agentReachable + "->" + after.agentReachable
                + " totalReach=" + before.totalReachable + "->" + after.totalReachable
                + " goalAdj=" + before.goalAdjacentReachable + "->" + after.goalAdjacentReachable
                + (subgoal.isSyntheticRelief() ? " cert=" + reliefCertificateLabel(subgoal) : "")
                + (subgoal.isAgentGoal ? "" : " " + subgoal.boxType + "=" + before.boxPositions + "->" + after.boxPositions);
        if ("ACCEPTED".equals(verdict)) {
            logVerbose(msg);
        } else {
            logNormal(msg);
        }
    }

    private String subgoalLabel(Subgoal subgoal) {
        return subgoal.isAgentGoal
                ? "agent" + subgoal.agentId + "->@" + subgoal.goalPos
                : "agent" + subgoal.agentId + "->" + subgoal.boxType + "@" + subgoal.goalPos;
    }

    private Set<Position> bfsReachableForEval(Position start, State state, Level level) {
        Set<Position> visited = new HashSet<>();
        if (start == null) return visited;
        Queue<Position> queue = new LinkedList<>();
        visited.add(start);
        queue.add(start);
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (state.hasBoxAt(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }

    private int totalReachableCells(State state, Level level) {
        int total = 0;
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            total += bfsReachableForEval(state.getAgentPosition(agentId), state, level).size();
        }
        return total;
    }

    private boolean adjacentReachable(Position target, Set<Position> reachable, Level level) {
        if (target == null) return false;
        if (reachable.contains(target)) return true;
        for (Direction dir : Direction.values()) {
            Position adj = target.move(dir);
            if (!level.isWall(adj) && reachable.contains(adj)) return true;
        }
        return false;
    }

    private List<String> boxPositionsOfType(State state, char boxType) {
        List<String> positions = new ArrayList<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() != boxType) continue;
            Position p = entry.getKey();
            positions.add(p.row + "," + p.col);
        }
        Collections.sort(positions);
        return positions;
    }

    private String stateSignature(State state) {
        List<String> parts = new ArrayList<>();
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            Position p = state.getAgentPosition(agentId);
            if (p != null) parts.add("a" + agentId + "@" + p.row + "," + p.col);
        }
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Position p = entry.getKey();
            parts.add(entry.getValue() + "@" + p.row + "," + p.col);
        }
        Collections.sort(parts);
        return String.join("|", parts);
    }

    private static class SealRisk {
        final Subgoal future;
        final Set<Position> frozen;
        final SealStagingPlan staging;
        final boolean allowStaging;

        SealRisk(Subgoal future, Set<Position> frozen, SealStagingPlan staging,
                 boolean allowStaging) {
            this.future = future;
            this.frozen = frozen;
            this.staging = staging;
            this.allowStaging = allowStaging;
        }
    }

    private static class SealStagingPlan {
        final int agentId;
        final Position target;
        final List<Action> path;

        SealStagingPlan(int agentId, Position target, List<Action> path) {
            this.agentId = agentId;
            this.target = target;
            this.path = path;
        }
    }

    private static class SubgoalEval {
        final String stateSignature;
        final int agentReachable;
        final int totalReachable;
        final boolean goalAdjacentReachable;
        final List<String> boxPositions;

        SubgoalEval(String stateSignature, int agentReachable, int totalReachable,
                    boolean goalAdjacentReachable, List<String> boxPositions) {
            this.stateSignature = stateSignature;
            this.agentReachable = agentReachable;
            this.totalReachable = totalReachable;
            this.goalAdjacentReachable = goalAdjacentReachable;
            this.boxPositions = boxPositions;
        }
    }
    
    /**
     * True if goal pos should be treated as path-critical (filled last, suspended when displaced):
     *   TRANSIT or CHOKEPOINT from GoalTransitAnalyzer (crossingCount ≥ TRANSIT_THRESHOLD).
     * TERMINAL and NEUTRAL goals are NOT classified as transit here — they are handled by
     * disturbance-count-based suspension in REGRESS-ACCEPT (after 2+ disturbances).
     */
    private boolean isTransitGoal(Position p) {
        GoalTransitAnalyzer.GoalProfile prof = transitProfiles.get(p);
        return prof != null
                && prof.profile != GoalTransitAnalyzer.TransitProfile.TERMINAL
                && prof.profile != GoalTransitAnalyzer.TransitProfile.NEUTRAL;
    }

    /** Revalidate completed goals - remove any that were disturbed (e.g. by soft-unlock). */
    private void revalidateCompletedGoals(State state, Level level) {
        boolean anyRemoved = false;
        Iterator<Position> it = completedBoxGoals.iterator();
        while (it.hasNext()) {
            Position goalPos = it.next();
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') {
                it.remove();
                anyRemoved = true;
                continue;
            }
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox == null || actualBox != goalType) {
                if (isTransitGoal(goalPos)) {
                    // TRANSIT goals: suspend instead of remove — prevents re-queue oscillation.
                    // Will be un-suspended when nothing else remains (getOrComputeSubgoalOrder fail-safe).
                    suspendedTransitGoals.add(goalPos);
                    logVerbose("[PP] TRANSIT goal at " + goalPos + " suspended (box displaced)");
                } else {
                    logVerbose("[PP] Goal at " + goalPos + " disturbed (type " + goalType + "), removing from frozen set");
                    it.remove();
                }
                anyRemoved = true;
                // Force subgoal list refresh so this disturbed goal is handled
                cachedSubgoalOrder = null;
            }
        }
        // Invalidate Hungarian cache when goals are removed — the optimal assignment
        // changes as completed goals shrink (different boxes become available).
        if (anyRemoved) {
            subgoalManager.invalidateHungarianCache();
        }
    }
    
    /**
     * Detects which completed box goals were regressed (disturbed) in the given state.
     * 
     * This is a READ-ONLY check — it does not modify completedBoxGoals.
     * Used as a pre-commit validation: if a planned path caused regression,
     * the caller should rollback rather than accept the regression.
     * 
     * Difference from revalidateCompletedGoals: that method is a post-hoc cleanup
     * that accepts regressions and removes them from the frozen set. This method
     * is a pre-commit guard that PREVENTS regressions from being accepted.
     * 
     * @param state The state to check (typically after executing a candidate path)
     * @param level Level definition
     * @return List of goal positions that were completed but are now unsatisfied
     */
    private List<Position> detectRegressedGoals(State state, Level level) {
        List<Position> regressed = new ArrayList<>();
        for (Position goalPos : completedBoxGoals) {
            if (suspendedTransitGoals.contains(goalPos)) continue; // already suspended, expected vacant
            char goalType = level.getBoxGoal(goalPos.row, goalPos.col);
            if (goalType == '\0') continue;
            Character actualBox = state.getBoxes().get(goalPos);
            if (actualBox == null || actualBox != goalType) {
                regressed.add(goalPos);
            }
        }
        return regressed;
    }

    private int countSatisfiedBoxGoals(State state, Level level) {
        int count = 0;
        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            for (Position goalPos : entry.getValue()) {
                Character actual = state.getBoxes().get(goalPos);
                if (actual != null && actual == goalType) {
                    count++;
                }
            }
        }
        return count;
    }

    /** Try displacement-based recovery when stuck with cyclic dependencies. */
    private void tryCycleRecovery(State currentState, Level level,
            List<Action[]> fullPlan, int numAgents) {

        DependencyAnalyzer.AnalysisResult analysis = DependencyAnalyzer.analyze(currentState, level);

        if (analysis.hasCycle && displacementAttempts < MAX_DISPLACEMENT_ATTEMPTS) {
            logVerbose("[PP] Cyclic dependency detected, trying displacement...");
            displacementAttempts++;
            List<Action[]> displacementPlan = new ArrayList<>();
            boolean success = deadlockBreaker.attemptCycleBreaking(
                displacementPlan, currentState, level, numAgents,
                analysis.cycles.isEmpty() ? new ArrayList<>() : analysis.cycles.get(0),
                pathAnalyzer, conflictResolver);

            if (success && !displacementPlan.isEmpty()) {
                logVerbose("[PP] Displacement succeeded with " + displacementPlan.size() + " steps");
                fullPlan.addAll(displacementPlan);
            }
        }
    }

    /** Try recovery mechanisms when stuck. */
    private boolean tryRecovery(List<Subgoal> subgoals, List<Action[]> fullPlan,
            State currentState, Level level, int numAgents, State initialState) {
        
        if (subgoals.isEmpty()) return false;
        Subgoal blockedGoal = subgoals.get(0);
        
        // Strategy 1: Try clearing blocking agents
        Set<Position> criticalPositions = new HashSet<>();
        criticalPositions.add(blockedGoal.goalPos);
        Set<Position> satisfiedGoals = GoalChecker.computeSatisfiedGoalPositions(currentState, level);
        
        if (blockedGoal.isAgentGoal) {
            // Agent goal: find critical positions on agent-to-goal path
            criticalPositions.addAll(pathAnalyzer.findCriticalPositionsForAgentGoal(
                currentState, level, blockedGoal.agentId, blockedGoal.goalPos, satisfiedGoals));
        } else {
            // Box goal: find critical positions on BOTH agent-to-box AND box-to-goal paths
            Position boxPos = subgoalManager.findBestBoxForGoal(blockedGoal, currentState, level, completedBoxGoals);
            if (boxPos != null) {
                criticalPositions.addAll(pathAnalyzer.findCriticalPositions(
                    currentState, level, blockedGoal.agentId, blockedGoal.goalPos, boxPos, satisfiedGoals));
            } else {
                criticalPositions.addAll(pathAnalyzer.findCriticalPositionsForAgentGoal(
                    currentState, level, blockedGoal.agentId, blockedGoal.goalPos, satisfiedGoals));
            }
        }
        
        int planSizeBefore = fullPlan.size();
        AgentCoordinator.ClearingResult result = agentCoordinator.tryIdleAgentClearingWithResult(
            fullPlan, currentState, level, numAgents, blockedGoal.agentId, criticalPositions,
            pathAnalyzer, conflictResolver);
        
        if (result.success) {
            logVerbose("[PP] Cleared blocking agent " + result.clearedAgentId);
            return true;
        }
        
        // Strategy 2: Try box displacement
        // MAPF FIX: Pass explicit subgoals to DeadlockResolver so it knows exactly what the targets are
        // (Is it an Agent Goal? Is it a Box Goal? Which box?)
        // Helps avoid identifying the target box itself as an obstacle.
        List<DeadlockResolver.BlockingInfo> blockingInfos = 
            deadlockResolver.analyzeBlocking(currentState, level, subgoals, immovableBoxes);
        
        if (!blockingInfos.isEmpty()) {

            DeadlockResolver.DisplacementPlan displacement = 
                deadlockResolver.createDisplacementPlan(blockingInfos, currentState, level, displacementHistory);
            
            if (displacement != null) {
                List<Action> displacePath;
                boolean isAgentMove = displacement.isAgentDisplacement;
                
                if (isAgentMove) {
                    displacePath = boxSearchPlanner.searchForAgentGoal(
                        displacement.agentId, displacement.tempPosition, currentState, level);
                } else {
                    displacePath = boxSearchPlanner.planBoxDisplacement(
                        displacement.agentId, displacement.boxPosition,
                        displacement.tempPosition, displacement.boxType, currentState, level);
                }
                
                if (displacePath != null && !displacePath.isEmpty()) {
                    String historyKey = isAgentMove ? 
                        ("Agent" + displacement.agentId + "@" + displacement.boxPosition) : 
                        (displacement.boxType + "@" + displacement.boxPosition);
                        
                    displacementHistory.add(historyKey);
                    
                    State tempState = currentState;
                    for (Action action : displacePath) {
                        Action[] jointAction = planMerger.createJointActionWithMerging(
                            displacement.agentId, action, tempState, level, numAgents, isAgentMove, completedBoxGoals);
                        jointAction = conflictResolver.resolveConflicts(jointAction, tempState, level, displacement.agentId);
                        fullPlan.add(jointAction);
                        tempState = applyJointAction(jointAction, tempState, level, numAgents);
                    }
                    
                    // Displacement is INTENTIONAL disturbance of completed goals.
                    // Track displaced goals so planSubgoal excludes them from frozen
                    // and the regression guard in tryExecuteSubgoals ignores them.
                    if (!completedBoxGoals.isEmpty()) {
                        List<Position> regressed = detectRegressedGoals(tempState, level);
                        if (!regressed.isEmpty()) {
                            displacedGoals.addAll(regressed);
                            logVerbose("[PP] Displacement displaced " + regressed.size()
                                    + " completed goal(s): " + regressed + " (deferred for re-planning)");
                        }
                    }
                    return true;
                }
            }
        }
        
        // Strategy 3: Dependency-safe alternate-order retry.
        //
        // The old recovery path shuffled all subgoals and committed the first locally
        // executable one. That bypassed hard dependencies and the normal acceptance
        // checks, so it could make plausible local progress while sealing future goals.
        // Keep diversification as a last probe, but only among dependency-eligible
        // goals and route execution through tryExecuteSubgoals' standard validation.
        List<Subgoal> retryable = new ArrayList<>();
        for (Subgoal sg : subgoals) {
            if (!areDependenciesMet(sg.goalPos, level)) continue;
            retryable.add(sg);
        }
        if (retryable.size() > 1) {
            Collections.shuffle(retryable, random);
            return tryExecuteSubgoals(retryable, fullPlan, currentState, level, numAgents, initialState);
        }

        return false;
    }

    /** Recompute state from initial + all actions. Cached for incremental performance. */
    private State lastComputedState = null;
    private int lastComputedPlanSize = 0;
    
    private State recomputeState(State initial, List<Action[]> plan, Level level, int numAgents) {
        // Incremental: only replay from where we left off
        if (lastComputedState != null && lastComputedPlanSize <= plan.size()) {
            State state = lastComputedState;
            for (int i = lastComputedPlanSize; i < plan.size(); i++) {
                state = applyJointAction(plan.get(i), state, level, numAgents);
            }
            lastComputedState = state;
            lastComputedPlanSize = plan.size();
            return state;
        }
        
        // Full recompute (first call or if plan was modified)
        State state = initial;
        for (Action[] jointAction : plan) {
            state = applyJointAction(jointAction, state, level, numAgents);
        }
        lastComputedState = state;
        lastComputedPlanSize = plan.size();
        return state;
    }

    /** Apply joint action to state (simultaneous per CLAUDE.md). */
    private State applyJointAction(Action[] jointAction, State state, Level level, int numAgents) {
        Action[] effective = jointAction.clone();
        boolean changed = false;

        for (int agentId = 0; agentId < effective.length; agentId++) {
            Action action = effective[agentId];
            if (action != null && action.type != Action.ActionType.NOOP
                    && !state.isApplicable(action, agentId, level)) {
                effective[agentId] = Action.noOp();
                changed = true;
            }
        }

        List<ConflictDetector.Conflict> conflicts =
                jointActionValidator.detectConflicts(state, effective, level);
        for (ConflictDetector.Conflict conflict : conflicts) {
            if (effective[conflict.agent1].type != Action.ActionType.NOOP) {
                effective[conflict.agent1] = Action.noOp();
                changed = true;
            }
            if (effective[conflict.agent2].type != Action.ActionType.NOOP) {
                effective[conflict.agent2] = Action.noOp();
                changed = true;
            }
        }

        if (changed) {
            System.arraycopy(effective, 0, jointAction, 0, effective.length);
        }
        return state.applyJointAction(effective, level);
    }

    /** Subgoal: move a box or agent to a goal position. */
    public static class Subgoal {
        public final int agentId;
        public final char boxType;
        public final Position goalPos;
        public final boolean isAgentGoal;
        public final ReliefCertificate reliefCertificate;

        public Subgoal(int agentId, char boxType, Position goalPos, boolean isAgentGoal) {
            this(agentId, boxType, goalPos, isAgentGoal, null);
        }

        public Subgoal(int agentId, char boxType, Position goalPos, boolean isAgentGoal,
                       ReliefCertificate reliefCertificate) {
            this.agentId = agentId;
            this.boxType = boxType;
            this.goalPos = goalPos;
            this.isAgentGoal = isAgentGoal;
            this.reliefCertificate = reliefCertificate;
        }

        public Subgoal(int agentId, char boxType, Position goalPos) {
            this(agentId, boxType, goalPos, false);
        }

        public boolean isSyntheticRelief() {
            return reliefCertificate != null;
        }
    }

    public static final class ReliefCertificate {
        public enum Kind {
            AGENT_ACCESS,
            BOX_CORRIDOR
        }

        public final Kind kind;
        public final Position blockerStart;
        public final int blockedAgentId;
        public final Position primary;
        public final Position secondary;

        private ReliefCertificate(Kind kind, Position blockerStart, int blockedAgentId,
                                  Position primary, Position secondary) {
            this.kind = kind;
            this.blockerStart = blockerStart;
            this.blockedAgentId = blockedAgentId;
            this.primary = primary;
            this.secondary = secondary;
        }

        public static ReliefCertificate agentAccess(Position blockerStart, int blockedAgentId,
                                                    Position targetBox, Position alternateTarget) {
            return new ReliefCertificate(Kind.AGENT_ACCESS, blockerStart, blockedAgentId,
                    targetBox, alternateTarget);
        }

        public static ReliefCertificate boxCorridor(Position blockerStart,
                                                    Position box, Position goal) {
            return new ReliefCertificate(Kind.BOX_CORRIDOR, blockerStart, -1, box, goal);
        }
    }
}
