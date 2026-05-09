package mapf.planning;

import mapf.domain.*;
import mapf.planning.analysis.LevelAnalyzer;
import mapf.planning.analysis.LevelAnalyzer.LevelFeatures;
import mapf.planning.analysis.LevelAnalyzer.StrategyType;
import mapf.planning.analysis.ImmovableFusion;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.planning.strategy.PriorityPlanningStrategy;
import mapf.planning.strategy.PriorityPlanningStrategy.OrderingMode;
import mapf.planning.strategy.SingleAgentStrategy;

import java.util.*;

/**
 * Portfolio-based search controller that selects and executes strategies
 * based on level analysis. Implements fallback mechanism for robustness.
 * 
 * Per ARCHITECTURE.md: "Start with Option A (independent planning), 
 * upgrade to Option B or hybrid approach if competition levels require it."
 */
public class PortfolioController implements SearchStrategy {
    
    private final SearchConfig config;
    private long timeoutMs;
    private LevelFeatures features;
    private Level lastLevel;  // captured at search() entry for use by diagnostics (printAttemptSummary)
    
    // Cached heuristic - expensive to compute, reuse across strategies
    private Heuristic cachedHeuristic;
    
    // Cached SubgoalManager - shares ImmovableBoxDetector distance cache across PP retries
    private mapf.planning.strategy.SubgoalManager cachedSubgoalManager;
    
    // Track attempts for debugging
    private final List<AttemptRecord> attempts = new ArrayList<>();
    
    public PortfolioController(SearchConfig config) {
        this.config = config;
        this.timeoutMs = config.getTimeoutMs();
    }
    
    @Override
    public String getName() {
        return "Portfolio Controller";
    }
    
    @Override
    public void setTimeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    public void setMaxStates(int maxStates) {
        // Delegated to individual strategies
    }
    
    @Override
    public List<Action[]> search(State initialState, Level level) {
        long startTime = System.currentTimeMillis();
        long remainingTime = timeoutMs;
        attempts.clear();
        this.lastLevel = level;

        // Step 1: Pre-analyze level
        features = LevelAnalyzer.analyze(level, initialState);
        if (SearchConfig.isNormal()) {
            System.err.println(features.analysisReport);
        }

        // Step 1a: Immovable-box → wall fusion.
        // Boxes that no agent can ever push (wrong color / unreachable) are converted
        // into static walls and removed from the State. This shrinks the search space
        // for every downstream BFS / A* / heuristic precomputation.
        //
        // Relationship to ImmovableBoxDetector (used by SubgoalManager / Hungarian):
        //   - TaskFilter.immovableBoxes: color + reachability based; INPUT to fusion.
        //   - ImmovableBoxDetector.getImmovableBoxes(): color-only; computed lazily on
        //     the FUSED level/state where those boxes are already walls, so it returns
        //     the empty set (or only edge cases like wrong-letter-on-goal). The two are
        //     complementary, not duplicate; documented here to prevent future confusion.
        if (features != null && features.taskFilter != null
                && !features.taskFilter.immovableBoxes.isEmpty()) {
            ImmovableFusion.FusedProblem fused = ImmovableFusion.fuse(
                    level, initialState, features.taskFilter.immovableBoxes);
            if (fused.changed) {
                if (SearchConfig.isMinimal()) {
                    System.err.println("[Portfolio] Immovable-box fusion: " +
                            fused.fusedPositions.size() + " boxes → walls (of " +
                            features.taskFilter.immovableBoxes.size() + " immovable detected)");
                }
                level = fused.level;
                initialState = fused.state;
                // Re-analyze after fusion so downstream features (TaskFilter,
                // dependency analysis, coupling, recommended strategy) reflect
                // the smaller post-fusion problem.
                features = LevelAnalyzer.analyze(level, initialState);
                // Sanity: post-fusion immovable should be empty (or only non-fusible
                // edge cases). Logged once if violated for diagnostic purposes.
                if (SearchConfig.isNormal()
                        && features.taskFilter != null
                        && !features.taskFilter.immovableBoxes.isEmpty()) {
                    System.err.println("[Portfolio] Note: " +
                            features.taskFilter.immovableBoxes.size() +
                            " immovable boxes remain after fusion (likely wrong-letter-on-goal).");
                }
            }
        }

        // Step 1b: Independence detection — solve independent groups separately
        // This is a shortcut: if all groups solve, return immediately.
        // If any group fails (partial), fall through to normal portfolio as fallback.
        List<Action[]> bestPartialPlan = null;
        
        if (initialState.getNumAgents() > 1) {
            List<List<Integer>> independentGroups = detectIndependentGroups(initialState, level);
            // Plan-P1: refine physical-reachability groups by footprint disjointness.
            // Two agents in the same physical component but with disjoint footprints
            // (their boxes/goals/required corridors don't overlap) are actually
            // independent and can be solved as separate subproblems.
            List<List<Integer>> refinedGroups = refineGroupsByFootprint(
                    independentGroups, initialState, level);
            if (refinedGroups.size() > independentGroups.size() && SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] Footprint refinement: "
                        + independentGroups.size() + " -> " + refinedGroups.size() + " groups");
            }
            independentGroups = refinedGroups;

            // P3: cross-group goal-dependency merge.
            // refineGroupsByFootprint considers PHYSICAL coupling (shared cells/boxes/goals)
            // but ignores LOGICAL coupling encoded in features.goalDependsOn. If goal A
            // (serviced by group X) depends on goal B (serviced by group Y), then solving
            // X in isolation necessarily produces a partial plan because B is invisible
            // after projection. Merging X and Y eliminates the wasted attempt + fallback.
            // Per qanda.txt 2.2: dependency analysis identifies "must-do-before" edges;
            // those edges constrain which groups can truly act independently.
            if (features != null && features.goalDependsOn != null
                    && !features.goalDependsOn.isEmpty() && independentGroups.size() > 1) {
                List<List<Integer>> mergedGroups = mergeGroupsByGoalDependencies(
                        independentGroups, level, features.goalDependsOn);
                // Only ACCEPT the merge if it leaves at least 2 groups. A collapse to 1
                // means "everyone is logically coupled" — but in that case the existing
                // per-group attempts (each runs a short PP) provide cache warm-up and
                // fast feedback that the cold full-portfolio fallback cannot match.
                // Empirical: ClosedAI 5→1 collapse REGRESSED (60s timeout); 5→2 BigSplit
                // is the sweet spot. Keep merge effective when it preserves parallelism.
                if (mergedGroups.size() < independentGroups.size() && mergedGroups.size() >= 2) {
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] P3 cross-group dependency merge: "
                                + independentGroups.size() + " -> " + mergedGroups.size() + " groups");
                    }
                    independentGroups = mergedGroups;
                } else if (mergedGroups.size() == 1 && independentGroups.size() > 1
                        && SearchConfig.isMinimal()) {
                    System.err.println("[Portfolio] P3: dependency analysis suggests full coupling ("
                            + independentGroups.size() + " -> 1); keeping original "
                            + independentGroups.size() + " groups for warm-up + fallback path");
                }
            }

            // D13: NAMO-coupling merge.
            // Goal-dependency merge (P3) only sees the goalDependsOn graph. NAMO
            // coupling — agent A is geometrically blocked from its same-color box by
            // other-color boxes that only group Y can move — produces no edge in
            // goalDependsOn but still makes A's group dependent on Y. Merge those.
            // Conservative: only accept merges that leave ≥2 groups (same rule as P3).
            if (independentGroups.size() > 1) {
                List<List<Integer>> namoMerged = mergeGroupsByNAMOCoupling(
                        independentGroups, initialState, level);
                if (namoMerged.size() < independentGroups.size() && namoMerged.size() >= 2) {
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] D13 NAMO-coupling merge: "
                                + independentGroups.size() + " -> " + namoMerged.size() + " groups");
                    }
                    independentGroups = namoMerged;
                }
            }

            if (independentGroups.size() > 1) {
                if (SearchConfig.isMinimal()) {
                    System.err.println("[Portfolio] Detected " + independentGroups.size() + 
                        " independent agent groups: " + independentGroups);
                }
                List<Action[]> mergedPlan = solveIndependentGroups(independentGroups, initialState, level, startTime);
                if (mergedPlan != null && !mergedPlan.isEmpty()) {
                    State finalState = replayPlan(mergedPlan, initialState, level);
                    if (finalState != null && finalState.isGoalState(level)) {
                        if (SearchConfig.isMinimal()) {
                            System.err.println("[Portfolio] Independence detection: SOLVED (" + mergedPlan.size() + " steps)");
                        }
                        return mergedPlan;
                    }
                    // Partial — save as fallback, continue with normal portfolio
                    bestPartialPlan = mergedPlan;
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] Independence detection: partial (" + mergedPlan.size() + 
                            " steps) — falling back to normal portfolio");
                    }
                }
            }
        }
        
        // Hint GC before full portfolio — independence detection may have accumulated
        // significant garbage from projected levels/states/heuristics.
        System.gc();
        
        // Step 2: Build strategy sequence based on analysis
        List<StrategyConfig> strategies = buildStrategySequence(features, initialState);
        
        if (SearchConfig.isMinimal()) {
            System.err.println("[Portfolio] Strategy sequence: " + 
                strategies.stream().map(s -> s.type.name() + 
                    (s.orderingMode != null ? "(" + s.orderingMode + 
                        (s.orderingMode == OrderingMode.RANDOM ? "#" + s.randomSeed : "") + ")" : "")).toList());
        }
        
        // Step 3: Try strategies in sequence, keeping the best result
        
        // Minimum useful attempt budget. If remaining wall-clock falls below
        // this AND we already have a partial plan, returning early is strictly
        // better than starting another attempt that will be killed mid-search
        // (causing the cached bestPartialPlan to be lost).
        final long MIN_ATTEMPT_MS = 5_000;

        // P0b: accumulate goal positions that previous PP attempts got stuck on.
        // Each subsequent PP attempt will demote these to the end of its order \u2014
        // giving other goals a chance to physically clear blockers first.
        Set<Position> deprioritizedGoals = new HashSet<>();

        // CBSR: collect FailureReports from all PP attempts for nogood derivation.
        List<mapf.planning.signal.FailureReport> allPPFailures = new ArrayList<>();

        // P4 (conflict-driven PP): accumulate blocker goals derived from prior
        // FailureReports. A blocker is an unsatisfied goal G' such that the failed
        // subgoal G has G in goalDependsOn[G'] (i.e. G' must come before G).
        // PP promotes these to the front of the next attempt's order.
        // Per claudeopus47.txt 3.2.2 (subgoal-level reorder is the cheapest CBS-lite
        // repair) and qanda.txt 5.2 (failure-signal-driven repair).
        Set<Position> prioritizedGoals = new HashSet<>();

        // F2 (NAMO co-suspension): accumulate box-goal positions to suspend while
        // a relief subgoal is moving the corresponding blocker to a P_temp.
        // Without this, after the relief succeeds the planner re-pushes the box
        // back to its original goal, undoing the relief. PP applies these per attempt.
        Set<Position> suspendedBoxGoals = new HashSet<>();

        // P1: synthetic escape subgoals to break 2-cycles. Computed lazily once,
        // when the first PP attempt has failed on a level with hasCircularDependency.
        // Per qanda.txt 3 / claudeopus47.txt 3.2.2.
        List<mapf.planning.strategy.PriorityPlanningStrategy.Subgoal> escapeSubgoals = null;

        // Diagnostic snapshot inputs: capture the most recent PP instance + report so
        // that on portfolio failure we can dump a structured snapshot to disk.
        // (Per problem-0509.txt §2.3-2.4: pre-aggregated diagnostics beat raw logs.)
        PriorityPlanningStrategy lastPP = null;
        mapf.planning.signal.FailureReport lastReportForSnapshot = null;
        // Snapshot: also remember the FailureReport that was current when the BEST
        // partial plan was assigned, so we can render a local view at that blocker
        // (the best partial often fails at a different goal than the last attempt).
        mapf.planning.signal.FailureReport bestPartialReport = null;

        for (StrategyConfig strategyConfig : strategies) {
            if (remainingTime <= 0) {
                System.err.println("[Portfolio] Timeout - no more time for attempts");
                break;
            }
            // Plan-A guard: if we already saved a partial and there's not enough
            // time left for a meaningful new attempt, ship the partial NOW rather
            // than burning the rest of the budget on a strategy that will never
            // get to return.
            if (bestPartialPlan != null && remainingTime < MIN_ATTEMPT_MS) {
                System.err.println("[Portfolio] Remaining time " + remainingTime
                    + "ms < " + MIN_ATTEMPT_MS + "ms — shipping best partial ("
                    + bestPartialPlan.size() + " steps) instead of new attempt");
                break;
            }
            
            // Allocate time for this attempt
            long attemptTimeout = computeAttemptTimeout(strategyConfig, remainingTime, strategies.size());
            
            if (SearchConfig.isMinimal()) {
                String modeStr = "";
                if (strategyConfig.orderingMode != null) {
                    modeStr = "(" + strategyConfig.orderingMode;
                    if (strategyConfig.orderingMode == OrderingMode.RANDOM) modeStr += "#" + strategyConfig.randomSeed;
                    modeStr += ")";
                }
                System.err.println("[Portfolio] Trying " + strategyConfig.type + modeStr +
                    " (timeout=" + attemptTimeout + "ms, budget=" + 
                    String.format("%.0f%%", strategyConfig.timeBudgetFraction * 100) + ")");
            }
            
            // Create and configure strategy
            SearchStrategy strategy = createStrategy(strategyConfig, level);
            strategy.setTimeout(attemptTimeout);

            // P0b: pass accumulated deprioritized goals from prior failed attempts.
            if (!deprioritizedGoals.isEmpty() && strategy instanceof PriorityPlanningStrategy) {
                ((PriorityPlanningStrategy) strategy).setDeprioritizedGoals(deprioritizedGoals);
            }

            // P4: pass accumulated blocker goals (promote to front).
            // F3: SKIP promotion in RANDOM mode — promoting 25 goals overrides the
            // random shuffle, defeating the purpose of multi-seed diversification.
            if (!prioritizedGoals.isEmpty() && strategy instanceof PriorityPlanningStrategy
                    && strategyConfig.orderingMode != OrderingMode.RANDOM) {
                ((PriorityPlanningStrategy) strategy).setPrioritizedGoals(prioritizedGoals);
            }

            // P1: pass escape subgoals (synthesized lazily on first failure for cyclic levels).
            if (escapeSubgoals != null && !escapeSubgoals.isEmpty()
                    && strategy instanceof PriorityPlanningStrategy) {
                ((PriorityPlanningStrategy) strategy).setEscapeSubgoals(escapeSubgoals);
            }

            // F2: pass suspended box-goals (NAMO co-suspension).
            if (!suspendedBoxGoals.isEmpty() && strategy instanceof PriorityPlanningStrategy) {
                ((PriorityPlanningStrategy) strategy).setSuspendedBoxGoals(suspendedBoxGoals);
            }
            
            // Execute
            long attemptStart = System.currentTimeMillis();
            List<Action[]> result = null;
            try {
                result = strategy.search(initialState, level);
            } catch (Exception e) {
                System.err.println("[Portfolio] Strategy " + strategyConfig.type + " threw exception: " + e.getMessage());
                e.printStackTrace(System.err);
            }
            long attemptDuration = System.currentTimeMillis() - attemptStart;

            // P0a: log structured failure signal when present (PP nulls it on success).
            // P0b: also accumulate the failed subgoal's goal position into deprioritizedGoals
            // so the next PP attempt demotes it to the end.
            // SUMMARY: capture per-attempt diagnostics for the final structured table.
            String summaryFailedSubgoal = null;
            String summaryFailureKind = "NONE";
            int summaryUnsat = -1;
            int summaryReliefs = 0;
            if (strategy instanceof PriorityPlanningStrategy) {
                lastPP = (PriorityPlanningStrategy) strategy;
                mapf.planning.signal.FailureReport report =
                        ((PriorityPlanningStrategy) strategy).getLastFailureReport();
                if (report != null) lastReportForSnapshot = report;
                if (report != null) allPPFailures.add(report);
                if (report != null) {
                    // SUMMARY: extract from report
                    summaryUnsat = report.unsatisfiedAtFailure != null
                            ? report.unsatisfiedAtFailure.size() : -1;
                    summaryFailureKind = report.kind != null ? report.kind.name() : "PARTIAL_PLAN";
                    if (report.lastAttemptedSubgoal != null) {
                        mapf.planning.strategy.PriorityPlanningStrategy.Subgoal sg =
                                report.lastAttemptedSubgoal;
                        summaryFailedSubgoal = "agent" + sg.agentId
                                + (sg.isAgentGoal ? "->@" : "->" + sg.boxType + "@")
                                + (sg.goalPos != null ? sg.goalPos.toString() : "?");
                    }
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] FailureReport from " + strategyConfig.type
                                + ": " + report.summary());
                    }
                    if (report.lastAttemptedSubgoal != null && report.lastAttemptedSubgoal.goalPos != null) {
                        Position failedGoal = report.lastAttemptedSubgoal.goalPos;
                        if (deprioritizedGoals.add(failedGoal)
                                && SearchConfig.isMinimal()) {
                            System.err.println("[Portfolio] P0b: deprioritize goal "
                                    + failedGoal
                                    + " for next attempt (now " + deprioritizedGoals.size() + " total)");
                        }

                        // P4 (conflict-driven PP): identify the BLOCKERS of the failed
                        // subgoal and promote them to the front of the next attempt.
                        // goalDependsOn[src] = {dst : src must come BEFORE dst}.
                        // Inverse: blockers of failedGoal = {src : failedGoal in goalDependsOn[src]
                        //                                          AND src is still unsatisfied}.
                        // This is the CBS-lite ordering branch complementary to deprioritization.
                        if (features != null && features.goalDependsOn != null
                                && !report.unsatisfiedAtFailure.isEmpty()) {
                            Set<Position> stillUnsatisfied = new HashSet<>();
                            for (mapf.planning.strategy.PriorityPlanningStrategy.Subgoal sg
                                    : report.unsatisfiedAtFailure) {
                                if (sg.goalPos != null) stillUnsatisfied.add(sg.goalPos);
                            }
                            int promotedThisRound = 0;
                            for (Map.Entry<Position, Set<Position>> e
                                    : features.goalDependsOn.entrySet()) {
                                Position src = e.getKey();
                                if (src.equals(failedGoal)) continue;
                                if (!stillUnsatisfied.contains(src)) continue;
                                if (e.getValue() != null && e.getValue().contains(failedGoal)) {
                                    if (prioritizedGoals.add(src)) promotedThisRound++;
                                }
                            }
                            if (promotedThisRound > 0 && SearchConfig.isMinimal()) {
                                System.err.println("[Portfolio] P4: promote " + promotedThisRound
                                        + " blocker goal(s) of " + failedGoal
                                        + " for next attempt (now " + prioritizedGoals.size() + " total)");
                            }
                        }
                    }

                    // P1: synthesize escape subgoals once, after the first PP failure on a
                    // level with hasCircularDependency=true. Subsequent attempts reuse them.
                    if (escapeSubgoals == null && features != null && features.hasCircularDependency
                            && features.goalDependsOn != null) {
                        Set<Position> immovable = (features.taskFilter != null)
                                ? features.taskFilter.immovableBoxes : Collections.emptySet();
                        escapeSubgoals = mapf.planning.synthesis.EscapeSubgoalSynthesizer.synthesize(
                                initialState, level, features.goalDependsOn, immovable);
                        if (SearchConfig.isMinimal()) {
                            System.err.println("[Portfolio] P1: synthesized "
                                    + escapeSubgoals.size() + " escape subgoal(s) for cyclic level");
                        }
                    }

                    // P5 (NAMO / blocker-relief): synthesize relief subgoals when an agent
                    // cannot geometrically reach its target box because other-color boxes
                    // form a barrier. Reified as ordinary push subgoals for a helper agent
                    // of the blocker's color → consumed by the existing PP/BSP loop.
                    // F1: use the ACTUAL post-attempt state (replayed from partial plan)
                    // instead of initialState. Without this, recursive NAMO is dormant
                    // because the synthesizer keeps seeing the same untouched world and
                    // re-emits the same already-completed reliefs.
                    if (features != null) {
                        Set<Position> immovable = (features.taskFilter != null)
                                ? features.taskFilter.immovableBoxes : Collections.emptySet();
                        State stateForNAMO = initialState;
                        if (result != null && !result.isEmpty()) {
                            State replayed = replayPlan(result, initialState, level);
                            if (replayed != null) stateForNAMO = replayed;
                        }
                        mapf.planning.synthesis.BlockerReliefSynthesizer.ReliefResult relRes =
                                mapf.planning.synthesis.BlockerReliefSynthesizer.synthesizeWithMeta(
                                        stateForNAMO, level, immovable);
                        List<mapf.planning.strategy.PriorityPlanningStrategy.Subgoal> reliefs = relRes.reliefs;
                        if (!reliefs.isEmpty()) {
                            // F6: REPLACE escapeSubgoals with fresh reliefs (don't merge with
                            // stale ones from prior rounds). Stale reliefs target boxes that
                            // may already be moved \u2014 re-trying them creates oscillation.
                            // Keep only true cycle-escape subgoals (those originally from
                            // EscapeSubgoalSynthesizer, not blocker-relief).
                            // Simple impl: dedupe and keep the union but cap by goalPos
                            // preferring fresher (relief) over stale.
                            List<mapf.planning.strategy.PriorityPlanningStrategy.Subgoal> merged =
                                    new ArrayList<>(reliefs);
                            Set<Position> seen = new HashSet<>();
                            for (mapf.planning.strategy.PriorityPlanningStrategy.Subgoal sg : reliefs) {
                                seen.add(sg.goalPos);
                            }
                            // Preserve ONLY cycle-escape subgoals (not relief). Heuristic:
                            // EscapeSubgoalSynthesizer subgoals come from synthesize() above and
                            // were stored before relief synthesis. We can't easily distinguish
                            // here, so just keep a small subset (the first ones come from cycle
                            // escape if features.hasCircularDependency was set).
                            // For ISO (cyclic, but synthesized 0 escapes), this is a no-op.
                            escapeSubgoals = merged;

                            // F2: REPLACE suspended set instead of accumulating \u2014 each round
                            // re-computes suspensions based on current state.
                            if (relRes.suspendedBoxGoals != null) {
                                suspendedBoxGoals.clear();
                                suspendedBoxGoals.addAll(relRes.suspendedBoxGoals);
                            }
                            if (SearchConfig.isMinimal()) {
                                System.err.println("[Portfolio] P5 (NAMO): synthesized "
                                        + reliefs.size() + " blocker-relief subgoal(s)"
                                        + " (F2: " + suspendedBoxGoals.size() + " suspended)");
                            }
                            summaryReliefs = reliefs.size();
                        }
                    }
                }
            }

            // Record attempt
            int planSteps = (result != null) ? result.size() : 0;
            // SUMMARY: success = full goal-state reached. Partial plans (PP returns
            // last reachable state via fallback) are NOT successes for the table.
            boolean isFullSuccess = false;
            if (result != null && !result.isEmpty()) {
                State checkFinal = replayPlan(result, initialState, level);
                isFullSuccess = (checkFinal != null && checkFinal.isGoalState(level));
            }
            if (isFullSuccess) summaryFailureKind = "SUCCESS";
            attempts.add(new AttemptRecord(strategyConfig.type, strategyConfig.orderingMode,
                    strategyConfig.randomSeed, attemptDuration,
                    isFullSuccess, planSteps,
                    summaryUnsat, summaryFailedSubgoal, summaryFailureKind,
                    summaryReliefs, suspendedBoxGoals.size()));
            
            if (result != null && !result.isEmpty()) {
                // Verify this is a full solution (goal state reached)
                // by checking the plan length vs. a trivially detected partial plan.
                // Full solutions come from PP's isGoalState() check path.
                // Partial plans come from PP's fallback path and are logged as [PARTIAL].
                // Keep best partial as fallback, but only return immediately for full solutions.
                State finalState = replayPlan(result, initialState, level);
                if (finalState != null && finalState.isGoalState(level)) {
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] SUCCESS with " + strategyConfig.type + 
                            " (" + result.size() + " actions, " + attemptDuration + "ms)");
                    }
                    return result;
                }
                // Partial plan — save if best so far, continue trying
                if (bestPartialPlan == null || result.size() > bestPartialPlan.size()) {
                    bestPartialPlan = result;
                    bestPartialReport = lastReportForSnapshot;
                    if (SearchConfig.isMinimal()) {
                        System.err.println("[Portfolio] Partial plan from " + strategyConfig.type + 
                            " (" + result.size() + " steps, saved as best so far)");
                    }
                }
            }
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] " + strategyConfig.type + " " +
                    (result != null && !result.isEmpty() ? "partial" : "failed") + " after " + attemptDuration + "ms");
            }
            
            // Update remaining time
            remainingTime = timeoutMs - (System.currentTimeMillis() - startTime);
        }

        // ===== CBSR: Conflict-Based Subgoal Reordering =====
        // After warm-start attempts (TOPOLOGICAL, DISTANCE_GREEDY), use learned ordering
        // constraints (nogoods) to repair the goal ordering and re-run PP iteratively.
        // When nogood-directed ordering stabilizes, falls back to random-seeded diversification.
        List<OrderingNogood> cbsrNogoods = new ArrayList<>(); // promoted: used in pre-F7 attempt
        List<Position> finalCbsrOrder = (features != null && features.executionOrder != null)
                ? new ArrayList<>(features.executionOrder) : new ArrayList<>();
        // Per claudeopus47.txt §3.2.2 / qanda.txt §5.2 (failure signals drive ordering repair).
        {
            long cbsrElapsed = System.currentTimeMillis() - startTime;
            long cbsrBudget = timeoutMs - cbsrElapsed;

            boolean cbsrApplicable = cbsrBudget > 5_000
                    && features != null
                    && features.executionOrder != null
                    && !features.executionOrder.isEmpty();

            if (cbsrApplicable) {
                // Derive initial nogoods from ALL PP failures collected in the for-loop above.
                // (cbsrNogoods declared at outer scope for post-CBSR use)
                for (mapf.planning.signal.FailureReport fr : allPPFailures) {
                    cbsrNogoods.addAll(learnNogoodsFromReport(fr));
                }
                cbsrNogoods = deduplicateNogoods(cbsrNogoods);

                System.err.println("[CBSR] Starting loop: baseOrder=" + features.executionOrder.size()
                        + " goals, initialNogoods=" + cbsrNogoods.size()
                        + ", budget=" + cbsrBudget + "ms");

                final int MAX_CBSR_ITER = 8;
                // RANDOM fallback seeds used when directed ordering stabilizes
                final int[] CBSR_RANDOM_SEEDS = {42, 137, 9999, 12345};
                int randomSeedIdx = 0;

                List<Position> cbsrOrder = new ArrayList<>(features.executionOrder);
                boolean orderingStable = false;

                for (int cbsrIter = 1; cbsrIter <= MAX_CBSR_ITER; cbsrIter++) {
                    long cbsrRemaining = timeoutMs - (System.currentTimeMillis() - startTime);
                    if (cbsrRemaining < 3_000) {
                        System.err.println("[CBSR] Stopping: remaining=" + cbsrRemaining + "ms < 3s");
                        break;
                    }

                    // Adaptive per-iteration budget: balance remaining time across up to MAX_CBSR_ITER iters
                    long perIterBudget = Math.min(cbsrRemaining / Math.max(1, MAX_CBSR_ITER - cbsrIter + 1), 40_000L);

                    // Create PP for this iteration
                    StrategyConfig cbsrCfg = new StrategyConfig(
                            StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.0);
                    SearchStrategy cbsrStrategy = createStrategy(cbsrCfg, level);
                    if (!(cbsrStrategy instanceof PriorityPlanningStrategy)) break;
                    PriorityPlanningStrategy cbsrPP = (PriorityPlanningStrategy) cbsrStrategy;

                    if (!orderingStable) {
                        // Phase 1: nogood-directed ordering repair
                        List<Position> repairedOrder = applyNogoodsToOrdering(cbsrOrder, cbsrNogoods);
                        if (repairedOrder.equals(cbsrOrder) && cbsrIter > 1) {
                            // Ordering has converged — switch to random diversification
                            orderingStable = true;
                            System.err.println("[CBSR] Ordering converged after " + (cbsrIter - 1)
                                    + " directed iters — switching to random diversification");
                        } else {
                            cbsrOrder = repairedOrder;
                        }
                    }

                    if (orderingStable) {
                        // Phase 2: random-seeded fallback (the original RANDOM#42/137 role)
                        if (randomSeedIdx >= CBSR_RANDOM_SEEDS.length) {
                            System.err.println("[CBSR] All random seeds exhausted — stopping");
                            break;
                        }
                        int seed = CBSR_RANDOM_SEEDS[randomSeedIdx++];
                        System.err.println("[CBSR] Iteration #" + cbsrIter
                                + " [RANDOM#" + seed + "], budget=" + perIterBudget + "ms");
                        cbsrPP.setOrderingMode(OrderingMode.RANDOM);
                        cbsrPP.setRandomSeed(seed);
                    } else {
                        System.err.println("[CBSR] Iteration #" + cbsrIter
                                + ", nogoods=" + cbsrNogoods.size()
                                + ", ordering(first5)=" + cbsrOrder.subList(0, Math.min(5, cbsrOrder.size()))
                                + ", budget=" + perIterBudget + "ms");
                        cbsrPP.setGoalExecutionOrder(cbsrOrder);
                    }

                    cbsrPP.setTimeout(perIterBudget);

                    // Carry forward accumulated hints from warm-start attempts
                    if (!deprioritizedGoals.isEmpty()) cbsrPP.setDeprioritizedGoals(deprioritizedGoals);
                    if (!prioritizedGoals.isEmpty()) cbsrPP.setPrioritizedGoals(prioritizedGoals);
                    if (escapeSubgoals != null && !escapeSubgoals.isEmpty()) cbsrPP.setEscapeSubgoals(escapeSubgoals);
                    if (!suspendedBoxGoals.isEmpty()) cbsrPP.setSuspendedBoxGoals(suspendedBoxGoals);

                    long iterStart = System.currentTimeMillis();
                    List<Action[]> cbsrResult = null;
                    try {
                        cbsrResult = cbsrStrategy.search(initialState, level);
                    } catch (Exception e) {
                        System.err.println("[CBSR] Iteration #" + cbsrIter + " exception: " + e.getMessage());
                    }
                    long iterDuration = System.currentTimeMillis() - iterStart;

                    // Extract failure report from this CBSR iteration
                    mapf.planning.signal.FailureReport cbsrReport = cbsrPP.getLastFailureReport();
                    lastPP = cbsrPP;
                    if (cbsrReport != null) lastReportForSnapshot = cbsrReport;

                    // Compute tracking data for attempt table
                    int cbsrUnsat = (cbsrReport != null && cbsrReport.unsatisfiedAtFailure != null)
                            ? cbsrReport.unsatisfiedAtFailure.size() : -1;
                    String cbsrKind = "NONE";
                    String cbsrFailedSg = null;
                    if (cbsrReport != null) {
                        cbsrKind = cbsrReport.kind != null ? cbsrReport.kind.name() : "PARTIAL_PLAN";
                        if (cbsrReport.lastAttemptedSubgoal != null) {
                            PriorityPlanningStrategy.Subgoal sg = cbsrReport.lastAttemptedSubgoal;
                            cbsrFailedSg = "agent" + sg.agentId
                                    + (sg.isAgentGoal ? "->@" : "->" + sg.boxType + "@")
                                    + (sg.goalPos != null ? sg.goalPos : "?");
                        }
                    }
                    boolean cbsrFull = false;
                    if (cbsrResult != null && !cbsrResult.isEmpty()) {
                        State cbsrFinal = replayPlan(cbsrResult, initialState, level);
                        cbsrFull = (cbsrFinal != null && cbsrFinal.isGoalState(level));
                    }
                    if (cbsrFull) cbsrKind = "SUCCESS";
                    attempts.add(new AttemptRecord(StrategyType.STRICT_ORDER, OrderingMode.TOPOLOGICAL,
                            -(cbsrIter), iterDuration, cbsrFull,
                            (cbsrResult != null ? cbsrResult.size() : 0),
                            cbsrUnsat, cbsrFailedSg, "[CBSR#" + cbsrIter + "]" + cbsrKind,
                            0, suspendedBoxGoals.size()));

                    if (cbsrFull) {
                        System.err.println("[CBSR] Total iterations=" + cbsrIter
                                + ", totalNogoods=" + cbsrNogoods.size() + ", result=SUCCESS");
                        printAttemptSummary();
                        return cbsrResult;
                    }

                    // Update best partial
                    if (cbsrResult != null && !cbsrResult.isEmpty()) {
                        if (bestPartialPlan == null || cbsrResult.size() > bestPartialPlan.size()) {
                            bestPartialPlan = cbsrResult;
                            bestPartialReport = cbsrReport;
                        }
                    }

                    // P0b/P4: update deprioritized/prioritized goals from CBSR failure
                    if (cbsrReport != null && cbsrReport.lastAttemptedSubgoal != null
                            && cbsrReport.lastAttemptedSubgoal.goalPos != null) {
                        deprioritizedGoals.add(cbsrReport.lastAttemptedSubgoal.goalPos);
                        if (features != null && features.goalDependsOn != null
                                && !cbsrReport.unsatisfiedAtFailure.isEmpty()) {
                            Position failedGoal = cbsrReport.lastAttemptedSubgoal.goalPos;
                            Set<Position> stillUnsatisfied = new HashSet<>();
                            for (PriorityPlanningStrategy.Subgoal sg : cbsrReport.unsatisfiedAtFailure) {
                                if (sg.goalPos != null) stillUnsatisfied.add(sg.goalPos);
                            }
                            for (Map.Entry<Position, Set<Position>> e : features.goalDependsOn.entrySet()) {
                                Position src = e.getKey();
                                if (!stillUnsatisfied.contains(src)) continue;
                                if (e.getValue() != null && e.getValue().contains(failedGoal)) {
                                    prioritizedGoals.add(src);
                                }
                            }
                        }
                    }

                    // Learn new nogoods from this iteration's failure (directed phase only)
                    if (!orderingStable && cbsrReport != null) {
                        List<OrderingNogood> newNogoods = learnNogoodsFromReport(cbsrReport);
                        if (!newNogoods.isEmpty()) {
                            cbsrNogoods.addAll(newNogoods);
                            cbsrNogoods = deduplicateNogoods(cbsrNogoods);
                        }
                    }

                    System.err.println("[CBSR] Iteration #" + cbsrIter + " "
                            + (cbsrResult != null && !cbsrResult.isEmpty() ? "partial(" + (cbsrResult != null ? cbsrResult.size() : 0) + ")" : "failed")
                            + " (" + iterDuration + "ms), nogoods=" + cbsrNogoods.size());
                } // end CBSR iteration loop

                finalCbsrOrder = new ArrayList<>(cbsrOrder); // save for pre-F7 use
                System.err.println("[CBSR] Loop done, nogoods=" + cbsrNogoods.size()
                        + ", result=TIMEOUT/UNSAT");
            } // end if cbsrApplicable
        } // end CBSR block
        // ===== END CBSR =====

        // Save the CBSR result BEFORE F7-PRE may modify bestPartialPlan.
        // F7B will use this as its warm-start base (original CBSR state, not F7-PRE's potentially harder state).
        final List<Action[]> originalCbsrBestPlan = bestPartialPlan;

        // ===== F7-PRE: Unsatisfied-First Fresh Attempt =====
        // After CBSR, compute which box goals are still unsatisfied in the CBSR partial plan's
        // final state. Build an ordering that puts those goals FIRST (preserving relative order
        // from the CBSR-repaired ordering), then appends already-satisfied goals.
        // This avoids K/L/G/E boxes being displaced to bad positions during other goal planning.
        // The CBSR nogood (I after L) is naturally preserved because L goals appear in the
        // unsatisfied set and thus come before I in the new ordering.
        if (bestPartialPlan != null && features != null && !finalCbsrOrder.isEmpty()) {
            long preF7Elapsed = System.currentTimeMillis() - startTime;
            long preF7Remaining = timeoutMs - preF7Elapsed;
            if (preF7Remaining > 15_000) {
                State cbsrFinalState = replayPlan(bestPartialPlan, initialState, level);
                if (cbsrFinalState != null && !cbsrFinalState.isGoalState(level)) {
                    // Partition finalCbsrOrder into unsatisfied vs satisfied
                    List<Position> unsatFirst = new ArrayList<>();
                    List<Position> satLast = new ArrayList<>();
                    for (Position p : finalCbsrOrder) {
                        char goalType = level.getBoxGoal(p.row, p.col);
                        if (goalType != '\0') {
                            if (cbsrFinalState.getBoxAt(p) == goalType) satLast.add(p);
                            else unsatFirst.add(p);
                        }
                    }
                    if (!unsatFirst.isEmpty()) {
                        List<Position> preF7Order = new ArrayList<>(unsatFirst);
                        preF7Order.addAll(satLast);
                        System.err.println("[Portfolio] F7-PRE: unsatisfied-first ordering ("
                                + unsatFirst.size() + " unsat, " + satLast.size() + " sat)");

                        // Try TOPOLOGICAL with this ordering from initialState (fresh start)
                        for (int preF7Seed : new int[]{0, 9001}) { // 0 = no-random (topological), 9001 = random seed
                            long pBudget = Math.min(timeoutMs - (System.currentTimeMillis() - startTime) - 1000, 30_000L);
                            if (pBudget < 5000) break;

                            StrategyConfig preF7Cfg = new StrategyConfig(
                                    StrategyType.STRICT_ORDER, 1.0,
                                    preF7Seed == 0 ? OrderingMode.TOPOLOGICAL : OrderingMode.RANDOM,
                                    preF7Seed, 0.5);
                            SearchStrategy preF7Strategy = createStrategy(preF7Cfg, level);
                            if (preF7Strategy == null) continue;
                            preF7Strategy.setTimeout((int) pBudget);
                            if (preF7Strategy instanceof PriorityPlanningStrategy) {
                                PriorityPlanningStrategy ppp = (PriorityPlanningStrategy) preF7Strategy;
                                ppp.setGoalExecutionOrder(preF7Order);
                                ppp.setOrderingMode(preF7Seed == 0 ? OrderingMode.TOPOLOGICAL : OrderingMode.RANDOM);
                                if (preF7Seed != 0) ppp.setRandomSeed(preF7Seed);
                                if (!suspendedBoxGoals.isEmpty()) ppp.setSuspendedBoxGoals(suspendedBoxGoals);
                                if (escapeSubgoals != null && !escapeSubgoals.isEmpty()) ppp.setEscapeSubgoals(escapeSubgoals);
                            }
                            List<Action[]> preF7Result = null;
                            try {
                                preF7Result = preF7Strategy.search(initialState, level);
                            } catch (Exception e) {
                                System.err.println("[Portfolio] F7-PRE exception: " + e.getMessage());
                            }
                            if (preF7Result != null && !preF7Result.isEmpty()) {
                                State preF7Final = replayPlan(preF7Result, initialState, level);
                                if (preF7Final != null && preF7Final.isGoalState(level)) {
                                    System.err.println("[Portfolio] F7-PRE SOLVED level (" + preF7Result.size() + " actions)");
                                    printAttemptSummary();
                                    return preF7Result;
                                }
                                int preF7Goals = countSatisfiedGoals(preF7Final, level);
                                int bestCurrent = countSatisfiedGoals(replayPlan(bestPartialPlan, initialState, level), level);
                                System.err.println("[Portfolio] F7-PRE goals=" + preF7Goals
                                        + "/" + (level.getBoxGoalsByType().values().stream().mapToInt(java.util.List::size).sum()
                                                + level.getAgentGoalPositionMap().size())
                                        + " steps=" + preF7Result.size() + " (current best=" + bestCurrent + ")");
                                if (preF7Goals > bestCurrent
                                        || (preF7Goals == bestCurrent && preF7Result.size() < bestPartialPlan.size())) {
                                    bestPartialPlan = preF7Result;
                                    System.err.println("[Portfolio] F7-PRE improved best to " + preF7Goals + " goals");
                                }
                            } else {
                                System.err.println("[Portfolio] F7-PRE no result (seed=" + preF7Seed + ")");
                            }
                        }
                    }
                }
            }
        }
        // ===== END F7-PRE =====

        // F7: Warm-start extension loop. After all fresh attempts produced only partial
        // plans, repeatedly continue from the current best partial's end state using
        // different random seeds. Chains improvements (each improved partial becomes the
        // new base for the next attempt). Uses the full remaining budget.
        // Improvement is judged by GOAL COUNT first, then plan length (competition scoring order).
        // Save the pre-F7-PRE CBSR result as cbsrBestPlan for F7B, so F7B uses the original
        // CBSR planning state (not F7-PRE's potentially harder state).
        // NOTE: cbsrBestPlan was set BEFORE F7-PRE above. bestPartialPlan may have been
        // updated by F7-PRE. Re-save original CBSR result here.
        // (If F7-PRE didn't improve, cbsrBestPlan equals the CBSR result anyway.)
        final List<Action[]> cbsrBestPlan = bestPartialPlan; // F7B base: could be CBSR or F7-PRE result
        {
            final int[] F7_SEEDS = {9001, 42, 137, 9999, 12345, 7777, 55555};
            int f7SeedIdx = 0;
            int f7ConsecutiveNoImprovement = 0;
            int f7Iter = 0;

            // Track best-by-goals separately (competition primary metric)
            List<Action[]> bestByGoalsPlan = bestPartialPlan;
            int bestGoalCount = 0;
            if (bestByGoalsPlan != null) {
                State bestFinal = replayPlan(bestByGoalsPlan, initialState, level);
                bestGoalCount = countSatisfiedGoals(bestFinal, level);
            }

            while (f7SeedIdx < F7_SEEDS.length) {
                long elapsedTotal = System.currentTimeMillis() - startTime;
                long remainingForWarm = timeoutMs - elapsedTotal;
                if (bestPartialPlan == null || bestPartialPlan.size() <= 100 || remainingForWarm < 5000) break;

                State replayed = replayPlan(bestPartialPlan, initialState, level);
                if (replayed == null || replayed.isGoalState(level)) break;

                int seed = F7_SEEDS[f7SeedIdx++];
                f7Iter++;
                // Per-iteration budget: cap at 30s; rely on PP's natural early-exit to bound time
                long perWarmBudget = Math.min(remainingForWarm - 1000, 30_000L);

                System.err.println("[Portfolio] F7." + f7Iter + " warm-start from "
                        + bestPartialPlan.size() + "-step plan, seed=" + seed
                        + " budget=" + perWarmBudget + "ms (remaining=" + remainingForWarm + "ms)");

                StrategyConfig warmCfg = new StrategyConfig(
                        StrategyType.STRICT_ORDER, 1.0,
                        OrderingMode.RANDOM, seed, 0.5);
                SearchStrategy warmStrategy = createStrategy(warmCfg, level);
                if (warmStrategy == null) break;

                warmStrategy.setTimeout((int) perWarmBudget);
                if (warmStrategy instanceof PriorityPlanningStrategy) {
                    PriorityPlanningStrategy wpp = (PriorityPlanningStrategy) warmStrategy;
                    if (!suspendedBoxGoals.isEmpty()) wpp.setSuspendedBoxGoals(suspendedBoxGoals);
                    if (escapeSubgoals != null && !escapeSubgoals.isEmpty()) wpp.setEscapeSubgoals(escapeSubgoals);
                }

                List<Action[]> tail = null;
                try {
                    tail = warmStrategy.search(replayed, level);
                } catch (Exception e) {
                    System.err.println("[Portfolio] F7." + f7Iter + " exception: " + e.getMessage());
                }

                if (tail != null && !tail.isEmpty()) {
                    List<Action[]> combined = new ArrayList<>(bestPartialPlan.size() + tail.size());
                    combined.addAll(bestPartialPlan);
                    combined.addAll(tail);
                    State combinedFinal = replayPlan(combined, initialState, level);
                    if (combinedFinal != null && combinedFinal.isGoalState(level)) {
                        System.err.println("[Portfolio] F7." + f7Iter + " SOLVED level ("
                                + combined.size() + " actions)");
                        printAttemptSummary();
                        return combined;
                    }
                    // Compare by goal count (primary), then by plan length (secondary, less is better)
                    int combinedGoalCount = countSatisfiedGoals(combinedFinal, level);
                    boolean goalImproved = combinedGoalCount > bestGoalCount;
                    boolean lengthImproved = combinedGoalCount == bestGoalCount
                            && combined.size() < bestPartialPlan.size();

                    System.err.println("[Portfolio] F7." + f7Iter + " goals=" + combinedGoalCount
                            + "/" + (level.getBoxGoalsByType().values().stream().mapToInt(java.util.List::size).sum()
                                     + level.getAgentGoalPositionMap().size())
                            + " steps=" + combined.size());

                    if (goalImproved || lengthImproved) {
                        int prevGoalCount = bestGoalCount;
                        bestPartialPlan = combined;
                        bestGoalCount = combinedGoalCount;
                        bestByGoalsPlan = combined;
                        if (warmStrategy instanceof PriorityPlanningStrategy) {
                            mapf.planning.signal.FailureReport wr =
                                    ((PriorityPlanningStrategy) warmStrategy).getLastFailureReport();
                            if (wr != null) bestPartialReport = wr;
                        }
                        System.err.println("[Portfolio] F7." + f7Iter + " improved: goals "
                                + prevGoalCount + "→" + combinedGoalCount
                                + ", steps=" + combined.size());
                        f7ConsecutiveNoImprovement = 0;
                    } else {
                        f7ConsecutiveNoImprovement++;
                        System.err.println("[Portfolio] F7." + f7Iter + " no improvement (consec="
                                + f7ConsecutiveNoImprovement + ")");
                    }
                } else {
                    f7ConsecutiveNoImprovement++;
                    System.err.println("[Portfolio] F7." + f7Iter + " no result (consec="
                            + f7ConsecutiveNoImprovement + ")");
                }

                // Stop if 3+ consecutive non-improvements (budget conservation)
                if (f7ConsecutiveNoImprovement >= 3) {
                    System.err.println("[Portfolio] F7 stopping: 3 consecutive non-improvements");
                    break;
                }
            } // end F7 loop

            // Ensure we return the best-by-goals plan (in case a later iteration degraded goals)
            if (bestByGoalsPlan != null && bestByGoalsPlan != bestPartialPlan) {
                int curGoals = countSatisfiedGoals(replayPlan(bestPartialPlan, initialState, level), level);
                int byGoals = countSatisfiedGoals(replayPlan(bestByGoalsPlan, initialState, level), level);
                if (byGoals > curGoals) {
                    bestPartialPlan = bestByGoalsPlan;
                    System.err.println("[Portfolio] F7 final: restored best-by-goals plan ("
                            + byGoals + " vs " + curGoals + ")");
                }
            }
        }

        // F7B: Independent warm-start from CBSR base using seeds not tried from that base.
        // F7 chained from CBSR base: seed 9001 gave improvement. F7.2+ chained from improved state.
        // F7B tries seeds [42, 137, 9999, 12345, 7777, 55555] from the original CBSR base
        // independently, to discover alternative 25+ goal plans that avoid painting K/L/G/E
        // boxes into bad positions.
        // Uses originalCbsrBestPlan (pre-F7-PRE) to avoid F7-PRE's potentially harder state.
        if (originalCbsrBestPlan != null && originalCbsrBestPlan != bestPartialPlan) {
            int cbsrBaseGoals = countSatisfiedGoals(replayPlan(originalCbsrBestPlan, initialState, level), level);
            int f7bCurrentBest = countSatisfiedGoals(replayPlan(bestPartialPlan, initialState, level), level);
            final int[] F7B_SEEDS = {42, 137, 9999, 12345, 7777, 55555};
            int f7bIter = 0;
            System.err.println("[Portfolio] F7B: trying " + F7B_SEEDS.length + " seeds from CBSR base ("
                    + originalCbsrBestPlan.size() + " steps, " + cbsrBaseGoals + " goals), current best=" + f7bCurrentBest);
            for (int f7bSeed : F7B_SEEDS) {
                long elapsed7b = System.currentTimeMillis() - startTime;
                long remaining7b = timeoutMs - elapsed7b;
                if (remaining7b < 5000) break;
                State cbsrFinal = replayPlan(originalCbsrBestPlan, initialState, level);
                if (cbsrFinal == null || cbsrFinal.isGoalState(level)) break;

                f7bIter++;
                long perF7bBudget = Math.min(remaining7b - 1000, 30_000L);
                System.err.println("[Portfolio] F7B." + f7bIter + " warm-start from CBSR base, seed=" + f7bSeed
                        + " budget=" + perF7bBudget + "ms");

                StrategyConfig f7bCfg = new StrategyConfig(
                        StrategyType.STRICT_ORDER, 1.0,
                        OrderingMode.RANDOM, f7bSeed, 0.5);
                SearchStrategy f7bStrategy = createStrategy(f7bCfg, level);
                if (f7bStrategy == null) break;
                f7bStrategy.setTimeout((int) perF7bBudget);
                if (f7bStrategy instanceof PriorityPlanningStrategy) {
                    PriorityPlanningStrategy wpp = (PriorityPlanningStrategy) f7bStrategy;
                    if (!suspendedBoxGoals.isEmpty()) wpp.setSuspendedBoxGoals(suspendedBoxGoals);
                    if (escapeSubgoals != null && !escapeSubgoals.isEmpty()) wpp.setEscapeSubgoals(escapeSubgoals);
                }

                List<Action[]> f7bTail = null;
                try {
                    f7bTail = f7bStrategy.search(cbsrFinal, level);
                } catch (Exception e) {
                    System.err.println("[Portfolio] F7B." + f7bIter + " exception: " + e.getMessage());
                }

                if (f7bTail != null && !f7bTail.isEmpty()) {
                    List<Action[]> f7bCombined = new ArrayList<>(originalCbsrBestPlan.size() + f7bTail.size());
                    f7bCombined.addAll(originalCbsrBestPlan);
                    f7bCombined.addAll(f7bTail);
                    State f7bFinal = replayPlan(f7bCombined, initialState, level);
                    if (f7bFinal != null && f7bFinal.isGoalState(level)) {
                        System.err.println("[Portfolio] F7B." + f7bIter + " SOLVED level ("
                                + f7bCombined.size() + " actions)");
                        printAttemptSummary();
                        return f7bCombined;
                    }
                    int f7bGoals = countSatisfiedGoals(f7bFinal, level);
                    System.err.println("[Portfolio] F7B." + f7bIter + " goals=" + f7bGoals
                            + " steps=" + f7bCombined.size());
                    if (f7bGoals > f7bCurrentBest
                            || (f7bGoals == f7bCurrentBest && f7bCombined.size() < bestPartialPlan.size())) {
                        bestPartialPlan = f7bCombined;
                        f7bCurrentBest = f7bGoals;
                        System.err.println("[Portfolio] F7B." + f7bIter + " improved best to " + f7bGoals + " goals");
                    }
                } else {
                    System.err.println("[Portfolio] F7B." + f7bIter + " no result");
                }
            }
        }

        if (bestPartialPlan != null) {
            // Count satisfied goals in the final partial plan state
            State finalPartial = replayPlan(bestPartialPlan, initialState, level);
            int satisfiedGoals = 0, totalGoals = 0;
            if (finalPartial != null) {
                for (int row = 0; row < level.getRows(); row++) {
                    for (int col = 0; col < level.getCols(); col++) {
                        char goalType = level.getBoxGoal(row, col);
                        if (goalType != '\0') {
                            totalGoals++;
                            if (finalPartial.getBoxAt(Position.of(row, col)) == goalType) satisfiedGoals++;
                        }
                        int agGoal = level.getAgentGoal(row, col);
                        if (agGoal != -1) {
                            totalGoals++;
                            Position agPos = finalPartial.getAgentPosition(agGoal);
                            if (agPos != null && agPos.row == row && agPos.col == col) satisfiedGoals++;
                        }
                    }
                }
            }
            System.err.println("[Portfolio] No full solution found. Returning best partial plan ("
                + bestPartialPlan.size() + " steps, " + satisfiedGoals + "/" + totalGoals + " goals satisfied)");
            // Diagnostic: show which goals are NOT satisfied
            if (finalPartial != null) {
                StringBuilder unsatGoals = new StringBuilder("[Portfolio] Unsatisfied goals:");
                for (int row = 0; row < level.getRows(); row++) {
                    for (int col = 0; col < level.getCols(); col++) {
                        char goalType = level.getBoxGoal(row, col);
                        if (goalType != '\0' && finalPartial.getBoxAt(Position.of(row, col)) != goalType) {
                            char actualBox = finalPartial.getBoxAt(Position.of(row, col));
                            unsatGoals.append(" box").append(goalType).append("@(").append(row).append(",").append(col).append(")");
                            if (actualBox != '\0') unsatGoals.append("[has ").append(actualBox).append("]");
                        }
                        int agGoal = level.getAgentGoal(row, col);
                        if (agGoal != -1) {
                            Position agPos = finalPartial.getAgentPosition(agGoal);
                            if (agPos == null || agPos.row != row || agPos.col != col) {
                                unsatGoals.append(" agent").append(agGoal).append("->@(").append(row).append(",").append(col).append(")");
                                if (agPos != null) unsatGoals.append("[at(").append(agPos.row).append(",").append(agPos.col).append(")]");
                            }
                        }
                    }
                }
                System.err.println(unsatGoals);
            }
            printAttemptSummary();
            writeFailureSnapshot(level, initialState, bestPartialPlan, lastPP, lastReportForSnapshot, bestPartialReport);
            return bestPartialPlan;
        }
        
        System.err.println("[Portfolio] All strategies failed");
        printAttemptSummary();
        writeFailureSnapshot(level, initialState, null, lastPP, lastReportForSnapshot, null);
        return null;
    }
    
    /**
     * Replay a plan from initial state to get the final state.
     * Used to verify if a returned plan actually solves the level.
     */
    private State replayPlan(List<Action[]> plan, State initialState, Level level) {
        State state = initialState;
        for (Action[] jointAction : plan) {
            try {
                state = state.applyJointAction(jointAction, level);
            } catch (Exception e) {
                return state; // Return whatever state we reached
            }
        }
        return state;
    }

    /**
     * Counts the number of satisfied goals (box + agent) in a state.
     * Returns 0 if state is null. Used to compare partial plans by quality.
     */
    private int countSatisfiedGoals(State state, Level level) {
        if (state == null) return 0;
        int count = 0;
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0') {
                    if (state.getBoxAt(Position.of(row, col)) == goalType) count++;
                }
                int agGoal = level.getAgentGoal(row, col);
                if (agGoal != -1) {
                    Position agPos = state.getAgentPosition(agGoal);
                    if (agPos != null && agPos.row == row && agPos.col == col) count++;
                }
            }
        }
        return count;
    }
    
    /**
     * Builds strategy sequence based on level features.
     * 
     * Pull-Sokoban portfolio design principles:
     * - All multi-agent levels use PriorityPlanningStrategy with different ordering modes.
     *   (CBS / JointAStar removed: CBS models point-to-point MAPF, not Sokoban push/pull;
     *   JointAStar is O(5^n) unusable above 3 agents.)
     * - REVERSE_TOPOLOGICAL is removed (never solved any competition level in testing).
     * - Multi-seed RANDOM is last-resort diversification with minimal budget.
     * 
     * Feature-aware branching:
     * - corridorRatio distinguishes spiral/maze (>0.5) from open-room (<= 0.5) topologies.
     *   Spiral levels need DISTANCE_FARTHEST (outer goals first); open levels need GREEDY.
     * - hasCircularDependency gates whether dependency-aware ordering is possible.
     * - Single-agent threshold (activeBoxGoals > 8) routes large levels to PP decomposition.
     */
    private List<StrategyConfig> buildStrategySequence(LevelFeatures f, State state) {
        List<StrategyConfig> strategies = new ArrayList<>();
        
        if (f.recommendedStrategy == StrategyType.SINGLE_AGENT) {
            // Count active box goals (total goals minus already-satisfied minus agent goals)
            int activeBoxGoals = f.numGoals;
            if (f.taskFilter != null) {
                activeBoxGoals -= f.taskFilter.satisfiedGoals.size();
            }
            
            if (activeBoxGoals > 8) {
                // Many box goals: SingleAgentStrategy's full-state closed list causes 
                // state explosion (e.g., pacMAn with 45+ same-type boxes).
                // Use PriorityPlanningStrategy which decomposes into one-box-at-a-time subgoals.
                System.err.println("[Portfolio] Single agent with " + activeBoxGoals + 
                    " active goals → using PP decomposition instead of full-state A*");
                // Warm-start orderings only; CBSR handles the remaining budget with nogood-guided repair.
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.40));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.35));
            } else {
                // Few box goals: SingleAgentStrategy A* is efficient
                strategies.add(new StrategyConfig(StrategyType.SINGLE_AGENT, 1.0, null, 0, 0.40));
                strategies.add(new StrategyConfig(StrategyType.SINGLE_AGENT, 5.0, null, 0, 0.60));
            }
        } else if (f.hasCircularDependency) {
            // Cyclic dependencies: strategy depends on topology.
            if (f.corridorRatio > 0.5) {
                // Spiral/maze topology (high corridor ratio): outer goals must be filled
                // before inner goals block narrow corridors with cross-color boxes.
                // DISTANCE_FARTHEST is the primary strategy.
                System.err.println("[Portfolio] Cyclic + spiral topology (corridorRatio=" + 
                    String.format("%.2f", f.corridorRatio) + ") → FARTHEST-first");
                // Warm-start orderings only; CBSR handles the remaining budget with nogood-guided repair.
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_FARTHEST, 0, 0.40));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.30));
            } else {
                // Open-room cyclic topology (low corridor ratio): TOPO uses partial
                // ordering from dependency analysis (valuable even with cycles).
                // GREEDY nearest-first is a good fallback for coordination-heavy levels.
                System.err.println("[Portfolio] Cyclic + open topology (corridorRatio=" + 
                    String.format("%.2f", f.corridorRatio) + ") → TOPO-first");
                // Warm-start orderings only; CBSR handles the remaining budget with nogood-guided repair.
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.35));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.35));
            }
        } else {
            // No cycles: topological order is well-founded, give it the most budget.
            // Warm-start orderings only; CBSR handles the remaining budget with nogood-guided repair.
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.40));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.30));
        }
        
        return strategies;
    }
    
    /**
     * Computes timeout for a single attempt based on pre-assigned time budget fraction.
     * Fractions are relative to total timeout, but capped by remaining time.
     */
    private long computeAttemptTimeout(StrategyConfig config, long remainingTime, int totalStrategies) {
        long budgetMs = (long) (timeoutMs * config.timeBudgetFraction);
        return Math.min(budgetMs, remainingTime);
    }
    
    /**
     * Creates actual strategy instance based on config.
     */
    private SearchStrategy createStrategy(StrategyConfig config, Level level) {
        Heuristic heuristic = createHeuristic(level);
        SearchConfig strategyConfig = new SearchConfig(
            config.weight == Double.POSITIVE_INFINITY ? this.config.getTimeoutMs() : this.config.getTimeoutMs(),
            this.config.getMaxStates(),
            config.weight
        );
        
        switch (config.type) {
            case SINGLE_AGENT:
                SingleAgentStrategy singleAgent = new SingleAgentStrategy(heuristic, strategyConfig);
                singleAgent.setWeight(config.weight);
                return singleAgent;

            case STRICT_ORDER:
            case CYCLE_BREAKER:
            case GREEDY_WITH_RETRY:
            default:
                // All use PriorityPlanningStrategy with different ordering modes
                // Share SubgoalManager across retries to reuse BFS distance cache
                if (cachedSubgoalManager == null) {
                    cachedSubgoalManager = new mapf.planning.strategy.SubgoalManager(heuristic);
                }
                PriorityPlanningStrategy priorityPlanning = new PriorityPlanningStrategy(heuristic, strategyConfig, cachedSubgoalManager);
                // Pass execution order from analysis
                if (features != null && features.executionOrder != null) {
                    priorityPlanning.setGoalExecutionOrder(features.executionOrder);
                }
                // Pass goal dependencies (for hard/soft frozen distinction)
                if (features != null && features.goalDependsOn != null) {
                    priorityPlanning.setGoalDependencies(features.goalDependsOn);
                }
                // Pass immovable boxes (treated as walls)
                if (features != null && features.taskFilter != null) {
                    priorityPlanning.setImmovableBoxes(features.taskFilter.immovableBoxes);
                }
                // Set ordering mode for this retry attempt
                if (config.orderingMode != null) {
                    priorityPlanning.setOrderingMode(config.orderingMode);
                }
                // Set random seed for RANDOM ordering (multi-seed diversification)
                if (config.orderingMode == OrderingMode.RANDOM && config.randomSeed != 0) {
                    priorityPlanning.setRandomSeed(config.randomSeed);
                }
                return priorityPlanning;
        }
    }
    
    private Heuristic createHeuristic(Level level) {
        // Cache heuristic - BFS precomputation is expensive
        if (cachedHeuristic == null) {
            try {
                cachedHeuristic = new TrueDistanceHeuristic(level);
            } catch (Exception e) {
                cachedHeuristic = new ManhattanHeuristic();
            }
        }
        return cachedHeuristic;
    }
    
    /**
     * Write a structured failure snapshot to {@code target/diagnostics/}. Best-effort:
     * never throws. See {@link mapf.planning.diag.FailureSnapshot}.
     */
    private void writeFailureSnapshot(Level level, State initialState,
                                      List<Action[]> bestPartialPlan,
                                      PriorityPlanningStrategy lastPP,
                                      mapf.planning.signal.FailureReport lastReport,
                                      mapf.planning.signal.FailureReport bestPartialReport) {
        try {
            State partialFinal = (bestPartialPlan != null && !bestPartialPlan.isEmpty())
                    ? replayPlan(bestPartialPlan, initialState, level)
                    : initialState;
            int steps = bestPartialPlan != null ? bestPartialPlan.size() : 0;
            List<mapf.planning.diag.FailureSnapshot.AttemptInfo> infos = new ArrayList<>(attempts.size());
            for (AttemptRecord r : attempts) {
                String label;
                if (r.orderingMode == null) label = r.strategy.name();
                else if (r.orderingMode == OrderingMode.RANDOM)
                    label = r.strategy.name() + "(RANDOM#" + r.randomSeed + ")";
                else label = r.strategy.name() + "(" + r.orderingMode.name() + ")";
                infos.add(new mapf.planning.diag.FailureSnapshot.AttemptInfo(
                        label, r.durationMs, r.success, r.planSteps,
                        r.unsatCount, r.failedSubgoal, r.failureKind));
            }
            Set<Position> completed = lastPP != null ? lastPP.getCompletedBoxGoals() : null;
            Set<Position> suspended = lastPP != null ? lastPP.getSuspendedTransitGoals() : null;
            Map<Position, mapf.planning.analysis.GoalTransitAnalyzer.GoalProfile> profiles =
                    lastPP != null ? lastPP.getTransitProfiles() : null;
            java.nio.file.Path file = mapf.planning.diag.FailureSnapshot.dump(
                    mapf.planning.diag.FailureSnapshot.defaultDir(),
                    level, initialState, partialFinal, features, infos, lastReport,
                    bestPartialReport,
                    completed, suspended, profiles, steps);
            if (file != null) {
                System.err.println("[SNAPSHOT] wrote " + file);
            }
        } catch (Throwable t) {
            System.err.println("[SNAPSHOT] failed: " + t);
        }
    }

    private void printAttemptSummary() {
        if (!SearchConfig.isMinimal()) return;
        if (attempts.isEmpty()) {
            System.err.println("\n=== Portfolio Attempt Summary === (no attempts)");
            return;
        }
        // Build a fixed-width table. Columns:
        //   #  | Strategy(Mode#seed)        | Steps | Unsat | Reliefs | Susp | Time     | Result/Failed subgoal
        String header = String.format("%-3s %-26s %6s %5s %4s %4s %8s  %s",
                "#", "Strategy(Mode)", "Steps", "Unsat", "Rlf", "Susp", "Time", "Failure / Result");
        String sep = new String(new char[header.length()]).replace('\0', '-');
        System.err.println("\n=== Portfolio Attempt Summary ===");
        System.err.println(sep);
        System.err.println(header);
        System.err.println(sep);
        int successAttempt = -1;
        long totalMs = 0;
        for (int i = 0; i < attempts.size(); i++) {
            AttemptRecord r = attempts.get(i);
            String mode;
            if (r.orderingMode == null) {
                mode = r.strategy.name();
            } else if (r.orderingMode == OrderingMode.RANDOM) {
                mode = r.strategy.name() + "(" + r.orderingMode.name() + "#" + r.randomSeed + ")";
            } else {
                mode = r.strategy.name() + "(" + r.orderingMode.name() + ")";
            }
            if (mode.length() > 26) mode = mode.substring(0, 26);
            String result;
            if (r.success) {
                result = "SUCCESS";
                if (successAttempt < 0) successAttempt = i + 1;
            } else if (r.failedSubgoal != null) {
                result = r.failureKind + " " + r.failedSubgoal;
            } else {
                result = r.failureKind;
            }
            String unsat = r.unsatCount >= 0 ? String.valueOf(r.unsatCount) : "-";
            System.err.println(String.format("%-3d %-26s %6d %5s %4d %4d %7dms  %s",
                    i + 1, mode, r.planSteps, unsat, r.reliefCount, r.suspendedCount,
                    r.durationMs, result));
            totalMs += r.durationMs;
        }
        System.err.println(sep);
        System.err.println(String.format("Total attempts: %d  |  Total time: %dms", attempts.size(), totalMs));
        if (successAttempt > 0) {
            System.err.println("[Portfolio][DIAG] Solved on attempt " + successAttempt + "/" + attempts.size()
                    + (successAttempt == 1 ? " (no reordering needed)" : " (reordering rescued)"));
        } else {
            // Surface the dominant blocker for triage
            Map<String, Integer> blockerCounts = new HashMap<>();
            int bestSteps = 0;
            String bestStrategy = "?";
            String bestBlocker = null;
            for (AttemptRecord r : attempts) {
                if (r.failedSubgoal != null) {
                    blockerCounts.merge(r.failedSubgoal, 1, Integer::sum);
                }
                if (r.planSteps > bestSteps) {
                    bestSteps = r.planSteps;
                    bestStrategy = r.orderingMode != null
                            ? r.strategy.name() + "(" + r.orderingMode.name()
                                + (r.orderingMode == OrderingMode.RANDOM ? "#" + r.randomSeed : "") + ")"
                            : r.strategy.name();
                    bestBlocker = r.failedSubgoal;
                }
            }
            String dominantBlocker = blockerCounts.entrySet().stream()
                    .max(Map.Entry.comparingByValue())
                    .map(e -> e.getKey() + " (x" + e.getValue() + ")")
                    .orElse("none");
            System.err.println("[Portfolio][DIAG] All " + attempts.size()
                    + " attempts FAILED — action-layer fix needed");
            System.err.println("[Portfolio][DIAG] Best partial: " + bestSteps + " steps via "
                    + bestStrategy + (bestBlocker != null ? " blocked at " + bestBlocker : ""));
            System.err.println("[Portfolio][DIAG] Dominant blocker: " + dominantBlocker);

            // Bug-1 follow-up (2026-05): print full ancestor / dependent chains for each
            // distinct failed subgoal. Reads features.goalDependsOn (X depends on deps[X]
            // means deps[X] must be filled first). For each blocker:
            //   ANCESTORS  = transitive closure of dependsOn — what must be filled BEFORE X
            //   DEPENDENTS = transitive closure of inverse  — who is BLOCKED by X
            if (SearchConfig.isVerbose() && features != null && features.goalDependsOn != null
                    && lastLevel != null && !blockerCounts.isEmpty()) {
                printBlockerDepChains(blockerCounts.keySet(), features.goalDependsOn, lastLevel);
            }
        }
    }

    /**
     * Bug-1 follow-up diagnostics. For each unique failed-subgoal string (e.g.
     * "agent7->K@(34,31)"), parse the (row,col) and print:
     *   - ancestors (must-fill-first, via dependsOn)
     *   - dependents (who is waiting, via inverse)
     * Indented BFS, max depth 6, dedup repeats with "... (cycle)".
     */
    private void printBlockerDepChains(Set<String> failedSubgoalStrings,
                                       Map<Position, Set<Position>> dependsOn,
                                       Level level) {
        // Build inverse: dependents[X] = { Y : X in dependsOn[Y] } — who is waiting on X.
        Map<Position, Set<Position>> dependents = new HashMap<>();
        for (Map.Entry<Position, Set<Position>> e : dependsOn.entrySet()) {
            for (Position dep : e.getValue()) {
                dependents.computeIfAbsent(dep, k -> new HashSet<>()).add(e.getKey());
            }
        }

        java.util.regex.Pattern posPat = java.util.regex.Pattern.compile("@\\((\\d+),(\\d+)\\)");
        for (String s : failedSubgoalStrings) {
            java.util.regex.Matcher m = posPat.matcher(s);
            if (!m.find()) continue;
            int row, col;
            try {
                row = Integer.parseInt(m.group(1));
                col = Integer.parseInt(m.group(2));
            } catch (NumberFormatException ex) { continue; }
            Position p = Position.of(row, col);
            // Sanity: p must appear in the dep map (i.e. it's an active goal).
            if (!dependsOn.containsKey(p) && !dependents.containsKey(p)) {
                System.err.println("[Portfolio][DIAG]   chain for " + s
                        + ": position not in dep graph (already satisfied or not an active goal)");
                continue;
            }
            String selfLbl = formatGoalLabel(p, level);
            System.err.println("[Portfolio][DIAG] dependency chain for " + s + "  (" + selfLbl + ")");
            System.err.println("[Portfolio][DIAG]   ancestors  (must fill BEFORE " + selfLbl + "):");
            printChain(p, dependsOn, level, 0, 6, new HashSet<>());
            System.err.println("[Portfolio][DIAG]   dependents (BLOCKED by " + selfLbl + "):");
            printChain(p, dependents, level, 0, 6, new HashSet<>());
        }
    }

    /** Recursively prints the chain from start. graph[X] = neighbours of X to recurse into. */
    private void printChain(Position start, Map<Position, Set<Position>> graph, Level level,
                            int depth, int maxDepth, Set<Position> visited) {
        Set<Position> nbrs = graph.getOrDefault(start, Collections.emptySet());
        if (nbrs.isEmpty()) {
            if (depth == 0) System.err.println("[Portfolio][DIAG]     (none)");
            return;
        }
        List<Position> sorted = new ArrayList<>(nbrs);
        sorted.sort(Comparator.<Position>comparingInt(q -> q.row).thenComparingInt(q -> q.col));
        StringBuilder pad = new StringBuilder("[Portfolio][DIAG]     ");
        for (int i = 0; i < depth; i++) pad.append("  ");
        for (Position q : sorted) {
            String line = pad.toString() + "- " + formatGoalLabel(q, level);
            if (visited.contains(q)) {
                System.err.println(line + " (already shown / cycle)");
                continue;
            }
            if (depth + 1 >= maxDepth) {
                System.err.println(line + " (depth cap)");
                continue;
            }
            System.err.println(line);
            visited.add(q);
            printChain(q, graph, level, depth + 1, maxDepth, visited);
        }
    }

    /** Lightweight label: "X@(r,c)" with X = box-goal letter / 'a<id>' / '?'. */
    private static String formatGoalLabel(Position p, Level level) {
        char bg = level.getBoxGoal(p);
        int ag = level.getAgentGoal(p);
        String head;
        if (bg != 0) head = String.valueOf(bg);
        else if (ag != -1) head = "a" + ag;
        else head = "?";
        return head + "@(" + p.row + "," + p.col + ")";
    }
    
    /**
     * Returns the analysis result (for debugging/testing).
     */
    public LevelFeatures getFeatures() {
        return features;
    }
    
    // ========== Independence Detection ==========
    
    /**
     * Detects independent agent groups using BFS reachability.
     * Two agents are in the same group if their reachable areas overlap
     * (i.e., they can physically reach each other's positions).
     * Immovable boxes (pre-satisfied boxes that shouldn't be moved) are treated as walls
     * because pushing them would unsatisfy goals. This matches PPS's componentization.
     */
    private List<List<Integer>> detectIndependentGroups(State state, Level level) {
        int numAgents = state.getNumAgents();
        Set<Position> immovable = features != null && features.taskFilter != null 
            ? features.taskFilter.immovableBoxes : Collections.emptySet();
        
        // BFS reachability per agent
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
                    if (!level.isWall(next) && !immovable.contains(next) && !area.contains(next)) {
                        area.add(next);
                        queue.add(next);
                    }
                }
            }
            reachable.put(a, area);
        }
        
        // Union-Find: agents whose reachable sets overlap
        int[] parent = new int[numAgents];
        for (int i = 0; i < numAgents; i++) parent[i] = i;
        for (int a = 0; a < numAgents; a++) {
            for (int b = a + 1; b < numAgents; b++) {
                if (reachable.get(a).contains(state.getAgentPosition(b))) {
                    ufUnion(parent, a, b);
                }
            }
        }
        
        // Group by root
        Map<Integer, List<Integer>> groups = new LinkedHashMap<>();
        for (int a = 0; a < numAgents; a++) {
            groups.computeIfAbsent(ufFind(parent, a), k -> new ArrayList<>()).add(a);
        }
        return new ArrayList<>(groups.values());
    }
    
    private int ufFind(int[] parent, int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    
    private void ufUnion(int[] parent, int a, int b) {
        parent[ufFind(parent, a)] = ufFind(parent, b);
    }

    // ========== P3: Cross-Group Goal-Dependency Merge ==========

    /**
     * Merges groups whose goals have dependencies crossing group boundaries.
     *
     * Motivation (qanda.txt 2.2 / claudeopus47.txt 5.2):
     * Independence detection (BFS reachability + footprint refinement) identifies
     * spatial / physical coupling but does NOT consult the goal dependency graph.
     * If goalDependsOn says "goal A (in group X) depends on goal B (in group Y)",
     * projecting X in isolation drops B from the level entirely \u2014 making A
     * unsolvable as an isolated subproblem. The result is a wasted partial-plan
     * attempt plus a forced fallback to full-portfolio solve.
     *
     * This merge step uses union-find: for every dependency edge whose endpoints
     * lie in DIFFERENT current groups, union those groups. Conservative by design:
     * only ever merges (reduces group count); never splits.
     *
     * Mapping rule (goal -> candidate groups):
     *   - Agent-goal at p: the unique agent that owns that goal -> its group.
     *   - Box-goal at p of color c: every group containing at least one agent of
     *     color c (any of them could service that goal). If multiple groups match,
     *     they all become candidates and pairwise dependency edges merge them.
     */
    private List<List<Integer>> mergeGroupsByGoalDependencies(
            List<List<Integer>> groups, Level level,
            Map<Position, Set<Position>> goalDependsOn) {

        int numAgents = level.getNumAgents();
        int[] agentGroup = new int[numAgents];
        Arrays.fill(agentGroup, -1);
        for (int gi = 0; gi < groups.size(); gi++) {
            for (int aid : groups.get(gi)) {
                if (aid >= 0 && aid < numAgents) agentGroup[aid] = gi;
            }
        }

        // Pre-compute color -> set of group indices that contain at least one agent of that color.
        Map<Color, Set<Integer>> colorToGroups = new HashMap<>();
        for (int aid = 0; aid < numAgents; aid++) {
            if (agentGroup[aid] < 0) continue;
            Color c = level.getAgentColor(aid);
            if (c == null) continue;
            colorToGroups.computeIfAbsent(c, k -> new HashSet<>()).add(agentGroup[aid]);
        }

        int[] parent = new int[groups.size()];
        for (int i = 0; i < parent.length; i++) parent[i] = i;

        // Cache resolved goal -> candidate groups so we don't rescan per edge.
        Map<Position, Set<Integer>> goalGroupsCache = new HashMap<>();
        java.util.function.Function<Position, Set<Integer>> resolve = goalPos -> {
            Set<Integer> cached = goalGroupsCache.get(goalPos);
            if (cached != null) return cached;
            Set<Integer> resolved = resolveGoalGroups(goalPos, level, agentGroup, colorToGroups);
            goalGroupsCache.put(goalPos, resolved);
            return resolved;
        };

        int crossEdges = 0;
        for (Map.Entry<Position, Set<Position>> entry : goalDependsOn.entrySet()) {
            Position src = entry.getKey();
            Set<Position> deps = entry.getValue();
            if (deps == null || deps.isEmpty()) continue;
            Set<Integer> srcGroups = resolve.apply(src);
            if (srcGroups.isEmpty()) continue;
            for (Position dst : deps) {
                Set<Integer> dstGroups = resolve.apply(dst);
                if (dstGroups.isEmpty()) continue;
                // Edge crosses if there is at least one (s, d) pair in different groups.
                boolean crossed = false;
                for (int s : srcGroups) {
                    for (int d : dstGroups) {
                        if (ufFind(parent, s) != ufFind(parent, d)) {
                            ufUnion(parent, s, d);
                            crossed = true;
                        }
                    }
                }
                if (crossed) crossEdges++;
            }
        }

        if (crossEdges == 0) return groups;

        // Re-bucket groups by their union-find roots.
        Map<Integer, List<Integer>> merged = new LinkedHashMap<>();
        for (int gi = 0; gi < groups.size(); gi++) {
            merged.computeIfAbsent(ufFind(parent, gi), k -> new ArrayList<>()).addAll(groups.get(gi));
        }
        // Sort agent ids inside each merged group for deterministic logging.
        List<List<Integer>> result = new ArrayList<>(merged.values());
        for (List<Integer> g : result) Collections.sort(g);
        return result;
    }

    // ========== D13: NAMO-Coupling Group Merge ==========

    /**
     * Merges groups whose agents are NAMO-coupled (one needs another's color
     * to clear blockers from its push path). Complements
     * {@link #mergeGroupsByGoalDependencies}: that one consults goalDependsOn,
     * this one consults the actual geometry of cross-color blockers.
     *
     * <p>Uses {@link mapf.planning.synthesis.BlockerReliefSynthesizer#probeNAMOCoupling}
     * to obtain a {@code agentColor -> blockerColors} map. For each entry, unions
     * every group containing an agent of {@code agentColor} with every group
     * containing an agent of any {@code blockerColor}.
     *
     * <p>Conservative: only ever merges, never splits. Returns the input list
     * unchanged if no NAMO coupling is detected.
     */
    private List<List<Integer>> mergeGroupsByNAMOCoupling(
            List<List<Integer>> groups, State state, Level level) {
        int numAgents = level.getNumAgents();
        Set<Position> immovable = features != null && features.taskFilter != null
                ? features.taskFilter.immovableBoxes : Collections.emptySet();
        Map<Color, Set<Color>> coupling =
                mapf.planning.synthesis.BlockerReliefSynthesizer.probeNAMOCoupling(
                        state, level, immovable);
        if (coupling.isEmpty()) return groups;

        // Build agent -> group index and color -> set-of-groups map.
        int[] agentGroup = new int[numAgents];
        Arrays.fill(agentGroup, -1);
        for (int gi = 0; gi < groups.size(); gi++) {
            for (int aid : groups.get(gi)) {
                if (aid >= 0 && aid < numAgents) agentGroup[aid] = gi;
            }
        }
        Map<Color, Set<Integer>> colorToGroups = new HashMap<>();
        for (int aid = 0; aid < numAgents; aid++) {
            if (agentGroup[aid] < 0) continue;
            Color c = level.getAgentColor(aid);
            if (c == null) continue;
            colorToGroups.computeIfAbsent(c, k -> new HashSet<>()).add(agentGroup[aid]);
        }

        int[] parent = new int[groups.size()];
        for (int i = 0; i < parent.length; i++) parent[i] = i;
        int unions = 0;
        for (Map.Entry<Color, Set<Color>> e : coupling.entrySet()) {
            Set<Integer> srcGroups = colorToGroups.get(e.getKey());
            if (srcGroups == null || srcGroups.isEmpty()) continue;
            for (Color blockerColor : e.getValue()) {
                Set<Integer> dstGroups = colorToGroups.get(blockerColor);
                if (dstGroups == null || dstGroups.isEmpty()) continue;
                for (int s : srcGroups) {
                    for (int d : dstGroups) {
                        if (ufFind(parent, s) != ufFind(parent, d)) {
                            ufUnion(parent, s, d);
                            unions++;
                        }
                    }
                }
            }
        }
        if (unions == 0) return groups;

        Map<Integer, List<Integer>> bucketed = new LinkedHashMap<>();
        for (int gi = 0; gi < groups.size(); gi++) {
            bucketed.computeIfAbsent(ufFind(parent, gi), k -> new ArrayList<>()).addAll(groups.get(gi));
        }
        List<List<Integer>> out = new ArrayList<>(bucketed.values());
        for (List<Integer> g : out) Collections.sort(g);
        return out;
    }

    /**
     * Resolves the candidate group set that could service a given goal position.
     * Returns empty set if the goal has no servicing agents (treated as orphan).
     */
    private Set<Integer> resolveGoalGroups(Position goalPos, Level level,
                                            int[] agentGroup,
                                            Map<Color, Set<Integer>> colorToGroups) {
        // Agent-goal: unique owner.
        int agentId = level.getAgentGoal(goalPos);
        if (agentId >= 0 && agentId < agentGroup.length && agentGroup[agentId] >= 0) {
            Set<Integer> single = new HashSet<>();
            single.add(agentGroup[agentId]);
            return single;
        }
        // Box-goal: any group with a same-color agent could service.
        char boxType = level.getBoxGoal(goalPos);
        if (boxType != '\0') {
            Color c = level.getBoxColor(boxType);
            if (c != null) {
                Set<Integer> grps = colorToGroups.get(c);
                if (grps != null) return grps;
            }
        }
        return Collections.emptySet();
    }

    // ========== Footprint-Based Group Refinement (P1) ==========

    /**
     * Refines physical-reachability groups by per-agent "footprint" disjointness.
     * 
     * For each agent we compute its footprint = the cells it must occupy/traverse:
     *   - agent's current position
     *   - agent's goal (if any)
     *   - for each box of matching color with an unsatisfied same-component goal:
     *       * box current position
     *       * box goal position
     *       * BFS shortest path cells from box to goal (walls only)
     *   - BFS shortest path from agent to each owned box (walls only)
     * 
     * Two agents within the same physical group are considered TRULY coupled iff:
     *   (a) they own the same box, OR
     *   (b) they target the same box-goal cell, OR
     *   (c) their footprints share more than {@code FOOTPRINT_TOLERANCE} cells
     *       (small overlap = corridor pass-through, tolerate to avoid over-refinement)
     * 
     * Conservative by design: any uncertainty keeps agents coupled. Refinement is
     * only applied if it strictly increases group count AND no group becomes
     * "agent-with-goal-but-no-color-match" pathological.
     */
    private List<List<Integer>> refineGroupsByFootprint(List<List<Integer>> physicalGroups,
                                                          State state, Level level) {
        // Tolerance: paths in the same big room often share 1-2 corridor cells
        // without the agents actually conflicting. Higher = more aggressive split.
        final int FOOTPRINT_TOLERANCE = 2;
        
        List<List<Integer>> result = new ArrayList<>();
        for (List<Integer> phys : physicalGroups) {
            // Only refine groups with 4+ agents. Smaller groups (2-3) usually solve
            // fast enough as a single subproblem, and over-splitting them costs both
            // time (per-group portfolio overhead) and plan quality (independent
            // subplans cannot share agent timing). Empirically: AIMAS7 [0,1]/[5,6]
            // pairs solved 100 actions in 0.26s as one portfolio, but 115 actions
            // in 40s when forced into singletons.
            if (phys.size() < 4) {
                result.add(phys);
                continue;
            }
            
            // Compute per-agent footprint and box-ownership
            Map<Integer, Set<Position>> footprints = new HashMap<>();
            Map<Integer, Set<Position>> ownedBoxes = new HashMap<>();
            Map<Integer, Set<Position>> targetGoals = new HashMap<>();
            for (int a : phys) {
                FootprintInfo fp = computeFootprint(a, state, level);
                footprints.put(a, fp.cells);
                ownedBoxes.put(a, fp.boxPositions);
                targetGoals.put(a, fp.goalPositions);
            }
            
            // Union-Find by coupling
            Map<Integer, Integer> idx = new HashMap<>();
            for (int i = 0; i < phys.size(); i++) idx.put(phys.get(i), i);
            int[] parent = new int[phys.size()];
            for (int i = 0; i < phys.size(); i++) parent[i] = i;
            
            for (int i = 0; i < phys.size(); i++) {
                int a = phys.get(i);
                for (int j = i + 1; j < phys.size(); j++) {
                    int b = phys.get(j);
                    if (footprintsCouple(a, b, footprints, ownedBoxes, targetGoals,
                            FOOTPRINT_TOLERANCE)) {
                        ufUnion(parent, i, j);
                    }
                }
            }
            
            Map<Integer, List<Integer>> sub = new LinkedHashMap<>();
            for (int i = 0; i < phys.size(); i++) {
                sub.computeIfAbsent(ufFind(parent, i), k -> new ArrayList<>()).add(phys.get(i));
            }
            result.addAll(sub.values());
        }
        
        // Safety: if refinement produced a strictly larger partition that respects
        // every agent goal having its agent in the same subgroup as needed boxes,
        // accept it. Else fall back to physical groups.
        if (result.size() <= physicalGroups.size()) {
            return physicalGroups;
        }
        return result;
    }
    
    /** Per-agent footprint summary used by refinement. */
    private static class FootprintInfo {
        final Set<Position> cells = new HashSet<>();
        final Set<Position> boxPositions = new HashSet<>();
        final Set<Position> goalPositions = new HashSet<>();
    }
    
    private FootprintInfo computeFootprint(int agentId, State state, Level level) {
        FootprintInfo fp = new FootprintInfo();
        Position agentPos = state.getAgentPosition(agentId);
        Color agentColor = level.getAgentColor(agentId);
        fp.cells.add(agentPos);
        
        // Agent goal
        Position agentGoal = level.getAgentGoalPositionMap().get(agentId);
        if (agentGoal != null) {
            fp.cells.add(agentGoal);
            addBfsPathCells(agentPos, agentGoal, level, fp.cells);
        }
        
        // Owned boxes: same color, has unsatisfied same-color goal
        for (Map.Entry<Position, Character> e : state.getBoxes().entrySet()) {
            Position boxPos = e.getKey();
            char boxType = e.getValue();
            if (level.getBoxColor(boxType) != agentColor) continue;
            
            // Find an unsatisfied goal of this type
            List<Position> goals = level.getBoxGoalsByType().get(boxType);
            if (goals == null) continue;
            Position targetGoal = null;
            for (Position g : goals) {
                Character at = state.getBoxes().get(g);
                if (at == null || at != boxType) {
                    targetGoal = g;
                    break;
                }
            }
            if (targetGoal == null) continue; // box already on a goal of its type
            if (boxPos.equals(targetGoal)) continue;
            
            fp.boxPositions.add(boxPos);
            fp.goalPositions.add(targetGoal);
            fp.cells.add(boxPos);
            fp.cells.add(targetGoal);
            addBfsPathCells(agentPos, boxPos, level, fp.cells);
            addBfsPathCells(boxPos, targetGoal, level, fp.cells);
        }
        return fp;
    }
    
    private boolean footprintsCouple(int a, int b,
                                      Map<Integer, Set<Position>> footprints,
                                      Map<Integer, Set<Position>> ownedBoxes,
                                      Map<Integer, Set<Position>> targetGoals,
                                      int tolerance) {
        // (a) shared owned box
        Set<Position> aBoxes = ownedBoxes.get(a);
        for (Position p : ownedBoxes.get(b)) {
            if (aBoxes.contains(p)) return true;
        }
        // (b) shared target goal
        Set<Position> aGoals = targetGoals.get(a);
        for (Position p : targetGoals.get(b)) {
            if (aGoals.contains(p)) return true;
        }
        // (c) footprint overlap > tolerance
        Set<Position> aFp = footprints.get(a);
        Set<Position> bFp = footprints.get(b);
        int overlap = 0;
        for (Position p : (aFp.size() <= bFp.size() ? aFp : bFp)) {
            if ((aFp.size() <= bFp.size() ? bFp : aFp).contains(p)) {
                overlap++;
                if (overlap > tolerance) return true;
            }
        }
        return false;
    }
    
    /**
     * Adds BFS shortest-path cells (walls only — boxes treated as passable so
     * the footprint reflects geometric necessity, not current snapshot).
     * Bounded to 200 cells to avoid blowup on large open maps.
     */
    private void addBfsPathCells(Position start, Position goal, Level level, Set<Position> out) {
        if (start.equals(goal)) return;
        Map<Position, Position> parent = new HashMap<>();
        Queue<Position> queue = new LinkedList<>();
        queue.add(start);
        parent.put(start, start);
        int explored = 0;
        while (!queue.isEmpty() && explored < 5000) {
            Position cur = queue.poll();
            explored++;
            if (cur.equals(goal)) break;
            for (Direction d : Direction.values()) {
                Position nxt = cur.move(d);
                if (level.isWall(nxt) || parent.containsKey(nxt)) continue;
                parent.put(nxt, cur);
                queue.add(nxt);
            }
        }
        if (!parent.containsKey(goal)) return;
        Position p = goal;
        int safety = 0;
        while (!p.equals(start) && safety++ < 200) {
            out.add(p);
            p = parent.get(p);
        }
    }
    
    /**
     * Solves independent groups separately and merges results.
     * Each group gets a projected State+Level with remapped agent IDs (0..k-1),
     * an independent strategy sequence, and its own timeout budget.
     * 
     * Time management: independence detection phase is capped at 50% of total timeout
     * to reserve budget for the full portfolio fallback. Within that cap, time is
     * distributed fairly across remaining groups (remaining / groupsLeft).
     */
    private List<Action[]> solveIndependentGroups(List<List<Integer>> groups,
            State initialState, Level level, long startTime) {
        int numAgents = level.getNumAgents();
        Set<Position> immovable = features != null && features.taskFilter != null 
            ? features.taskFilter.immovableBoxes : Collections.emptySet();
        List<GroupResult> results = new ArrayList<>();
        
        // Cap independence detection at 50% of total timeout, reserving 50% for fallback
        long independenceDeadline = startTime + (long)(timeoutMs * 0.5);
        
        for (int gi = 0; gi < groups.size(); gi++) {
            List<Integer> group = groups.get(gi);
            long remaining = independenceDeadline - System.currentTimeMillis();
            if (remaining <= 0) break;
            
            // Fair share: distribute remaining time evenly across unsolved groups
            int groupsLeft = groups.size() - gi;
            long groupTimeout = remaining / groupsLeft;
            
            // Compute reachable area for this group
            Set<Position> reachableArea = computeReachableArea(group, initialState, level, immovable);
            
            // Project State and Level to this group
            int[] originalIds = group.stream().sorted().mapToInt(Integer::intValue).toArray();
            Map<Integer, Integer> newIdMap = new HashMap<>();
            for (int i = 0; i < originalIds.length; i++) newIdMap.put(originalIds[i], i);
            
            State projState = projectState(originalIds, initialState, reachableArea);
            Level projLevel = projectLevel(originalIds, newIdMap, level, reachableArea);
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] Solving group " + group + " (" + group.size() + 
                    " agents, timeout=" + groupTimeout + "ms)");
            }
            
            // Solve this group independently
            List<Action[]> groupPlan = solveGroup(projState, projLevel, groupTimeout);
            
            // Check if group subproblem was fully solved
            boolean groupSolved = false;
            if (groupPlan != null && !groupPlan.isEmpty()) {
                State finalState = replayPlan(groupPlan, projState, projLevel);
                groupSolved = finalState != null && finalState.isGoalState(projLevel);
            }
            results.add(new GroupResult(originalIds, groupPlan));
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] Group " + group + ": " + 
                    (groupPlan != null ? groupPlan.size() + " steps" + (groupSolved ? " SOLVED" : " PARTIAL") : "FAILED"));
            }
        }
        
        return mergeGroupPlans(results, numAgents);
    }
    
    /**
     * BFS reachability from all agents in the group.
     * Movable boxes are passable, immovable boxes and walls are not.
     */
    private Set<Position> computeReachableArea(List<Integer> agentIds, State state, 
            Level level, Set<Position> immovable) {
        Set<Position> area = new HashSet<>();
        Queue<Position> queue = new LinkedList<>();
        for (int agentId : agentIds) {
            Position start = state.getAgentPosition(agentId);
            if (area.add(start)) queue.add(start);
        }
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                if (level.isWall(next)) continue;
                if (immovable.contains(next)) {
                    area.add(next); // Include boundary immovable box but don't expand through
                    continue;
                }
                if (area.add(next)) {
                    queue.add(next);
                }
            }
        }
        return area;
    }
    
    /**
     * Projects a State to contain only the group's agents (remapped 0..k-1)
     * and boxes within the reachable area.
     */
    private State projectState(int[] originalIds, State state, Set<Position> reachableArea) {
        Position[] newAgentPositions = new Position[originalIds.length];
        for (int i = 0; i < originalIds.length; i++) {
            newAgentPositions[i] = state.getAgentPosition(originalIds[i]);
        }
        Map<Position, Character> newBoxes = new HashMap<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (reachableArea.contains(entry.getKey())) {
                newBoxes.put(entry.getKey(), entry.getValue());
            }
        }
        return new State(newAgentPositions, newBoxes);
    }
    
    /**
     * Projects a Level for an independent group:
     * - Walls: unchanged (immovable boxes are included as boxes in the projected state;
     *   the group's TaskFilter will naturally detect them as immovable)
     * - Box goals: only those in the reachable area
     * - Agent goals: remapped to new IDs, only active agents
     * - Agent colors: remapped to new IDs
     * - Box colors: unchanged (unused types are harmless)
     */
    private Level projectLevel(int[] originalIds, Map<Integer, Integer> newIdMap,
            Level level, Set<Position> reachableArea) {
        int rows = level.getRows();
        int cols = level.getCols();
        
        boolean[][] walls = new boolean[rows][cols];
        char[][] boxGoals = new char[rows][cols];
        int[][] agentGoals = new int[rows][cols];
        
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                walls[r][c] = level.isWall(r, c);
                agentGoals[r][c] = -1; // default: no agent goal
                
                // Keep box goals only in reachable area
                char bg = level.getBoxGoal(r, c);
                if (bg != '\0' && reachableArea.contains(Position.of(r, c))) {
                    boxGoals[r][c] = bg;
                }
                
                // Remap agent goals to new IDs
                int ag = level.getAgentGoal(r, c);
                if (ag >= 0 && newIdMap.containsKey(ag)) {
                    agentGoals[r][c] = newIdMap.get(ag);
                }
            }
        }
        
        Map<Integer, Color> agentColors = new HashMap<>();
        for (int i = 0; i < originalIds.length; i++) {
            agentColors.put(i, level.getAgentColor(originalIds[i]));
        }
        
        return new Level(level.getName() + "_g" + originalIds[0], rows, cols,
                         walls, boxGoals, agentGoals, level.getBoxColors(), agentColors);
    }
    
    /**
     * Runs the full strategy sequence for a projected group subproblem.
     */
    private List<Action[]> solveGroup(State projState, Level projLevel, long timeout) {
        LevelFeatures groupFeatures = LevelAnalyzer.analyze(projLevel, projState);
        List<StrategyConfig> strategies = buildStrategySequence(groupFeatures, projState);
        
        Heuristic heuristic;
        try {
            heuristic = new TrueDistanceHeuristic(projLevel);
        } catch (Exception e) {
            heuristic = new ManhattanHeuristic();
        }
        
        if (SearchConfig.isMinimal()) {
            System.err.println("  Strategy sequence: " + 
                strategies.stream().map(s -> s.type.name() + 
                    (s.orderingMode != null ? "(" + s.orderingMode + 
                        (s.orderingMode == OrderingMode.RANDOM ? "#" + s.randomSeed : "") + ")" : "")).toList());
        }
        
        long groupStart = System.currentTimeMillis();
        long remaining = timeout;
        List<Action[]> bestPartial = null;
        
        mapf.planning.strategy.SubgoalManager groupSubgoalManager = 
            new mapf.planning.strategy.SubgoalManager(heuristic);
        
        for (StrategyConfig sc : strategies) {
            if (remaining <= 0) break;
            long attemptTimeout = Math.min((long)(timeout * sc.timeBudgetFraction), remaining);
            
            SearchConfig strategyConfig = new SearchConfig(
                timeout, this.config.getMaxStates(), sc.weight);
            
            SearchStrategy strategy;
            if (sc.type == StrategyType.SINGLE_AGENT) {
                SingleAgentStrategy sa = new SingleAgentStrategy(heuristic, strategyConfig);
                sa.setWeight(sc.weight);
                strategy = sa;
            } else {
                PriorityPlanningStrategy pp = new PriorityPlanningStrategy(
                    heuristic, strategyConfig, groupSubgoalManager);
                if (groupFeatures.executionOrder != null) pp.setGoalExecutionOrder(groupFeatures.executionOrder);
                if (groupFeatures.goalDependsOn != null) pp.setGoalDependencies(groupFeatures.goalDependsOn);
                if (groupFeatures.taskFilter != null) pp.setImmovableBoxes(groupFeatures.taskFilter.immovableBoxes);
                if (sc.orderingMode != null) pp.setOrderingMode(sc.orderingMode);
                if (sc.orderingMode == OrderingMode.RANDOM && sc.randomSeed != 0) pp.setRandomSeed(sc.randomSeed);
                strategy = pp;
            }
            strategy.setTimeout(attemptTimeout);
            
            List<Action[]> result = null;
            try {
                result = strategy.search(projState, projLevel);
            } catch (Exception e) {
                System.err.println("[Portfolio] Group strategy " + sc.type + " threw: " + e.getMessage());
            }
            
            if (result != null && !result.isEmpty()) {
                State finalState = replayPlan(result, projState, projLevel);
                if (finalState != null && finalState.isGoalState(projLevel)) {
                    return result;
                }
                if (bestPartial == null || result.size() > bestPartial.size()) {
                    bestPartial = result;
                }
            }
            
            remaining = timeout - (System.currentTimeMillis() - groupStart);
        }
        
        return bestPartial;
    }
    
    /**
     * Merges per-group plans into a single joint action plan.
     * Each group's actions are remapped from projected agent IDs (0..k-1) back to
     * original agent IDs. Non-active agents get NoOp at each timestep.
     */
    private List<Action[]> mergeGroupPlans(List<GroupResult> results, int numAgents) {
        int maxLen = 0;
        for (GroupResult gr : results) {
            if (gr.plan != null) maxLen = Math.max(maxLen, gr.plan.size());
        }
        if (maxLen == 0) return null;
        
        List<Action[]> merged = new ArrayList<>(maxLen);
        for (int t = 0; t < maxLen; t++) {
            Action[] joint = new Action[numAgents];
            Arrays.fill(joint, Action.NOOP);
            for (GroupResult gr : results) {
                if (gr.plan != null && t < gr.plan.size()) {
                    Action[] groupAction = gr.plan.get(t);
                    for (int newId = 0; newId < gr.originalIds.length; newId++) {
                        int origId = gr.originalIds[newId];
                        if (newId < groupAction.length && groupAction[newId] != null) {
                            joint[origId] = groupAction[newId];
                        }
                    }
                }
            }
            merged.add(joint);
        }
        return merged;
    }
    
    private static class GroupResult {
        final int[] originalIds;
        final List<Action[]> plan;
        GroupResult(int[] originalIds, List<Action[]> plan) {
            this.originalIds = originalIds;
            this.plan = plan;
        }
    }
    
    // ========== CBSR Methods ==========

    /**
     * CBSR: Derive ordering nogoods from a PP failure report.
     * <p>
     * Rule: if PP was stuck on goal X (agent A) with other same-agent box goals still
     * unsatisfied, then X must come AFTER those goals (they are "required" before X).
     * <p>
     * Rationale for ISO: agent 8 (GREY) tries to deliver I@(10,43) first, but its
     * corridor is blocked by L boxes that only agent 8 can move. Delivering L's real
     * goals first would clear the corridor. The TRAP signal confirms this: parking L
     * would trap agent 8 from reaching its remaining same-color tasks.
     */
    private List<OrderingNogood> learnNogoodsFromReport(mapf.planning.signal.FailureReport report) {
        if (report == null || report.lastAttemptedSubgoal == null) return Collections.emptyList();
        // Only learn from genuine "stuck" or "partial" failures, not budget exhaustion.
        if (report.kind != mapf.planning.signal.FailureReport.Kind.STUCK_NO_PROGRESS
                && report.kind != mapf.planning.signal.FailureReport.Kind.PARTIAL_PLAN) {
            return Collections.emptyList();
        }
        PriorityPlanningStrategy.Subgoal failed = report.lastAttemptedSubgoal;
        if (failed.isAgentGoal || failed.goalPos == null) return Collections.emptyList();

        int agentId = failed.agentId;
        Set<Position> required = new LinkedHashSet<>();
        for (PriorityPlanningStrategy.Subgoal sg : report.unsatisfiedAtFailure) {
            if (sg.agentId == agentId && !sg.isAgentGoal
                    && sg.goalPos != null && !sg.goalPos.equals(failed.goalPos)) {
                required.add(sg.goalPos);
            }
        }

        if (required.isEmpty()) return Collections.emptyList();

        OrderingNogood nogood = new OrderingNogood(required, failed.goalPos);
        System.err.println("[CBSR] Learned nogood: " + nogood + " reason=" + report.kind);
        return Collections.singletonList(nogood);
    }

    /**
     * CBSR: Deduplicate nogoods with the same forbidden goal by merging required sets.
     */
    private List<OrderingNogood> deduplicateNogoods(List<OrderingNogood> nogoods) {
        if (nogoods.size() <= 1) return nogoods;
        Map<Position, Set<Position>> merged = new LinkedHashMap<>();
        for (OrderingNogood ng : nogoods) {
            merged.computeIfAbsent(ng.forbidden, k -> new LinkedHashSet<>()).addAll(ng.required);
        }
        List<OrderingNogood> result = new ArrayList<>(merged.size());
        for (Map.Entry<Position, Set<Position>> e : merged.entrySet()) {
            result.add(new OrderingNogood(e.getValue(), e.getKey()));
        }
        return result;
    }

    /**
     * CBSR: Repair a goal ordering so that all nogoods are satisfied.
     * <p>
     * For each nogood {required, forbidden}: if `forbidden` appears before any position
     * in `required`, move `forbidden` to just after the last `required` position.
     * Iterates until stable (fixed-point). O(N * M * K) where N = ordering size,
     * M = nogood count, K = required set size.
     */
    private List<Position> applyNogoodsToOrdering(List<Position> baseOrdering,
            List<OrderingNogood> nogoods) {
        if (nogoods.isEmpty()) return new ArrayList<>(baseOrdering);
        List<Position> ordering = new ArrayList<>(baseOrdering);
        boolean changed = true;
        int maxIter = nogoods.size() * 2 + 5;
        while (changed && maxIter-- > 0) {
            changed = false;
            for (OrderingNogood ng : nogoods) {
                int forbIdx = ordering.indexOf(ng.forbidden);
                if (forbIdx < 0) continue;  // forbidden not in this ordering, skip
                int lastReqIdx = -1;
                for (Position req : ng.required) {
                    int ri = ordering.indexOf(req);
                    if (ri > lastReqIdx) lastReqIdx = ri;
                }
                if (lastReqIdx >= 0 && lastReqIdx > forbIdx) {
                    // Violation: forbidden appears before some required goal.
                    // Move forbidden to just after the last required.
                    ordering.remove(forbIdx);
                    // lastReqIdx shifted by -1 because we removed an element before it
                    int insertAt = Math.min(lastReqIdx, ordering.size());
                    ordering.add(insertAt, ng.forbidden);
                    changed = true;
                }
            }
        }
        return ordering;
    }

    // ========== Helper Classes ==========
    
    private static class StrategyConfig {
        final StrategyType type;
        final double weight;
        final OrderingMode orderingMode;
        final int randomSeed;
        final double timeBudgetFraction;
        
        StrategyConfig(StrategyType type, double weight, OrderingMode orderingMode, int randomSeed, double timeBudgetFraction) {
            this.type = type;
            this.weight = weight;
            this.orderingMode = orderingMode;
            this.randomSeed = randomSeed;
            this.timeBudgetFraction = timeBudgetFraction;
        }
    }
    
    private static class AttemptRecord {
        final StrategyType strategy;
        final OrderingMode orderingMode;   // null for non-PP (e.g. SINGLE_AGENT)
        final int randomSeed;              // 0 if not RANDOM
        final long durationMs;
        final boolean success;
        final int planSteps;               // plan length returned by strategy (0 if null)
        final int unsatCount;              // unsatisfied goals at failure (-1 if unknown)
        final String failedSubgoal;        // "agent7->K@(34,31)" or null
        final String failureKind;          // SUCCESS / PARTIAL_PLAN / STUCK_NO_PROGRESS / EXCEPTION / NONE
        final int reliefCount;             // NAMO reliefs synthesized THIS round
        final int suspendedCount;          // suspended box-goals after THIS round
        
        AttemptRecord(StrategyType strategy, OrderingMode orderingMode, int randomSeed,
                      long durationMs, boolean success, int planSteps, int unsatCount,
                      String failedSubgoal, String failureKind,
                      int reliefCount, int suspendedCount) {
            this.strategy = strategy;
            this.orderingMode = orderingMode;
            this.randomSeed = randomSeed;
            this.durationMs = durationMs;
            this.success = success;
            this.planSteps = planSteps;
            this.unsatCount = unsatCount;
            this.failedSubgoal = failedSubgoal;
            this.failureKind = failureKind;
            this.reliefCount = reliefCount;
            this.suspendedCount = suspendedCount;
        }
    }

    /**
     * CBSR: An ordering constraint (nogood) learned from a PP failure.
     * Encodes: "the `forbidden` goal must not be attempted until ALL positions in
     * `required` have been completed." Used by {@link #applyNogoodsToOrdering} to
     * repair a topological ordering so that the next PP iteration avoids the same
     * ordering mistake.
     */
    private static class OrderingNogood {
        /** All of these goal positions must be completed before `forbidden` is attempted. */
        final Set<Position> required;
        /** This goal position may not be attempted until all of `required` are done. */
        final Position forbidden;

        OrderingNogood(Set<Position> required, Position forbidden) {
            this.required = Collections.unmodifiableSet(new LinkedHashSet<>(required));
            this.forbidden = forbidden;
        }

        @Override
        public String toString() {
            return "{required=" + required + ", forbidden=" + forbidden + "}";
        }
    }
}
