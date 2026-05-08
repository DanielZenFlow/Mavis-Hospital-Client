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

        // P4 (conflict-driven PP): accumulate blocker goals derived from prior
        // FailureReports. A blocker is an unsatisfied goal G' such that the failed
        // subgoal G has G in goalDependsOn[G'] (i.e. G' must come before G).
        // PP promotes these to the front of the next attempt's order.
        // Per claudeopus47.txt 3.2.2 (subgoal-level reorder is the cheapest CBS-lite
        // repair) and qanda.txt 5.2 (failure-signal-driven repair).
        Set<Position> prioritizedGoals = new HashSet<>();

        // P1: synthetic escape subgoals to break 2-cycles. Computed lazily once,
        // when the first PP attempt has failed on a level with hasCircularDependency.
        // Per qanda.txt 3 / claudeopus47.txt 3.2.2.
        List<mapf.planning.strategy.PriorityPlanningStrategy.Subgoal> escapeSubgoals = null;

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
            if (!prioritizedGoals.isEmpty() && strategy instanceof PriorityPlanningStrategy) {
                ((PriorityPlanningStrategy) strategy).setPrioritizedGoals(prioritizedGoals);
            }

            // P1: pass escape subgoals (synthesized lazily on first failure for cyclic levels).
            if (escapeSubgoals != null && !escapeSubgoals.isEmpty()
                    && strategy instanceof PriorityPlanningStrategy) {
                ((PriorityPlanningStrategy) strategy).setEscapeSubgoals(escapeSubgoals);
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
            if (strategy instanceof PriorityPlanningStrategy) {
                mapf.planning.signal.FailureReport report =
                        ((PriorityPlanningStrategy) strategy).getLastFailureReport();
                if (report != null) {
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
                }
            }

            // Record attempt
            attempts.add(new AttemptRecord(strategyConfig.type, attemptDuration, 
                                          result != null && !result.isEmpty()));
            
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
        
        if (bestPartialPlan != null) {
            System.err.println("[Portfolio] No full solution found. Returning best partial plan (" 
                + bestPartialPlan.size() + " steps)");
            printAttemptSummary();
            return bestPartialPlan;
        }
        
        System.err.println("[Portfolio] All strategies failed");
        printAttemptSummary();
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
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.40));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.35));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 42, 0.25));
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
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_FARTHEST, 0, 0.40));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.30));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 42, 0.15));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 137, 0.15));
            } else {
                // Open-room cyclic topology (low corridor ratio): TOPO uses partial
                // ordering from dependency analysis (valuable even with cycles).
                // GREEDY nearest-first is a good fallback for coordination-heavy levels.
                System.err.println("[Portfolio] Cyclic + open topology (corridorRatio=" + 
                    String.format("%.2f", f.corridorRatio) + ") → TOPO-first");
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.35));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.35));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 42, 0.15));
                strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 137, 0.15));
            }
        } else {
            // No cycles: topological order is well-founded, give it the most budget.
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0, 0.40));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0, 0.30));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 42, 0.30));
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
    
    private void printAttemptSummary() {
        if (!SearchConfig.isMinimal()) return;
        
        System.err.println("\n=== Portfolio Attempt Summary ===");
        int successAttempt = -1;
        int ppAttempts = 0;
        for (int i = 0; i < attempts.size(); i++) {
            AttemptRecord record = attempts.get(i);
            System.err.println(String.format("  [%d] %s: %dms, %s",
                i + 1, record.strategy, record.durationMs, record.success ? "SUCCESS" : "FAILED"));
            if (record.success && successAttempt < 0) successAttempt = i + 1;
            ppAttempts++;
        }
        if (successAttempt > 0) {
            System.err.println("[Portfolio][DIAG] Solved on attempt " + successAttempt + "/" + ppAttempts
                    + (successAttempt == 1 ? " (no reordering needed)" : " (reordering rescued)"));
        } else {
            System.err.println("[Portfolio][DIAG] All " + ppAttempts + " attempts FAILED — action-layer fix needed");
        }
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
        final long durationMs;
        final boolean success;
        
        AttemptRecord(StrategyType strategy, long durationMs, boolean success) {
            this.strategy = strategy;
            this.durationMs = durationMs;
            this.success = success;
        }
    }
}
