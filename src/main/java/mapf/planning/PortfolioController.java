package mapf.planning;

import mapf.domain.*;
import mapf.planning.analysis.LevelAnalyzer;
import mapf.planning.analysis.LevelAnalyzer.LevelFeatures;
import mapf.planning.analysis.LevelAnalyzer.StrategyType;
import mapf.planning.cbs.CBSStrategy;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.planning.strategy.JointAStarStrategy;
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
        
        // Step 2: Build strategy sequence based on analysis
        List<StrategyConfig> strategies = buildStrategySequence(features, initialState);
        
        if (SearchConfig.isMinimal()) {
            System.err.println("[Portfolio] Strategy sequence: " + 
                strategies.stream().map(s -> s.type.name() + 
                    (s.orderingMode != null ? "(" + s.orderingMode + ")" : "")).toList());
        }
        
        // Step 3: Try strategies in sequence
        for (StrategyConfig strategyConfig : strategies) {
            if (remainingTime <= 0) {
                System.err.println("[Portfolio] Timeout - no more time for attempts");
                break;
            }
            
            // Allocate time for this attempt
            long attemptTimeout = computeAttemptTimeout(strategyConfig, remainingTime, strategies.size());
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] Trying " + strategyConfig.type + 
                    (strategyConfig.orderingMode != null ? "(" + strategyConfig.orderingMode + ")" : "") +
                    " (timeout=" + attemptTimeout + "ms, budget=" + 
                    String.format("%.0f%%", strategyConfig.timeBudgetFraction * 100) + ")");
            }
            
            // Create and configure strategy
            SearchStrategy strategy = createStrategy(strategyConfig, level);
            strategy.setTimeout(attemptTimeout);
            
            // Execute
            long attemptStart = System.currentTimeMillis();
            List<Action[]> result = strategy.search(initialState, level);
            long attemptDuration = System.currentTimeMillis() - attemptStart;
            
            // Record attempt
            attempts.add(new AttemptRecord(strategyConfig.type, attemptDuration, 
                                          result != null && !result.isEmpty()));
            
            if (result != null && !result.isEmpty()) {
                if (SearchConfig.isMinimal()) {
                    System.err.println("[Portfolio] SUCCESS with " + strategyConfig.type + 
                        " (" + result.size() + " actions, " + attemptDuration + "ms)");
                }
                return result;
            }
            
            if (SearchConfig.isMinimal()) {
                System.err.println("[Portfolio] " + strategyConfig.type + " failed after " + attemptDuration + "ms");
            }
            
            // Update remaining time
            remainingTime = timeoutMs - (System.currentTimeMillis() - startTime);
        }
        
        System.err.println("[Portfolio] All strategies failed");
        printAttemptSummary();
        return null;
    }
    
    /**
     * Builds strategy sequence based on level features.
     * Multi-agent levels get PP with different ordering modes as the real "retry knob".
     * Time budget: TOPOLOGICAL 40%, REVERSE 25%, DISTANCE_GREEDY 20%, RANDOM 15%.
     */
    private List<StrategyConfig> buildStrategySequence(LevelFeatures f, State state) {
        List<StrategyConfig> strategies = new ArrayList<>();
        
        if (f.recommendedStrategy == StrategyType.SINGLE_AGENT) {
            // Simple case: just A* with increasing weights
            strategies.add(new StrategyConfig(StrategyType.SINGLE_AGENT, 1.0, null, 0.40));
            strategies.add(new StrategyConfig(StrategyType.SINGLE_AGENT, 5.0, null, 0.60));
        } else {
            // Multi-agent: PP with different ordering modes
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.TOPOLOGICAL, 0.40));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.REVERSE_TOPOLOGICAL, 0.25));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.DISTANCE_GREEDY, 0.20));
            strategies.add(new StrategyConfig(StrategyType.STRICT_ORDER, 1.0, OrderingMode.RANDOM, 0.15));
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
                
            case CBS:
                CBSStrategy cbs = new CBSStrategy(heuristic, strategyConfig);
                return cbs;
                
            case JOINT_SEARCH:
                JointAStarStrategy jointAStar = new JointAStarStrategy(heuristic, strategyConfig);
                jointAStar.setWeight(config.weight);
                return jointAStar;
                
            case STRICT_ORDER:
            case CYCLE_BREAKER:
            case GREEDY_WITH_RETRY:
            default:
                // All use PriorityPlanningStrategy with different ordering modes
                PriorityPlanningStrategy priorityPlanning = new PriorityPlanningStrategy(heuristic, strategyConfig);
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
        for (AttemptRecord record : attempts) {
            System.err.println(String.format("  %s: %dms, %s",
                record.strategy, record.durationMs, record.success ? "SUCCESS" : "FAILED"));
        }
    }
    
    /**
     * Returns the analysis result (for debugging/testing).
     */
    public LevelFeatures getFeatures() {
        return features;
    }
    
    // ========== Helper Classes ==========
    
    private static class StrategyConfig {
        final StrategyType type;
        final double weight;
        final OrderingMode orderingMode;
        final double timeBudgetFraction;
        
        StrategyConfig(StrategyType type, double weight, OrderingMode orderingMode, double timeBudgetFraction) {
            this.type = type;
            this.weight = weight;
            this.orderingMode = orderingMode;
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
