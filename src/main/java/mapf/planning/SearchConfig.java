package mapf.planning;

/**
 * Configuration for search algorithms.
 * Centralizes all configurable parameters to avoid hardcoding.
 */
public class SearchConfig {
    
    /** Default timeout for search (3 minutes as per PRODUCT.md) */
    public static final long DEFAULT_TIMEOUT_MS = 180_000;
    
    /** Default maximum states to explore */
    public static final int DEFAULT_MAX_STATES = 1_000_000;
    
    /** Threshold for switching from JointAStar to PriorityPlanning */
    public static final int JOINT_ASTAR_AGENT_THRESHOLD = 3;
    
    /** Weight for Weighted A* (1.0 = standard A*, higher = faster but less optimal) */
    public static final double DEFAULT_ASTAR_WEIGHT = 1.0;
    
    /** Weight for fallback Weighted A* */
    public static final double FALLBACK_ASTAR_WEIGHT = 1.5;
    
    /** Timeout for first attempt before trying fallback (60 seconds) */
    public static final long FIRST_ATTEMPT_TIMEOUT_MS = 60_000;
    
    /** Timeout for fallback attempt */
    public static final long FALLBACK_TIMEOUT_MS = 60_000;
    
    /** Maximum joint actions allowed (as per PRODUCT.md) */
    public static final int MAX_ACTIONS = 20_000;
    
    /** Maximum number of agents supported (0-9 as per PRODUCT.md) */
    public static final int MAX_AGENTS = 10;
    
    /** Maximum number of box types (A-Z as per PRODUCT.md) */
    public static final int MAX_BOX_TYPES = 26;
    
    /** Work score for a box not at its goal (used in priority planning) */
    public static final int BOX_NOT_AT_GOAL_WORK_SCORE = 10;
    
    /** Progress logging interval (log every N states) */
    public static final int PROGRESS_LOG_INTERVAL = 10_000;
    
    /** Maximum states per single-subgoal search in Priority Planning */
    public static final int MAX_STATES_PER_SUBGOAL = 50_000;
    
    /** Number of random re-orderings to try when stuck in Priority Planning */
    public static final int MAX_REORDER_ATTEMPTS = 5;
    
    // Instance configuration
    private long timeoutMs = DEFAULT_TIMEOUT_MS;
    private int maxStates = DEFAULT_MAX_STATES;
    private double astarWeight = DEFAULT_ASTAR_WEIGHT;
    private boolean useGreedyFallback = true;
    
    public SearchConfig() {}
    
    public SearchConfig(long timeoutMs, int maxStates, double astarWeight) {
        this.timeoutMs = timeoutMs;
        this.maxStates = maxStates;
        this.astarWeight = astarWeight;
    }
    
    /**
     * Creates a SearchConfig with default values.
     * Factory method for cleaner API.
     */
    public static SearchConfig defaults() {
        return new SearchConfig();
    }
    
    public long getTimeoutMs() { return timeoutMs; }
    public void setTimeoutMs(long timeoutMs) { this.timeoutMs = timeoutMs; }
    
    public int getMaxStates() { return maxStates; }
    public void setMaxStates(int maxStates) { this.maxStates = maxStates; }
    
    public double getAstarWeight() { return astarWeight; }
    public void setAstarWeight(double astarWeight) { this.astarWeight = astarWeight; }
    
    public boolean isUseGreedyFallback() { return useGreedyFallback; }
    public void setUseGreedyFallback(boolean useGreedyFallback) { this.useGreedyFallback = useGreedyFallback; }
}
