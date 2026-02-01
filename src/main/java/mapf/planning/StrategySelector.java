package mapf.planning;

import mapf.domain.*;
import mapf.planning.cbs.CBSStrategy;

/**
 * Factory for selecting appropriate search strategy based on level characteristics.
 * Implements the recommendation from ARCHITECTURE.md:
 * "Start with Option A (independent planning), upgrade to Option B or hybrid approach"
 */
public class StrategySelector {
    
    private final Heuristic heuristic;
    private final SearchConfig config;
    
    public StrategySelector(Heuristic heuristic) {
        this(heuristic, new SearchConfig());
    }
    
    public StrategySelector(Heuristic heuristic, SearchConfig config) {
        this.heuristic = heuristic;
        this.config = config;
    }
    
    /**
     * Selects the most appropriate strategy based on level analysis.
     * 
     * Strategy selection logic:
     * - 1 agent: SingleAgentStrategy (wraps AStar)
     * - 2-3 agents: JointAStarStrategy (optimal, manageable state space)
     * - 4+ agents: PriorityPlanningStrategy (scalable, may be suboptimal)
     * 
     * @param level the level to analyze
     * @param initialState the initial state
     * @return the selected search strategy
     */
    public SearchStrategy selectStrategy(Level level, State initialState) {
        int numAgents = initialState.getNumAgents();
        
        System.err.println("StrategySelector: " + numAgents + " agents detected");
        
        if (numAgents <= 1) {
            System.err.println("StrategySelector: Using SingleAgentStrategy");
            return new SingleAgentStrategy(heuristic, config);
        } else if (numAgents <= SearchConfig.JOINT_ASTAR_AGENT_THRESHOLD) {
            System.err.println("StrategySelector: Using JointAStarStrategy");
            return new JointAStarStrategy(heuristic, config);
        } else {
            System.err.println("StrategySelector: Using PriorityPlanningStrategy (many agents)");
            return new PriorityPlanningStrategy(heuristic, config);
        }
    }
    
    /**
     * Creates a CBS strategy for handling cyclic dependencies.
     * Called when PriorityPlanningStrategy detects a cyclic dependency.
     * 
     * @return CBS strategy instance
     */
    public CBSStrategy createCBSStrategy() {
        System.err.println("StrategySelector: Creating CBS strategy for cyclic dependency resolution");
        return new CBSStrategy(heuristic, config);
    }
    
    /**
     * Analyzes level complexity to help with strategy tuning.
     */
    public static class LevelAnalysis {
        public final int numAgents;
        public final int numBoxes;
        public final int gridSize;
        public final double density; // (agents + boxes) / free cells
        
        public LevelAnalysis(Level level, State state) {
            this.numAgents = state.getNumAgents();
            this.numBoxes = state.getBoxes().size();
            this.gridSize = level.getRows() * level.getCols();
            
            // Count free cells
            int freeCells = 0;
            for (int r = 0; r < level.getRows(); r++) {
                for (int c = 0; c < level.getCols(); c++) {
                    if (!level.isWall(r, c)) freeCells++;
                }
            }
            this.density = (double)(numAgents + numBoxes) / Math.max(1, freeCells);
        }
        
        @Override
        public String toString() {
            return String.format("LevelAnalysis{agents=%d, boxes=%d, gridSize=%d, density=%.3f}",
                    numAgents, numBoxes, gridSize, density);
        }
    }
}
