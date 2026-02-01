package mapf.planning.goal;

import mapf.domain.Color;
import mapf.domain.Level;
import mapf.domain.Position;
import mapf.domain.State;

import java.util.*;

/**
 * Extracts goals from Level and State information.
 * 
 * Follows ARCHITECTURE.md: Single Responsibility - only extracts goals.
 * 
 * Key design decisions:
 * 1. Box goals: Assigned to agents of matching color that can reach the box
 * 2. Agent goals: Read directly from Level
 * 3. Phased extraction: Box goals first (Phase 1), then Agent goals (Phase 2)
 * 
 * IMPORTANT insight from domain analysis:
 * Push+Pull means NO dead-ends exist. A box can always be moved back.
 * This eliminates the need for complex dead-end-first ordering.
 */
public class GoalExtractor {
    
    private final Level level;
    
    // Cached goals (computed once, reused)
    private List<Goal> allBoxGoals;
    private List<Goal> allAgentGoals;
    
    // Goal lookup caches
    private Map<Position, Goal> boxGoalByPosition;
    private Map<Integer, Goal> agentGoalByAgentId;
    private Map<Integer, List<Goal>> boxGoalsByAgent;
    
    public GoalExtractor(Level level) {
        this.level = level;
        extractAllGoals();
    }
    
    /**
     * Extracts all goals from the level once during construction.
     */
    private void extractAllGoals() {
        allBoxGoals = new ArrayList<>();
        allAgentGoals = new ArrayList<>();
        boxGoalByPosition = new HashMap<>();
        agentGoalByAgentId = new HashMap<>();
        boxGoalsByAgent = new HashMap<>();
        
        // Initialize per-agent lists
        for (int i = 0; i < level.getNumAgents(); i++) {
            boxGoalsByAgent.put(i, new ArrayList<>());
        }
        
        // Scan the level for goals
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                Position pos = new Position(row, col);
                
                // Check for box goal
                char boxType = level.getBoxGoal(row, col);
                if (boxType != '\0') {
                    // Find an agent that can handle this box type (same color)
                    int assignedAgent = findAgentForBoxType(boxType);
                    if (assignedAgent >= 0) {
                        Goal goal = Goal.boxGoal(assignedAgent, boxType, pos);
                        allBoxGoals.add(goal);
                        boxGoalByPosition.put(pos, goal);
                        boxGoalsByAgent.get(assignedAgent).add(goal);
                    }
                    // Note: If no agent can handle this box type, the level may be unsolvable
                }
                
                // Check for agent goal
                int agentId = level.getAgentGoal(row, col);
                if (agentId >= 0 && agentId < level.getNumAgents()) {
                    Goal goal = Goal.agentGoal(agentId, pos);
                    allAgentGoals.add(goal);
                    agentGoalByAgentId.put(agentId, goal);
                }
            }
        }
    }
    
    /**
     * Finds an agent that can move the given box type (same color).
     * If multiple agents have the same color, returns the lowest-numbered one.
     */
    private int findAgentForBoxType(char boxType) {
        Color boxColor = level.getBoxColor(boxType);
        if (boxColor == null) return -1;
        
        for (int agent = 0; agent < level.getNumAgents(); agent++) {
            Color agentColor = level.getAgentColor(agent);
            if (boxColor.equals(agentColor)) {
                return agent;
            }
        }
        return -1;
    }
    
    /**
     * Gets all unsatisfied goals for the current state.
     * Returns box goals first (Phase 1), then agent goals (Phase 2).
     * 
     * @param state Current game state
     * @param includeAgentGoals If false, only returns box goals
     * @return List of unsatisfied goals
     */
    public List<Goal> getUnsatisfiedGoals(State state, boolean includeAgentGoals) {
        List<Goal> unsatisfied = new ArrayList<>();
        
        // Phase 1: Unsatisfied box goals
        for (Goal boxGoal : allBoxGoals) {
            if (!isBoxGoalSatisfied(boxGoal, state)) {
                unsatisfied.add(boxGoal);
            }
        }
        
        // Phase 2: Unsatisfied agent goals (only if includeAgentGoals is true)
        if (includeAgentGoals) {
            for (Goal agentGoal : allAgentGoals) {
                if (!isAgentGoalSatisfied(agentGoal, state)) {
                    unsatisfied.add(agentGoal);
                }
            }
        }
        
        return unsatisfied;
    }
    
    /**
     * SMART Phase extraction:
     * - Returns box goals if any exist
     * - Once all box goals done, returns agent goals
     * - If an agent has completed all its box goals, also returns its agent goal early
     */
    public List<Goal> getUnsatisfiedGoalsSmart(State state) {
        List<Goal> unsatisfied = new ArrayList<>();
        
        // Track which agents still have box work
        Set<Integer> agentsWithBoxWork = new HashSet<>();
        
        // Phase 1: Unsatisfied box goals
        for (Goal boxGoal : allBoxGoals) {
            if (!isBoxGoalSatisfied(boxGoal, state)) {
                unsatisfied.add(boxGoal);
                agentsWithBoxWork.add(boxGoal.agentId);
            }
        }
        
        // Phase 2: Agent goals for agents done with boxes
        for (Goal agentGoal : allAgentGoals) {
            if (!isAgentGoalSatisfied(agentGoal, state)) {
                // If agent has no more box work OR no box goals at all exist, add agent goal
                if (!agentsWithBoxWork.contains(agentGoal.agentId)) {
                    unsatisfied.add(agentGoal);
                }
            }
        }
        
        return unsatisfied;
    }
    
    /**
     * Check if a box goal is satisfied: correct box type at goal position.
     */
    public boolean isBoxGoalSatisfied(Goal goal, State state) {
        if (!goal.isBoxGoal()) return false;
        char boxAtPos = state.getBoxAt(goal.position);
        return boxAtPos == goal.boxType;
    }
    
    /**
     * Check if an agent goal is satisfied: agent at goal position.
     */
    public boolean isAgentGoalSatisfied(Goal goal, State state) {
        if (!goal.isAgentGoal()) return false;
        Position agentPos = state.getAgentPosition(goal.agentId);
        return goal.position.equals(agentPos);
    }
    
    /**
     * Gets all box goals assigned to a specific agent.
     */
    public List<Goal> getBoxGoalsForAgent(int agentId) {
        return Collections.unmodifiableList(
            boxGoalsByAgent.getOrDefault(agentId, Collections.emptyList())
        );
    }
    
    /**
     * Gets the agent goal for a specific agent, if any.
     */
    public Goal getAgentGoal(int agentId) {
        return agentGoalByAgentId.get(agentId);
    }
    
    /**
     * Gets the box goal at a specific position, if any.
     */
    public Goal getBoxGoalAtPosition(Position pos) {
        return boxGoalByPosition.get(pos);
    }
    
    /**
     * Checks if all goals are satisfied.
     */
    public boolean areAllGoalsSatisfied(State state) {
        for (Goal goal : allBoxGoals) {
            if (!isBoxGoalSatisfied(goal, state)) return false;
        }
        for (Goal goal : allAgentGoals) {
            if (!isAgentGoalSatisfied(goal, state)) return false;
        }
        return true;
    }
    
    /**
     * @return Total number of box goals
     */
    public int getBoxGoalCount() {
        return allBoxGoals.size();
    }
    
    /**
     * @return Total number of agent goals
     */
    public int getAgentGoalCount() {
        return allAgentGoals.size();
    }
    
    /**
     * @return All box goals (unmodifiable)
     */
    public List<Goal> getAllBoxGoals() {
        return Collections.unmodifiableList(allBoxGoals);
    }
    
    /**
     * @return All agent goals (unmodifiable)
     */
    public List<Goal> getAllAgentGoals() {
        return Collections.unmodifiableList(allAgentGoals);
    }
}
