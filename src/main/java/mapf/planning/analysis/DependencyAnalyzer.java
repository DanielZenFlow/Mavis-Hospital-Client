package mapf.planning.analysis;

import mapf.domain.*;
import java.util.*;

/**
 * Analyzes dependencies between agents and detects cyclic dependencies.
 * 
 * This is a diagnostic tool used when Priority Planning gets stuck.
 * It helps identify WHY the planner is stuck by finding dependency cycles
 * like: Agent 0 waits for Agent 1, Agent 1 waits for Agent 2, Agent 2 waits for Agent 0.
 * 
 * Key concepts:
 * - Dependency: Agent A "depends on" Agent B if A cannot proceed until B moves
 * - Cyclic dependency: A circular chain of dependencies (A→B→C→A)
 * - When cyclic dependencies exist, Priority Planning cannot solve the problem
 *   because no matter which agent goes first, it's blocked by another
 */
public class DependencyAnalyzer {
    
    // ========== Result Data Structure ==========
    
    /**
     * Result of dependency analysis
     */
    public static class AnalysisResult {
        /** Dependency graph: key depends on all agents in the value set */
        public final Map<Integer, Set<Integer>> dependencies;
        
        /** All detected cycles (each cycle is a list of agent IDs) */
        public final List<List<Integer>> cycles;
        
        /** Whether any cyclic dependency exists */
        public final boolean hasCycle;
        
        /** Human-readable report */
        public final String report;
        
        /** Detailed dependency reasons for debugging */
        public final Map<String, String> dependencyReasons;
        
        public AnalysisResult(Map<Integer, Set<Integer>> dependencies,
                             List<List<Integer>> cycles,
                             Map<String, String> reasons) {
            this.dependencies = dependencies;
            this.cycles = cycles;
            this.hasCycle = !cycles.isEmpty();
            this.dependencyReasons = reasons;
            this.report = generateReport(dependencies, cycles, reasons);
        }
        
        private static String generateReport(Map<Integer, Set<Integer>> deps,
                                            List<List<Integer>> cycles,
                                            Map<String, String> reasons) {
            StringBuilder sb = new StringBuilder();
            sb.append("=== Dependency Analysis Report ===\n");
            
            // Report dependencies
            sb.append("\nDependencies:\n");
            if (deps.isEmpty()) {
                sb.append("  (none)\n");
            } else {
                for (Map.Entry<Integer, Set<Integer>> entry : deps.entrySet()) {
                    int agent = entry.getKey();
                    for (int dependency : entry.getValue()) {
                        String key = agent + "->" + dependency;
                        String reason = reasons.getOrDefault(key, "unknown");
                        sb.append(String.format("  Agent %d depends on Agent %d: %s\n",
                                agent, dependency, reason));
                    }
                }
            }
            
            // Report cycles
            sb.append("\nCyclic Dependencies:\n");
            if (cycles.isEmpty()) {
                sb.append("  (none detected)\n");
            } else {
                for (int i = 0; i < cycles.size(); i++) {
                    List<Integer> cycle = cycles.get(i);
                    sb.append("  Cycle ").append(i + 1).append(": ");
                    for (int j = 0; j < cycle.size(); j++) {
                        sb.append("Agent ").append(cycle.get(j));
                        if (j < cycle.size() - 1) {
                            sb.append(" → ");
                        }
                    }
                    sb.append(" → Agent ").append(cycle.get(0)); // Close the cycle
                    sb.append("\n");
                }
            }
            
            // Summary
            sb.append("\nConclusion: ");
            if (cycles.isEmpty()) {
                sb.append("No cyclic dependencies. Stuck state may be due to other reasons.\n");
            } else {
                sb.append("CYCLIC DEPENDENCY DETECTED! Priority Planning cannot resolve this.\n");
                sb.append("Recommendation: Use CBS or break the cycle by temporary displacement.\n");
            }
            
            return sb.toString();
        }
        
        @Override
        public String toString() {
            return report;
        }
    }
    
    // ========== Main Analysis Method ==========
    
    /**
     * Analyzes agent dependencies in the current state.
     * 
     * @param state Current (simulated) state during planning
     * @param level Level information (walls, goals, colors)
     * @return Analysis result with dependency graph and detected cycles
     */
    public static AnalysisResult analyze(State state, Level level) {
        int numAgents = state.getNumAgents();
        
        // Step 0: Compute immovable boxes (no matching agent color) — treated as walls
        Set<Position> immovableBoxes = computeImmovableBoxes(state, level);
        
        // Step 0b: Precompute per-agent reachable areas (BFS from each agent, immovable boxes = walls)
        Map<Integer, Set<Position>> agentReachable = new HashMap<>();
        for (int i = 0; i < numAgents; i++) {
            agentReachable.put(i, bfsReachable(state.getAgentPosition(i), level, immovableBoxes));
        }
        
        // Step 1: Build dependency graph
        Map<Integer, Set<Integer>> dependencies = new HashMap<>();
        Map<String, String> reasons = new HashMap<>();
        
        for (int i = 0; i < numAgents; i++) {
            dependencies.put(i, new HashSet<>());
        }
        
        // Check all pairs of agents for dependencies
        for (int agentA = 0; agentA < numAgents; agentA++) {
            for (int agentB = 0; agentB < numAgents; agentB++) {
                if (agentA == agentB) continue;
                
                String reason = checkDependency(agentA, agentB, state, level,
                                                immovableBoxes, agentReachable.get(agentA));
                if (reason != null) {
                    dependencies.get(agentA).add(agentB);
                    reasons.put(agentA + "->" + agentB, reason);
                }
            }
        }
        
        // Step 2: Detect cycles using DFS
        List<List<Integer>> cycles = detectCycles(dependencies);
        
        return new AnalysisResult(dependencies, cycles, reasons);
    }
    
    // ========== Dependency Detection ==========
    
    /**
     * Checks if agentA depends on agentB.
     * 
     * Dependency exists when:
     * 1. AgentB is at agentA's goal position
     * 2. AgentB is blocking the only path to agentA's goal
     * 3. AgentB is blocking access to a box that agentA needs to push
     * 
     * @return Reason string if dependency exists, null otherwise
     */
    private static String checkDependency(int agentA, int agentB, State state, Level level,
                                            Set<Position> immovableBoxes, Set<Position> agentAReachable) {
        Position posA = state.getAgentPosition(agentA);
        Position posB = state.getAgentPosition(agentB);
        
        // Case 1: AgentB is at AgentA's goal position (only if goal is in agent's component)
        Position goalA = findAgentGoal(agentA, level);
        if (goalA != null && agentAReachable.contains(goalA) && posB.equals(goalA)) {
            return "Agent " + agentB + " is at Agent " + agentA + "'s goal position " + goalA;
        }
        
        // Case 2: AgentB is blocking the path to AgentA's goal (only if goal reachable)
        if (goalA != null && agentAReachable.contains(goalA)) {
            if (isBlockingPath(posA, goalA, posB, level, immovableBoxes)) {
                return "Agent " + agentB + " is blocking path from " + posA + " to goal " + goalA;
            }
        }
        
        // Case 3: Check if AgentB is blocking access to boxes that AgentA needs
        // CONNECTED-COMPONENT FIX: Only consider boxes AND goals in AgentA's reachable area.
        Color colorA = level.getAgentColor(agentA);
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            char boxType = box.getValue();
            Position boxPos = box.getKey();
            
            // Is this a box that AgentA can push and needs to push?
            if (level.getBoxColor(boxType) != colorA) continue;
            
            // Skip immovable boxes (no agent of matching color — already handled)
            if (immovableBoxes.contains(boxPos)) continue;
            
            // FIX: Box must be adjacent to a cell in AgentA's connected component
            if (!isAdjacentToReachable(boxPos, agentAReachable)) continue;
            
            // FIX: Find unsatisfied goal REACHABLE from this box's connected component
            Position boxGoal = findUnsatisfiedBoxGoalInComponent(boxType, state, level, agentAReachable);
            if (boxGoal != null && !boxPos.equals(boxGoal)) {
                // AgentA needs to push this box
                // Check if AgentB is blocking access to this box
                if (isBlockingPath(posA, boxPos, posB, level, immovableBoxes)) {
                    return "Agent " + agentB + " is blocking access to box " + boxType + " at " + boxPos;
                }
                // Check if AgentB is blocking the box's path to its goal
                if (isBlockingPath(boxPos, boxGoal, posB, level, immovableBoxes)) {
                    return "Agent " + agentB + " is blocking box " + boxType + "'s path to goal " + boxGoal;
                }
            }
        }
        
        return null; // No dependency
    }
    
    /**
     * Finds the goal position for an agent, if any.
     */
    private static Position findAgentGoal(int agentId, Level level) {
        return level.getAgentGoalPositionMap().get(agentId);
    }
    
    /**
     * Finds an unsatisfied goal position for a box type (legacy, no component filter).
     */
    private static Position findUnsatisfiedBoxGoal(char boxType, State state, Level level) {
        return findUnsatisfiedBoxGoalInComponent(boxType, state, level, null);
    }
    
    /**
     * Finds an unsatisfied goal for a box type, restricted to the given connected component.
     * If reachableComponent is null, no restriction is applied (backward compatible).
     *
     * @param boxType       The box type character
     * @param state         Current state
     * @param level         Level definition
     * @param reachableComponent Positions reachable by the servicing agent (null = no filter)
     * @return An unsatisfied goal Position in the component, or null
     */
    private static Position findUnsatisfiedBoxGoalInComponent(char boxType, State state, Level level,
                                                              Set<Position> reachableComponent) {
        List<Position> goals = level.getBoxGoalsByType().get(boxType);
        if (goals == null) return null;
        for (Position goalPos : goals) {
            // FIX: Skip goals outside the agent's connected component
            if (reachableComponent != null && !reachableComponent.contains(goalPos)) continue;
            
            Character currentBox = state.getBoxes().get(goalPos);
            if (currentBox == null || currentBox != boxType) {
                return goalPos; // This goal is unsatisfied and in-component
            }
        }
        return null;
    }
    
    /**
     * Checks if a position is adjacent to any cell in the reachable set.
     * Used to determine if a box can be interacted with by an agent in that component.
     */
    private static boolean isAdjacentToReachable(Position pos, Set<Position> reachable) {
        for (Direction dir : Direction.values()) {
            if (reachable.contains(pos.move(dir))) return true;
        }
        return false;
    }
    
    /**
     * Checks if blocker position is on the critical path from start to goal.
     * Uses BFS considering immovable boxes as permanent walls.
     *
     * CONNECTED-COMPONENT FIX: Immovable boxes are now treated as walls so that
     * two positions separated by an immovable box are correctly identified as
     * being in different connected components rather than falsely connected.
     */
    private static boolean isBlockingPath(Position start, Position goal, Position blocker,
                                          Level level, Set<Position> immovableBoxes) {
        if (start.equals(goal)) return false;
        if (start.equals(blocker) || goal.equals(blocker)) return false;
        
        // BFS to check if we can reach goal without going through blocker
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        queue.add(start);
        visited.add(start);
        visited.add(blocker); // Treat blocker as a wall
        
        while (!queue.isEmpty()) {
            Position current = queue.poll();
            
            if (current.equals(goal)) {
                return false; // Found a path that doesn't go through blocker
            }
            
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                if (immovableBoxes.contains(next)) continue; // FIX: immovable boxes = walls
                
                visited.add(next);
                queue.add(next);
            }
        }
        
        // Couldn't reach goal without going through blocker
        return true;
    }
    
    // ========== Connected-Component Helpers ==========
    
    /**
     * BFS reachability from a start position, treating immovable boxes as walls.
     * Returns the set of all reachable positions (the connected component).
     */
    private static Set<Position> bfsReachable(Position start, Level level, Set<Position> immovableBoxes) {
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
                if (immovableBoxes.contains(next)) continue;
                visited.add(next);
                queue.add(next);
            }
        }
        return visited;
    }
    
    /**
     * Computes the set of immovable box positions (boxes with no agent of matching color).
     * Single Responsibility: this is a pure query with no caching side-effects.
     */
    private static Set<Position> computeImmovableBoxes(State state, Level level) {
        Set<Color> pushableColors = EnumSet.noneOf(Color.class);
        for (int i = 0; i < state.getNumAgents(); i++) {
            pushableColors.add(level.getAgentColor(i));
        }
        
        Set<Position> immovable = new HashSet<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            Color boxColor = level.getBoxColor(entry.getValue());
            if (!pushableColors.contains(boxColor)) {
                immovable.add(entry.getKey());
            }
        }
        return immovable;
    }
    
    // ========== Cycle Detection ==========
    
    /**
     * Detects all cycles in the dependency graph using DFS.
     * 
     * Algorithm:
     * - For each unvisited node, start a DFS
     * - Track nodes in current path (visiting) vs fully processed (visited)
     * - If we encounter a node that's currently being visited, we found a cycle
     */
    private static List<List<Integer>> detectCycles(Map<Integer, Set<Integer>> graph) {
        List<List<Integer>> cycles = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();      // Fully processed nodes
        Set<Integer> inStack = new HashSet<>();      // Nodes in current DFS path
        List<Integer> path = new ArrayList<>();      // Current path for cycle reconstruction
        
        for (Integer node : graph.keySet()) {
            if (!visited.contains(node)) {
                dfsFindCycles(node, graph, visited, inStack, path, cycles);
            }
        }
        
        return cycles;
    }
    
    /**
     * DFS helper for cycle detection.
     */
    private static void dfsFindCycles(Integer node, Map<Integer, Set<Integer>> graph,
                                      Set<Integer> visited, Set<Integer> inStack,
                                      List<Integer> path, List<List<Integer>> cycles) {
        visited.add(node);
        inStack.add(node);
        path.add(node);
        
        for (Integer neighbor : graph.getOrDefault(node, Collections.emptySet())) {
            if (!visited.contains(neighbor)) {
                dfsFindCycles(neighbor, graph, visited, inStack, path, cycles);
            } else if (inStack.contains(neighbor)) {
                // Found a cycle! Extract it from path
                int cycleStart = path.indexOf(neighbor);
                if (cycleStart >= 0) {
                    List<Integer> cycle = new ArrayList<>(path.subList(cycleStart, path.size()));
                    // Avoid duplicate cycles (same cycle reported from different starting points)
                    if (!isDuplicateCycle(cycle, cycles)) {
                        cycles.add(cycle);
                    }
                }
            }
        }
        
        path.remove(path.size() - 1);
        inStack.remove(node);
    }
    
    /**
     * Checks if this cycle is a duplicate of an already found cycle.
     * Two cycles are duplicates if they contain the same nodes (possibly in different order).
     */
    private static boolean isDuplicateCycle(List<Integer> newCycle, List<List<Integer>> existingCycles) {
        Set<Integer> newSet = new HashSet<>(newCycle);
        for (List<Integer> existing : existingCycles) {
            if (new HashSet<>(existing).equals(newSet)) {
                return true;
            }
        }
        return false;
    }
    
    // ========== Utility Methods ==========
    
    /**
     * Quick check: does any cyclic dependency exist?
     * Faster than full analysis when you only need a yes/no answer.
     */
    public static boolean hasCyclicDependency(State state, Level level) {
        return analyze(state, level).hasCycle;
    }
}
