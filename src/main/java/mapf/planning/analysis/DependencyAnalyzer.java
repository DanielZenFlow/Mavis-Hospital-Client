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
                
                String reason = checkDependency(agentA, agentB, state, level);
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
    private static String checkDependency(int agentA, int agentB, State state, Level level) {
        Position posA = state.getAgentPosition(agentA);
        Position posB = state.getAgentPosition(agentB);
        
        // Case 1: AgentB is at AgentA's goal position
        Position goalA = findAgentGoal(agentA, level);
        if (goalA != null && posB.equals(goalA)) {
            return "Agent " + agentB + " is at Agent " + agentA + "'s goal position " + goalA;
        }
        
        // Case 2: AgentB is blocking the path to AgentA's goal
        if (goalA != null) {
            if (isBlockingPath(posA, goalA, posB, state, level)) {
                return "Agent " + agentB + " is blocking path from " + posA + " to goal " + goalA;
            }
        }
        
        // Case 3: Check if AgentB is blocking access to boxes that AgentA needs
        Color colorA = level.getAgentColor(agentA);
        for (Map.Entry<Position, Character> box : state.getBoxes().entrySet()) {
            char boxType = box.getValue();
            Position boxPos = box.getKey();
            
            // Is this a box that AgentA can push and needs to push?
            if (level.getBoxColor(boxType) == colorA) {
                // Check if there's an unsatisfied goal for this box type
                Position boxGoal = findUnsatisfiedBoxGoal(boxType, state, level);
                if (boxGoal != null && !boxPos.equals(boxGoal)) {
                    // AgentA needs to push this box
                    // Check if AgentB is blocking access to this box
                    if (isBlockingPath(posA, boxPos, posB, state, level)) {
                        return "Agent " + agentB + " is blocking access to box " + boxType + " at " + boxPos;
                    }
                    // Check if AgentB is blocking the box's path to its goal
                    if (isBlockingPath(boxPos, boxGoal, posB, state, level)) {
                        return "Agent " + agentB + " is blocking box " + boxType + "'s path to goal " + boxGoal;
                    }
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
     * Finds an unsatisfied goal position for a box type.
     */
    private static Position findUnsatisfiedBoxGoal(char boxType, State state, Level level) {
        List<Position> goals = level.getBoxGoalsByType().get(boxType);
        if (goals == null) return null;
        for (Position goalPos : goals) {
            Character currentBox = state.getBoxes().get(goalPos);
            if (currentBox == null || currentBox != boxType) {
                return goalPos; // This goal is unsatisfied
            }
        }
        return null;
    }
    
    /**
     * Checks if blocker position is on the critical path from start to goal.
     * Uses BFS to find if all shortest paths go through the blocker.
     */
    private static boolean isBlockingPath(Position start, Position goal, Position blocker,
                                          State state, Level level) {
        if (start.equals(goal)) return false;
        if (start.equals(blocker) || goal.equals(blocker)) return false;
        
        // BFS to check if we can reach goal without going through blocker
        Queue<Position> queue = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        
        queue.add(start);
        visited.add(start);
        visited.add(blocker); // Treat blocker as a wall
        
        int searchLimit = level.getRows() * level.getCols(); // Prevent infinite loops
        int explored = 0;
        
        while (!queue.isEmpty() && explored < searchLimit) {
            Position current = queue.poll();
            explored++;
            
            if (current.equals(goal)) {
                return false; // Found a path that doesn't go through blocker
            }
            
            // Try all 4 directions
            for (Direction dir : Direction.values()) {
                Position next = current.move(dir);
                
                if (visited.contains(next)) continue;
                if (level.isWall(next)) continue;
                
                visited.add(next);
                queue.add(next);
            }
        }
        
        // Couldn't reach goal without going through blocker
        return true;
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
