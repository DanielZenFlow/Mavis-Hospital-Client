package mapf.planning.strategy;

import mapf.domain.*;
import mapf.planning.heuristic.Heuristic;

import java.util.*;

/**
 * Manages subgoal identification, ordering, and difficulty estimation.
 * Extracted from PriorityPlanningStrategy for Single Responsibility Principle.
 */
public class SubgoalManager {
    
    private final Heuristic heuristic;
    private final ImmovableBoxDetector immovableDetector;
    
    public SubgoalManager(Heuristic heuristic) {
        this.heuristic = heuristic;
        this.immovableDetector = new ImmovableBoxDetector();
    }
    
    /**
     * Gets all unsatisfied subgoals in priority order.
     * Phase 1: Box goals (excluding already completed ones)
     * Phase 2: Agent goals for agents whose own-color box tasks are done
     *          (not deferred until ALL box goals complete)
     */
    public List<PriorityPlanningStrategy.Subgoal> getUnsatisfiedSubgoals(State state, Level level, Set<Position> completedBoxGoals) {
        List<PriorityPlanningStrategy.Subgoal> unsatisfied = new ArrayList<>();
        
        Set<Position> staticGoals = immovableDetector.findPreSatisfiedStaticGoals(state, level);
        
        // Phase 1: Box goals (skip completed ones - MAPF permanent obstacle)
        addBoxGoals(unsatisfied, state, level, staticGoals, completedBoxGoals);
        
        // Phase 2: Agent goals for agents that have completed their own box tasks.
        // This allows agent goals to be attempted in parallel with other agents' box goals,
        // rather than waiting for ALL box goals to complete first.
        addCompletedAgentGoals(unsatisfied, state, level);
        
        // Phase 3: If no box goals remain at all, add ALL remaining agent goals
        // (catches agent goals for agents that had no box tasks)
        if (unsatisfied.isEmpty() || !hasAnyBoxGoals(unsatisfied)) {
            addAllAgentGoals(unsatisfied, state, level);
        }
        
        return unsatisfied;
    }
    
    /** Legacy overload for backward compatibility. */
    public List<PriorityPlanningStrategy.Subgoal> getUnsatisfiedSubgoals(State state, Level level) {
        return getUnsatisfiedSubgoals(state, level, Collections.emptySet());
    }
    
    private void addBoxGoals(List<PriorityPlanningStrategy.Subgoal> unsatisfied, 
                            State state, Level level, Set<Position> staticGoals, Set<Position> completedBoxGoals) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType == '\0') continue;
                
                Position goalPos = new Position(row, col);
                
                // Skip pre-satisfied static goals (decorations)
                if (staticGoals.contains(goalPos)) continue;
                
                // Skip completed goals (MAPF: permanent obstacle)
                if (completedBoxGoals.contains(goalPos)) continue;
                
                Character actualBox = state.getBoxes().get(goalPos);
                if (actualBox == null || actualBox != goalType) {
                    Color boxColor = level.getBoxColor(goalType);
                    // MAPF FIX: Choose NEAREST agent instead of first available (index-based)
                    int agentId = findNearestAgentForColor(boxColor, goalPos, level, state);
                    if (agentId != -1) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentId, goalType, goalPos, false));
                    }
                }
            }
        }
    }
    
    private void addCompletedAgentGoals(List<PriorityPlanningStrategy.Subgoal> unsatisfied,
                                       State state, Level level) {
        for (int agentId = 0; agentId < state.getNumAgents(); agentId++) {
            if (hasCompletedBoxTasks(agentId, state, level)) {
                Position agentGoal = findAgentGoalPosition(agentId, level);
                if (agentGoal != null) {
                    Position agentPos = state.getAgentPosition(agentId);
                    if (!agentPos.equals(agentGoal)) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentId, '\0', agentGoal, true));
                    }
                }
            }
        }
    }
    
    private void addAllAgentGoals(List<PriorityPlanningStrategy.Subgoal> unsatisfied,
                                 State state, Level level) {
        // Collect agent IDs already present in the list to avoid duplicates
        Set<Integer> existingAgentGoals = new HashSet<>();
        for (PriorityPlanningStrategy.Subgoal sg : unsatisfied) {
            if (sg.isAgentGoal) {
                existingAgentGoals.add(sg.agentId);
            }
        }
        
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                int agentGoal = level.getAgentGoal(row, col);
                if (agentGoal >= 0 && agentGoal < state.getNumAgents()) {
                    if (existingAgentGoals.contains(agentGoal)) continue; // Already added
                    Position goalPos = new Position(row, col);
                    Position agentPos = state.getAgentPosition(agentGoal);
                    if (!agentPos.equals(goalPos)) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentGoal, '\0', goalPos, true));
                    }
                }
            }
        }
    }
    
    /** Checks if the subgoal list contains any box goals. */
    private boolean hasAnyBoxGoals(List<PriorityPlanningStrategy.Subgoal> subgoals) {
        for (PriorityPlanningStrategy.Subgoal sg : subgoals) {
            if (!sg.isAgentGoal) return true;
        }
        return false;
    }
    
    public int estimateSubgoalDifficulty(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level) {
        if (subgoal.isAgentGoal) {
            Position agentPos = state.getAgentPosition(subgoal.agentId);
            return immovableDetector.getDistanceWithImmovableBoxes(agentPos, subgoal.goalPos, state, level);
        }
        
        Position closestBox = findBestBoxForGoal(subgoal, state, level);
        if (closestBox == null) return Integer.MAX_VALUE;
        
        int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(closestBox, subgoal.goalPos, state, level);
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        int agentToBox = immovableDetector.getDistanceWithImmovableBoxes(agentPos, closestBox, state, level);
        
        if (agentToBox == Integer.MAX_VALUE || boxToGoal == Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        
        return boxToGoal + agentToBox;
    }
    
    public Position findBestBoxForGoal(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level) {
        return findBestBoxForGoal(subgoal, state, level, Collections.emptyList());
    }

    /**
     * Finds the best box for a goal, ensuring that the assignment doesn't leave other goals unsolvable.
     * Uses Bipartite Matching to verify feasibility (Global Allocation Check).
     */
    public Position findBestBoxForGoal(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level, List<PriorityPlanningStrategy.Subgoal> allPendingSubgoals) {
        Position bestBox = null;
        int bestTotalDist = Integer.MAX_VALUE;
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        
        Set<Position> immovableBoxes = immovableDetector.getImmovableBoxes(state, level);
        
        // Collect all valid candidates first
        List<BoxCandidate> candidates = new ArrayList<>();

        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() != subgoal.boxType) continue;
            
            Position boxPos = entry.getKey();
            
            // Skip if box already at satisfied goal
            if (level.getBoxGoal(boxPos) == subgoal.boxType) continue;
            
            // Skip if box is immovable
            if (immovableBoxes.contains(boxPos)) continue;
            
            // Pukoban fix: check box has at least one direction where push or pull is possible
            if (!isBoxMovable(boxPos, state, level)) continue;
            
            // Check if agent can reach box
            int agentToBox = immovableDetector.getDistanceWithImmovableBoxes(agentPos, boxPos, state, level);
            if (agentToBox == Integer.MAX_VALUE) continue;
            
            int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(boxPos, subgoal.goalPos, state, level);
            if (boxToGoal == Integer.MAX_VALUE) continue;
            
            int totalDist = agentToBox + boxToGoal;
            candidates.add(new BoxCandidate(boxPos, totalDist));
        }

        // Sort by distance (greedy preference)
        candidates.sort(Comparator.comparingInt(c -> c.dist));

        // Iterate candidates and check feasibility
        for (BoxCandidate candidate : candidates) {
            // Task-Aware Allocation: Check if using this box leaves other goals solvable
            if (isAllocationFeasible(candidate.pos, subgoal, state, level, allPendingSubgoals)) {
                return candidate.pos;
            } else {
                if (mapf.planning.SearchConfig.isVerbose()) {
                    System.err.println("[SubgoalManager] Allocation rejected: Box " + candidate.pos 
                        + " for " + subgoal.goalPos + " would block other " + subgoal.boxType + " goals.");
                }
            }
        }

        // Fallback: If strict feasibility check fails for ALL (e.g. tight spacing), 
        // return NULL to indicate this goal cannot be safely served yet.
        // This allows PriorityPlanningStrategy to skip this goal and try others (Dynamic Reordering).
        if (!candidates.isEmpty()) {
            if (mapf.planning.SearchConfig.isNormal()) {
                System.err.println("[SubgoalManager] All " + candidates.size() + " candidates for " + subgoal.goalPos 
                    + " rejected by global feasibility check. Deferring goal.");
            }
            return null;
        }
        
        return null;
    }

    private static class BoxCandidate {
        Position pos;
        int dist;
        BoxCandidate(Position p, int d) { pos = p; dist = d; }
    }

    /**
     * Checks if assigning candidateBox to currentSubgoal leaves a valid assignment for all 
     * other pending subgoals of the SAME type.
     */
    private boolean isAllocationFeasible(Position candidateBox, PriorityPlanningStrategy.Subgoal currentSubgoal, 
                                         State state, Level level, List<PriorityPlanningStrategy.Subgoal> allPendingSubgoals) {
        if (allPendingSubgoals == null || allPendingSubgoals.isEmpty()) return true;

        // 1. Identify remaining goals of same type
        List<Position> remainingGoals = new ArrayList<>();
        for (PriorityPlanningStrategy.Subgoal sg : allPendingSubgoals) {
            if (sg != currentSubgoal && !sg.isAgentGoal && sg.boxType == currentSubgoal.boxType) {
                remainingGoals.add(sg.goalPos);
            }
        }
        
        if (remainingGoals.isEmpty()) return true; // No other goals of this type, trivial yes

        // 2. Identify remaining boxes of same type (excluding candidateBox)
        List<Position> remainingBoxes = new ArrayList<>();
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == currentSubgoal.boxType) {
                Position p = entry.getKey();
                // Exclude the box we are planning to use
                if (!p.equals(candidateBox)) {
                    // Exclude boxes already at correct goals (they verify themselves)
                    if (level.getBoxGoal(p) != currentSubgoal.boxType) {
                        remainingBoxes.add(p);
                    }
                }
            }
        }

        // If fewer boxes than goals, obviously infeasible (but shouldn't happen in valid level)
        if (remainingBoxes.size() < remainingGoals.size()) return false;

        // 3. Build Reachability Graph (Bipartite)
        // Constraints:
        // - candidateBox is GONE (already removed from remainingBoxes)
        // - currentSubgoal.goalPos is BLOCKED (it will be filled)
        // - Existing immovable boxes are BLOCKED
        // - Agents? We assume agent can move almost anywhere reachable. 
        //   We check box-path reachability.
        
        Set<Position> obstacles = new HashSet<>(immovableDetector.getImmovableBoxes(state, level));
        obstacles.add(currentSubgoal.goalPos); // The goal we are filling becomes an obstacle

        // Adjacency: goals -> reachable boxes
        Map<Position, List<Position>> adj = new HashMap<>();
        for (Position g : remainingGoals) {
            List<Position> reachable = new ArrayList<>();
            for (Position b : remainingBoxes) {
                if (isReachable(b, g, obstacles, level)) {
                    reachable.add(b);
                }
            }
            adj.put(g, reachable);
            if (reachable.isEmpty()) return false; // Optimization: goal unreachable by ANY box
        }

        // 4. Max Bipartite Matching
        // goal -> box matching
        return canMatchAll(remainingGoals, remainingBoxes, adj);
    }
    
    // Simple BFS for reachability with custom obstacles
    private boolean isReachable(Position from, Position to, Set<Position> obstacles, Level level) {
        if (from.equals(to)) return true;
        
        Queue<Position> q = new LinkedList<>();
        Set<Position> visited = new HashSet<>();
        q.add(from);
        visited.add(from);
        
        while(!q.isEmpty()) {
            Position cur = q.poll();
            if (cur.equals(to)) return true;
            
            for (Direction dir : Direction.values()) {
                Position next = cur.move(dir);
                if (!level.isWall(next) && !obstacles.contains(next) && !visited.contains(next)) {
                    visited.add(next);
                    q.add(next);
                }
            }
        }
        return false;
    }

    // Maximum Bipartite Matching (Hopcroft-Karp or simply augmenting paths for small graphs)
    private boolean canMatchAll(List<Position> goals, List<Position> boxes, Map<Position, List<Position>> adj) {
        Map<Position, Position> match = new HashMap<>(); // goal -> box (not needed actually, we need box->goal)
        Map<Position, Position> boxMatch = new HashMap<>(); // box -> matching goal
        
        int matches = 0;
        for (Position goal : goals) {
            Set<Position> visitedBoxes = new HashSet<>();
            if (dfsMatch(goal, visitedBoxes, boxMatch, adj)) {
                matches++;
            }
        }
        return matches == goals.size();
    }

    private boolean dfsMatch(Position goal, Set<Position> visitedBoxes, Map<Position, Position> boxMatch, Map<Position, List<Position>> adj) {
        for (Position box : adj.getOrDefault(goal, Collections.emptyList())) {
            if (visitedBoxes.contains(box)) continue;
            visitedBoxes.add(box);
            
            Position assignedGoal = boxMatch.get(box);
            if (assignedGoal == null || dfsMatch(assignedGoal, visitedBoxes, boxMatch, adj)) {
                boxMatch.put(box, goal);
                return true;
            }
        }
        return false;
    }
    
    /**
     * Checks if a box can be pushed or pulled in at least one direction.
     * Push: requires agent-side free AND push-direction free (opposite neighbors along an axis)
     * Pull: requires agent adjacent AND agent has somewhere to move while pulling
     * Returns true if box is not stuck.
     */
    private boolean isBoxMovable(Position boxPos, State state, Level level) {
        int freeNeighborCount = 0;
        boolean hasPushAxis = false;
        
        Direction[] dirs = Direction.values();
        for (Direction dir : dirs) {
            Position neighbor = boxPos.move(dir);
            if (!level.isWall(neighbor) && !state.hasBoxAt(neighbor)) {
                freeNeighborCount++;
                // Check opposite direction for push possibility
                Position opposite = boxPos.move(dir.opposite());
                if (!level.isWall(opposite) && !state.hasBoxAt(opposite)) {
                    hasPushAxis = true; // Can push along this axis
                }
            }
        }
        
        // Pull only needs 1 free neighbor (agent stands adjacent, pulls box toward itself)
        // Push needs 2 opposite free neighbors
        // Either is sufficient to move the box
        return hasPushAxis || freeNeighborCount >= 1;
    }
    
    /**
     * Finds the nearest agent of the matching color to the target position.
     * Uses connectivity-aware BFS distance instead of Manhattan.
     */
    private int findNearestAgentForColor(Color color, Position target, Level level, State state) {
        int bestAgentId = -1;
        int minDistance = Integer.MAX_VALUE;
        int numAgents = state.getNumAgents();

        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) {
                Position agentPos = state.getAgentPosition(i);
                
                // FIX: Use reachable distance instead of Manhattan logic
                // This prevents assigning tasks to agents in disconnected components (e.g., trapped in rooms).
                // getDistanceWithImmovableBoxes returns MAX_VALUE if unreachable.
                int dist = immovableDetector.getDistanceWithImmovableBoxes(agentPos, target, state, level);
                
                if (dist < minDistance) {
                    minDistance = dist;
                    bestAgentId = i;
                }
            }
        }
        return bestAgentId;
    }
    
    /** Legacy method maintained for compilation if referenced elsewhere, but discouraged. */
    private int findAgentForColor(Color color, Level level, int numAgents) {
        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) return i;
        }
        return -1;
    }
    
    private boolean hasCompletedBoxTasks(int agentId, State state, Level level) {
        Color agentColor = level.getAgentColor(agentId);
        Position agentPos = state.getAgentPosition(agentId);

        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                char goalType = level.getBoxGoal(row, col);
                if (goalType != '\0' && level.getBoxColor(goalType) == agentColor) {
                    Position goalPos = new Position(row, col);
                    Character actualBox = state.getBoxes().get(goalPos);
                    
                    // If goal is not satisfied...
                    if (actualBox == null || actualBox != goalType) {
                        // FIX: Only consider it "my task" if I can actually reach the goal area.
                        // If the goal is in a disconnected component (e.g. locked room), 
                        // I shouldn't wait for it.
                        if (immovableDetector.getDistanceWithImmovableBoxes(agentPos, goalPos, state, level) != Integer.MAX_VALUE) {
                            return false; // Found a reachable, unsatisfied goal of my color
                        }
                    }
                }
            }
        }
        return true;
    }
    
    private Position findAgentGoalPosition(int agentId, Level level) {
        for (int row = 0; row < level.getRows(); row++) {
            for (int col = 0; col < level.getCols(); col++) {
                if (level.getAgentGoal(row, col) == agentId) {
                    return new Position(row, col);
                }
            }
        }
        return null;
    }
}
