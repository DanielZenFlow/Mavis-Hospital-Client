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
    
    /** Cached Hungarian optimal assignments: boxType → (goalPos → boxPos). */
    private Map<Character, HungarianBoxAssigner.AssignmentResult> hungarianCache = null;
    
    public SubgoalManager(Heuristic heuristic) {
        this.heuristic = heuristic;
        this.immovableDetector = new ImmovableBoxDetector();
    }
    
    /**
     * Constructor with shared ImmovableBoxDetector for cross-strategy cache reuse.
     * Follows Dependency Inversion Principle: caller controls detector lifecycle.
     */
    public SubgoalManager(Heuristic heuristic, ImmovableBoxDetector sharedDetector) {
        this.heuristic = heuristic;
        this.immovableDetector = sharedDetector;
    }
    
    /**
     * Initializes precomputed BFS distance cache for all goal positions.
     * Must be called once before the main planning loop for O(1) distance lookups.
     * Safe to call multiple times (idempotent).
     */
    public void initDistanceCache(State state, Level level) {
        immovableDetector.initializeDistanceCache(state, level);
    }
    
    /**
     * Computes (or recomputes) the Hungarian optimal box-to-goal assignment cache.
     * Should be called once at the start of each planning episode (when subgoal list is first built).
     * Invalidates automatically when state changes require re-planning.
     */
    public void computeHungarianAssignment(State state, Level level, Set<Position> completedBoxGoals) {
        hungarianCache = HungarianBoxAssigner.computeAllAssignments(
                state, level, completedBoxGoals, immovableDetector);
        if (mapf.planning.SearchConfig.isVerbose()) {
            int totalAssigned = 0;
            for (HungarianBoxAssigner.AssignmentResult r : hungarianCache.values()) {
                totalAssigned += r.getGoalToBoxMap().size();
            }
            System.err.println("[SubgoalManager] Hungarian pre-assignment: " 
                    + totalAssigned + " goals across " + hungarianCache.size() + " types");
        }
    }
    
    /**
     * Invalidates the Hungarian cache, forcing recomputation on next access.
     * Call when the world state has changed significantly (e.g., after completing a subgoal).
     */
    public void invalidateHungarianCache() {
        hungarianCache = null;
    }
    
    /** Returns true if a Hungarian assignment cache is currently active. */
    public boolean hasHungarianCache() {
        return hungarianCache != null;
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
        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            Color boxColor = level.getBoxColor(goalType);
            
            for (Position goalPos : entry.getValue()) {
                // Skip pre-satisfied static goals (decorations)
                if (staticGoals.contains(goalPos)) continue;
                
                // Skip completed goals (MAPF: permanent obstacle)
                if (completedBoxGoals.contains(goalPos)) continue;
                
                Character actualBox = state.getBoxes().get(goalPos);
                if (actualBox == null || actualBox != goalType) {
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
        
        for (Map.Entry<Integer, Position> entry : level.getAgentGoalPositionMap().entrySet()) {
            int agentGoal = entry.getKey();
            if (agentGoal >= state.getNumAgents()) continue;
            if (existingAgentGoals.contains(agentGoal)) continue; // Already added
            Position goalPos = entry.getValue();
            Position agentPos = state.getAgentPosition(agentGoal);
            if (!agentPos.equals(goalPos)) {
                unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentGoal, '\0', goalPos, true));
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
    
    public int estimateSubgoalDifficulty(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level, Set<Position> completedBoxGoals) {
        if (subgoal.isAgentGoal) {
            Position agentPos = state.getAgentPosition(subgoal.agentId);
            return immovableDetector.getDistanceWithImmovableBoxes(agentPos, subgoal.goalPos, state, level);
        }
        
        Position closestBox = findBestBoxForGoal(subgoal, state, level, completedBoxGoals);
        if (closestBox == null) return Integer.MAX_VALUE;
        
        int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(closestBox, subgoal.goalPos, state, level);
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        int agentToBox = immovableDetector.getDistanceWithImmovableBoxes(agentPos, closestBox, state, level);
        
        if (agentToBox == Integer.MAX_VALUE || boxToGoal == Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        
        return boxToGoal + agentToBox;
    }
    
    public Position findBestBoxForGoal(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level, Set<Position> completedBoxGoals) {
        return findBestBoxForGoal(subgoal, state, level, Collections.emptyList(), completedBoxGoals);
    }

    /**
     * Finds the best box for a goal, ensuring that the assignment doesn't leave other goals unsolvable.
     * 
     * Strategy (layered):
     * 1. Consult Hungarian pre-computed optimal assignment (if available and box still valid)
     * 2. Fall back to greedy distance sort + bipartite feasibility check
     */
    public Position findBestBoxForGoal(PriorityPlanningStrategy.Subgoal subgoal, State state, Level level, List<PriorityPlanningStrategy.Subgoal> allPendingSubgoals, Set<Position> completedBoxGoals) {
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        Set<Position> immovableBoxes = immovableDetector.getImmovableBoxes(state, level);

        // --- Layer 1: Hungarian pre-assignment ---
        Position hungarianBox = getHungarianCandidate(subgoal, state, level, immovableBoxes, agentPos);
        if (hungarianBox != null) {
            // Validate: the Hungarian pick must pass the same feasibility check
            if (isAllocationFeasible(hungarianBox, subgoal, state, level, allPendingSubgoals)) {
                if (mapf.planning.SearchConfig.isVerbose()) {
                    System.err.println("[SubgoalManager] Using Hungarian assignment for " 
                            + subgoal.boxType + " -> " + subgoal.goalPos + ": box at " + hungarianBox);
                }
                return hungarianBox;
            }
        }

        // --- Layer 2: Greedy fallback with feasibility check ---
        List<BoxCandidate> candidates = new ArrayList<>();

        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() != subgoal.boxType) continue;
            
            Position boxPos = entry.getKey();
            
            // Skip if box already at satisfied goal
            if (level.getBoxGoal(boxPos) == subgoal.boxType) continue;
            
            // Skip if box is on a COMPLETED goal (meaning it's frozen/permanent)
            if (subgoal.boxType == level.getBoxGoal(boxPos) && completedBoxGoals.contains(boxPos)) continue;

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
            if (isAllocationFeasible(candidate.pos, subgoal, state, level, allPendingSubgoals)) {
                return candidate.pos;
            }
        }

        // Fallback: If strict feasibility check fails for ALL (e.g. tight spacing), 
        // return NULL to indicate this goal cannot be safely served yet.
        if (!candidates.isEmpty()) {
            if (mapf.planning.SearchConfig.isVerbose()) {
                System.err.println("[SubgoalManager] All " + candidates.size() + " candidates for " + subgoal.goalPos 
                    + " rejected by global feasibility check. Deferring goal.");
            }
            return null;
        }
        
        return null;
    }

    /**
     * Retrieves the Hungarian-assigned box for a subgoal, if the assignment is still valid.
     * Returns null if no Hungarian cache exists, the box type has no assignment,
     * or the assigned box is no longer available (moved, immovable, unreachable).
     */
    private Position getHungarianCandidate(PriorityPlanningStrategy.Subgoal subgoal, 
                                            State state, Level level,
                                            Set<Position> immovableBoxes, Position agentPos) {
        if (hungarianCache == null) return null;
        
        HungarianBoxAssigner.AssignmentResult result = hungarianCache.get(subgoal.boxType);
        if (result == null) return null;
        
        Position assignedBox = result.getAssignedBox(subgoal.goalPos);
        if (assignedBox == null) return null;
        
        // Validate the assigned box is still valid in the current state
        Character boxAtPos = state.getBoxes().get(assignedBox);
        if (boxAtPos == null || boxAtPos != subgoal.boxType) return null; // Box moved away
        if (immovableBoxes.contains(assignedBox)) return null;           // Box became immovable
        if (level.getBoxGoal(assignedBox) == subgoal.boxType) return null; // Box already at a goal
        if (!isBoxMovable(assignedBox, state, level)) return null;        // Box stuck
        
        // Check reachability
        int agentToBox = immovableDetector.getDistanceWithImmovableBoxes(agentPos, assignedBox, state, level);
        if (agentToBox == Integer.MAX_VALUE) return null;
        int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(assignedBox, subgoal.goalPos, state, level);
        if (boxToGoal == Integer.MAX_VALUE) return null;
        
        return assignedBox;
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

        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            if (level.getBoxColor(goalType) != agentColor) continue;
            for (Position goalPos : entry.getValue()) {
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
        return true;
    }
    
    private Position findAgentGoalPosition(int agentId, Level level) {
        return level.getAgentGoalPositionMap().get(agentId);
    }
}
