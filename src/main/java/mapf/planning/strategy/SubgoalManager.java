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
    private static final int OPERATION_SIDE_FALLBACK_PENALTY = 10;
    
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
     * Also invalidates the color-aware distance cache since box positions may have changed.
     * Call when the world state has changed significantly (e.g., after completing a subgoal).
     */
    public void invalidateHungarianCache() {
        hungarianCache = null;
        immovableDetector.invalidateColorAwareCache();
    }
    
    /** Returns true if a Hungarian assignment cache is currently active. */
    public boolean hasHungarianCache() {
        return hungarianCache != null;
    }
    
    /**
     * Gets all unsatisfied subgoals in priority order.
     * Phase 1: Box goals (excluding already completed ones).
     * Phase 2: Agent goals, but only after all remaining box goals are satisfied.
     *
     * Agent goals are terminal placement tasks. Allowing an agent to park at its
     * own goal while other box goals remain turns terminal cells into ordinary
     * transit resources and can force NAMO relief to clear boxes that should be
     * solved as first-class box goals instead.
     */
    public List<PriorityPlanningStrategy.Subgoal> getUnsatisfiedSubgoals(State state, Level level, Set<Position> completedBoxGoals) {
        List<PriorityPlanningStrategy.Subgoal> unsatisfied = new ArrayList<>();
        
        Set<Position> staticGoals = immovableDetector.findPreSatisfiedStaticGoals(state, level);
        
        // Phase 1: Box goals (skip completed ones - MAPF permanent obstacle)
        addBoxGoals(unsatisfied, state, level, staticGoals, completedBoxGoals);

        // Phase 2: terminal agent goals only after all box goals are done.
        if (unsatisfied.isEmpty()) {
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
        // P1: Same-color agent load balancing.
        //
        // Old behavior (per-goal independent nearest):
        //   When a color has K agents and N>K goals, all N goals tended to be assigned
        //   to the single closest agent — leaving K-1 agents idle and creating a serial
        //   bottleneck. Particularly painful on donkeyK (5 brown), GroupEZ (3 blue),
        //   Medibots (3 same-color pairs), NameHere (2 pink → 16 box types).
        //
        // New behavior (cap-based load balancing):
        //   Per color group: cap = ceil(N/K). For each goal, pick the nearest same-color
        //   agent whose current assignment count is below cap. If all are at cap, fall
        //   back to absolute-nearest (preserves single-agent-color correctness).
        //
        // Complexity: O(N * K_per_color). Deterministic. Recomputed each call to match
        // existing stateless re-planning semantics in PriorityPlanningStrategy.

        // Pass 1: count unsatisfied goals per color and agents per color.
        Map<Color, Integer> goalsPerColor = new EnumMap<>(Color.class);
        Map<Color, Integer> agentsPerColor = new EnumMap<>(Color.class);

        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            Color boxColor = level.getBoxColor(goalType);
            for (Position goalPos : entry.getValue()) {
                if (staticGoals.contains(goalPos)) continue;
                if (completedBoxGoals.contains(goalPos)) continue;
                Character actualBox = state.getBoxes().get(goalPos);
                if (actualBox == null || actualBox != goalType) {
                    goalsPerColor.merge(boxColor, 1, Integer::sum);
                }
            }
        }
        for (int i = 0; i < state.getNumAgents(); i++) {
            agentsPerColor.merge(level.getAgentColor(i), 1, Integer::sum);
        }

        Map<Color, Integer> capPerColor = new EnumMap<>(Color.class);
        for (Map.Entry<Color, Integer> e : goalsPerColor.entrySet()) {
            int n = e.getValue();
            int m = agentsPerColor.getOrDefault(e.getKey(), 0);
            if (m <= 0) {
                capPerColor.put(e.getKey(), Integer.MAX_VALUE); // no same-color agent — leaves goal unassigned
            } else {
                capPerColor.put(e.getKey(), (n + m - 1) / m); // ceil(n/m)
            }
        }

        // Pass 2: assign goals respecting per-agent cap.
        Map<Integer, Integer> agentLoad = new HashMap<>();
        for (Map.Entry<Character, List<Position>> entry : level.getBoxGoalsByType().entrySet()) {
            char goalType = entry.getKey();
            Color boxColor = level.getBoxColor(goalType);
            int cap = capPerColor.getOrDefault(boxColor, Integer.MAX_VALUE);

            for (Position goalPos : entry.getValue()) {
                if (staticGoals.contains(goalPos)) continue;
                if (completedBoxGoals.contains(goalPos)) continue;

                Character actualBox = state.getBoxes().get(goalPos);
                if (actualBox == null || actualBox != goalType) {
                    int agentId = findBalancedAgentForColor(boxColor, goalPos, level, state, agentLoad, cap);
                    if (agentId != -1) {
                        unsatisfied.add(new PriorityPlanningStrategy.Subgoal(agentId, goalType, goalPos, false));
                        agentLoad.merge(agentId, 1, Integer::sum);
                    }
                }
            }
        }
    }

    /**
     * Picks the nearest same-color agent whose assigned-goal count is below {@code cap}.
     * Falls back to absolute-nearest if every same-color agent is at the cap.
     * Returns -1 if no same-color agent exists.
     */
    private int findBalancedAgentForColor(Color color, Position target, Level level, State state,
                                          Map<Integer, Integer> agentLoad, int cap) {
        int bestUnderCapId = -1;
        int bestUnderCapDist = Integer.MAX_VALUE;
        int bestAnyId = -1;
        int bestAnyDist = Integer.MAX_VALUE;

        int numAgents = state.getNumAgents();
        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) != color) continue;
            Position agentPos = state.getAgentPosition(i);
            int dist = immovableDetector.getDistanceWithImmovableBoxes(agentPos, target, state, level);

            if (dist < bestAnyDist) {
                bestAnyDist = dist;
                bestAnyId = i;
            }
            int load = agentLoad.getOrDefault(i, 0);
            if (load >= cap) continue;
            if (dist < bestUnderCapDist) {
                bestUnderCapDist = dist;
                bestUnderCapId = i;
            }
        }
        return bestUnderCapId != -1 ? bestUnderCapId : bestAnyId;
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
        
        Set<Position> immovableBoxes = immovableDetector.getImmovableBoxes(state, level);
        int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(closestBox, subgoal.goalPos, state, level);
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        int agentToBox = estimateAgentToBoxWorkDistance(
                subgoal.agentId, subgoal.boxType, agentPos, closestBox, state, level, immovableBoxes);
        
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
            int agentToBox = estimateAgentToBoxWorkDistance(
                    subgoal.agentId, subgoal.boxType, agentPos, boxPos, state, level, immovableBoxes);
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

    public BoxSelectionDiagnosis diagnoseBestBoxForGoal(PriorityPlanningStrategy.Subgoal subgoal,
                                                        State state,
                                                        Level level,
                                                        List<PriorityPlanningStrategy.Subgoal> allPendingSubgoals,
                                                        Set<Position> completedBoxGoals) {
        Map<String, String> details = new LinkedHashMap<>();
        Map<String, Integer> rejectCounts = new LinkedHashMap<>();
        List<String> samples = new ArrayList<>();
        List<BoxCandidate> candidates = new ArrayList<>();
        Position agentPos = state.getAgentPosition(subgoal.agentId);
        Set<Position> immovableBoxes = immovableDetector.getImmovableBoxes(state, level);
        int rawBoxesOfType = 0;
        int operationSideFallbacks = 0;

        String hungarianStatus = "no-cache";
        if (hungarianCache != null) {
            HungarianBoxAssigner.AssignmentResult result = hungarianCache.get(subgoal.boxType);
            if (result == null) {
                hungarianStatus = "no-assignment-for-box-type";
            } else {
                Position assignedBox = result.getAssignedBox(subgoal.goalPos);
                if (assignedBox == null) {
                    hungarianStatus = "no-assigned-box-for-goal";
                } else {
                    details.put("hungarianAssignedBox", positionText(assignedBox));
                    Character boxAtPos = state.getBoxes().get(assignedBox);
                    if (boxAtPos == null || boxAtPos != subgoal.boxType) {
                        hungarianStatus = "assigned-box-moved-or-wrong-type";
                    } else if (immovableBoxes.contains(assignedBox)) {
                        hungarianStatus = "assigned-box-immovable";
                    } else if (level.getBoxGoal(assignedBox) == subgoal.boxType) {
                        hungarianStatus = "assigned-box-already-on-goal";
                    } else if (!isBoxMovable(assignedBox, state, level)) {
                        hungarianStatus = "assigned-box-has-no-operation-side";
                    } else {
                        int agentToBox = estimateAgentToBoxWorkDistance(
                                subgoal.agentId, subgoal.boxType, agentPos, assignedBox,
                                state, level, immovableBoxes);
                        int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(
                                assignedBox, subgoal.goalPos, state, level);
                        if (agentToBox == Integer.MAX_VALUE) {
                            hungarianStatus = "assigned-box-agent-unreachable";
                        } else if (boxToGoal == Integer.MAX_VALUE) {
                            hungarianStatus = "assigned-box-goal-unreachable";
                        } else if (!isAllocationFeasible(assignedBox, subgoal, state, level, allPendingSubgoals)) {
                            hungarianStatus = "assigned-box-allocation-infeasible";
                        } else {
                            hungarianStatus = "would-select";
                            details.put("selectedBox", positionText(assignedBox));
                            details.put("selectionLayer", "hungarian");
                            details.put("agentToBoxWorkDistance", Integer.toString(agentToBox));
                            details.put("boxToGoalDistance", Integer.toString(boxToGoal));
                        }
                    }
                }
            }
        }

        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() != subgoal.boxType) continue;
            rawBoxesOfType++;
            Position boxPos = entry.getKey();

            String rejectReason = null;
            if (level.getBoxGoal(boxPos) == subgoal.boxType) {
                rejectReason = "already-satisfied-goal";
            } else if (subgoal.boxType == level.getBoxGoal(boxPos) && completedBoxGoals.contains(boxPos)) {
                rejectReason = "completed-goal-frozen";
            } else if (immovableBoxes.contains(boxPos)) {
                rejectReason = "immovable";
            } else if (!isBoxMovable(boxPos, state, level)) {
                rejectReason = "no-operation-side";
            }

            if (rejectReason != null) {
                increment(rejectCounts, rejectReason);
                addSample(samples, boxPos, rejectReason);
                continue;
            }

            int operationSide = estimateAgentToOperationSide(agentPos, boxPos, state, level, immovableBoxes);
            int agentToBox = estimateAgentToBoxWorkDistance(
                    subgoal.agentId, subgoal.boxType, agentPos, boxPos, state, level, immovableBoxes);
            if (agentToBox == Integer.MAX_VALUE) {
                increment(rejectCounts, "agent-to-box-unreachable");
                addSample(samples, boxPos, "agent-to-box-unreachable");
                continue;
            }
            if (operationSide == Integer.MAX_VALUE) {
                operationSideFallbacks++;
            }

            int boxToGoal = immovableDetector.getDistanceWithImmovableBoxes(boxPos, subgoal.goalPos, state, level);
            if (boxToGoal == Integer.MAX_VALUE) {
                increment(rejectCounts, "box-to-goal-unreachable");
                addSample(samples, boxPos, "box-to-goal-unreachable");
                continue;
            }

            candidates.add(new BoxCandidate(boxPos, agentToBox + boxToGoal));
        }

        candidates.sort(Comparator.comparingInt(c -> c.dist));
        int allocationFailed = 0;
        for (BoxCandidate candidate : candidates) {
            if (isAllocationFeasible(candidate.pos, subgoal, state, level, allPendingSubgoals)) {
                if (!"would-select".equals(hungarianStatus)) {
                    details.put("selectedBox", positionText(candidate.pos));
                    details.put("selectionLayer", "greedy");
                    details.put("selectedDistance", Integer.toString(candidate.dist));
                }
                return new BoxSelectionDiagnosis("would-select", detailsWithSelectionCounts(details,
                        hungarianStatus, rawBoxesOfType, candidates.size(), operationSideFallbacks,
                        allocationFailed, rejectCounts, samples));
            }
            allocationFailed++;
            increment(rejectCounts, "allocation-feasibility-failed");
            addSample(samples, candidate.pos, "allocation-feasibility-failed");
        }

        String reason;
        if (rawBoxesOfType == 0) {
            reason = "no-box-of-type";
        } else if (candidates.isEmpty()) {
            reason = dominantReason(rejectCounts, "no-usable-box-candidate");
        } else {
            reason = "allocation-feasibility-failed";
        }
        details.put("dominantReason", reason);
        return new BoxSelectionDiagnosis(reason, detailsWithSelectionCounts(details, hungarianStatus,
                rawBoxesOfType, candidates.size(), operationSideFallbacks, allocationFailed,
                rejectCounts, samples));
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
        int agentToBox = estimateAgentToBoxWorkDistance(
                subgoal.agentId, subgoal.boxType, agentPos, assignedBox, state, level, immovableBoxes);
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

    public record BoxSelectionDiagnosis(String reason, Map<String, String> details) {
    }

    private static Map<String, String> detailsWithSelectionCounts(Map<String, String> details,
                                                                  String hungarianStatus,
                                                                  int rawBoxesOfType,
                                                                  int usableCandidates,
                                                                  int operationSideFallbacks,
                                                                  int allocationFailed,
                                                                  Map<String, Integer> rejectCounts,
                                                                  List<String> samples) {
        Map<String, String> out = new LinkedHashMap<>(details);
        out.put("hungarianStatus", hungarianStatus);
        out.put("rawBoxesOfType", Integer.toString(rawBoxesOfType));
        out.put("usableBoxCandidates", Integer.toString(usableCandidates));
        out.put("operationSideFallbackCandidates", Integer.toString(operationSideFallbacks));
        out.put("allocationFailedCandidates", Integer.toString(allocationFailed));
        if (!rejectCounts.isEmpty()) {
            List<String> parts = new ArrayList<>();
            for (Map.Entry<String, Integer> entry : rejectCounts.entrySet()) {
                parts.add(entry.getKey() + "=" + entry.getValue());
            }
            out.put("candidateRejectCounts", String.join(", ", parts));
        }
        if (!samples.isEmpty()) {
            out.put("candidateSamples", String.join("; ", samples.subList(0, Math.min(8, samples.size()))));
        }
        return out;
    }

    private static void increment(Map<String, Integer> counts, String key) {
        counts.merge(key, 1, Integer::sum);
    }

    private static void addSample(List<String> samples, Position position, String reason) {
        if (samples.size() >= 8) return;
        samples.add(positionText(position) + ":" + reason);
    }

    private static String dominantReason(Map<String, Integer> counts, String fallback) {
        String best = fallback;
        int bestCount = -1;
        for (Map.Entry<String, Integer> entry : counts.entrySet()) {
            if (entry.getValue() > bestCount) {
                best = entry.getKey();
                bestCount = entry.getValue();
            }
        }
        return best;
    }

    private static String positionText(Position position) {
        return position == null ? "null" : position.row + "," + position.col;
    }

    /**
     * Estimates the cost for an agent to start working on a box.
     *
     * A raw agent->box distance is too optimistic in pull-push Sokoban: the agent
     * cannot stand on the box; it must reach an adjacent operation side with room
     * for at least one push/pull-style transition. Prefer that operation-side
     * distance, but keep a penalized direct fallback so tightly packed boxes can
     * still be considered by the real BSP search.
     */
    private int estimateAgentToBoxWorkDistance(int agentId, char boxType, Position agentPos,
                                               Position boxPos, State state, Level level,
                                               Set<Position> immovableBoxes) {
        if (agentPos == null || boxPos == null || !level.canAgentMoveBox(agentId, boxType)) {
            return Integer.MAX_VALUE;
        }

        int operationSide = estimateAgentToOperationSide(agentPos, boxPos, state, level, immovableBoxes);
        if (operationSide < Integer.MAX_VALUE) {
            return operationSide;
        }

        int direct = immovableDetector.getDistanceWithImmovableBoxes(agentPos, boxPos, state, level);
        if (direct == Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        return direct + OPERATION_SIDE_FALLBACK_PENALTY;
    }

    private int estimateAgentToOperationSide(Position agentPos, Position boxPos, State state,
                                             Level level, Set<Position> immovableBoxes) {
        int best = Integer.MAX_VALUE;
        for (Direction dir : Direction.values()) {
            Position stand = boxPos.move(dir);
            if (!isOperationStandCell(stand, agentPos, state, level, immovableBoxes)) {
                continue;
            }
            if (!hasOperationExit(boxPos, stand, state, level, immovableBoxes)) {
                continue;
            }
            int d = immovableDetector.getDistanceWithImmovableBoxes(agentPos, stand, state, level);
            if (d < best) {
                best = d;
            }
        }
        return best;
    }

    private boolean isOperationStandCell(Position pos, Position agentPos, State state,
                                         Level level, Set<Position> immovableBoxes) {
        if (!level.isFree(pos) || immovableBoxes.contains(pos) || state.hasBoxAt(pos)) {
            return false;
        }
        return !state.hasAgentAt(pos) || pos.equals(agentPos);
    }

    private boolean hasOperationExit(Position boxPos, Position stand, State state,
                                     Level level, Set<Position> immovableBoxes) {
        for (Direction dir : Direction.values()) {
            Position next = boxPos.move(dir);
            if (next.equals(stand)) {
                continue;
            }
            if (level.isFree(next) && !immovableBoxes.contains(next)
                    && !state.hasBoxAt(next) && !state.hasAgentAt(next)) {
                return true;
            }
        }
        for (Direction dir : Direction.values()) {
            Position nextAgent = stand.move(dir);
            if (nextAgent.equals(boxPos)) {
                continue;
            }
            if (level.isFree(nextAgent) && !immovableBoxes.contains(nextAgent)
                    && !state.hasBoxAt(nextAgent) && !state.hasAgentAt(nextAgent)) {
                return true;
            }
        }
        return false;
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
     * Uses color-agnostic distance (not color-aware) because agent assignment is about
     * REACHABILITY with full coordination — other agents can clear obstacles.
     */
    private int findNearestAgentForColor(Color color, Position target, Level level, State state) {
        int bestAgentId = -1;
        int minDistance = Integer.MAX_VALUE;
        int numAgents = state.getNumAgents();

        for (int i = 0; i < numAgents; i++) {
            if (level.getAgentColor(i) == color) {
                Position agentPos = state.getAgentPosition(i);
                
                int dist = immovableDetector.getDistanceWithImmovableBoxes(agentPos, target, state, level);
                
                if (dist < minDistance) {
                    minDistance = dist;
                    bestAgentId = i;
                }
            }
        }
        return bestAgentId;
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
