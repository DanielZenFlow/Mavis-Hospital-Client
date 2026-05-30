package mapf.planning.diag;

import mapf.domain.Position;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Lightweight side-channel for planner intent diagnostics.
 * <p>
 * ReplayRecorder observes server actions and states; it cannot infer why the
 * planner chose those actions. PlanTrace lets planners attach that missing
 * intent to replay frame numbers without changing the solver result type.
 */
public final class PlanTrace {
    private final Map<Integer, List<Event>> eventsByFrame = new LinkedHashMap<>();
    private final List<PortfolioAttempt> portfolioAttempts = new ArrayList<>();

    public void clear() {
        eventsByFrame.clear();
        portfolioAttempts.clear();
    }

    public void truncate(int actionCount) {
        int lastFrame = Math.max(0, actionCount);
        eventsByFrame.keySet().removeIf(frame -> frame > lastFrame);
    }

    public int count() {
        int count = 0;
        for (List<Event> events : eventsByFrame.values()) {
            count += events.size();
        }
        return count;
    }

    public List<PortfolioAttempt> portfolioAttempts() {
        return Collections.unmodifiableList(portfolioAttempts);
    }

    public void replacePortfolioAttempts(Collection<PortfolioAttempt> attempts) {
        portfolioAttempts.clear();
        if (attempts == null) return;
        for (PortfolioAttempt attempt : attempts) {
            if (attempt != null) portfolioAttempts.add(attempt);
        }
    }

    public List<Event> eventsForFrame(int frame) {
        List<Event> events = eventsByFrame.get(frame);
        return events == null ? Collections.emptyList() : Collections.unmodifiableList(events);
    }

    public void add(Event event) {
        if (event == null || event.frame() < 0) return;
        eventsByFrame.computeIfAbsent(event.frame(), ignored -> new ArrayList<>()).add(event);
    }

    public void recordIntentStep(int actionIndex,
                                 String phase,
                                 int agentId,
                                 String subgoalType,
                                 String subgoal,
                                 Position goal,
                                 Character boxType,
                                 int stepInSegment,
                                 int segmentSteps,
                                 String plannedAction,
                                 String actualAction,
                                 String reason) {
        int frame = actionIndex + 1;
        String progress = segmentSteps > 0 ? stepInSegment + "/" + segmentSteps : "?";
        add(new Event(
                frame,
                "agent-intent",
                "debug",
                "agent" + agentId + " intent",
                phase + " " + subgoal + " step " + progress,
                agentId,
                phase,
                subgoal,
                subgoalType,
                goal,
                boxType,
                stepInSegment,
                segmentSteps,
                plannedAction,
                actualAction,
                reason,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                Collections.emptyMap()
        ));
    }

    public void recordSubgoalEval(int actionIndex,
                                  String phase,
                                  int agentId,
                                  String subgoalType,
                                  String subgoal,
                                  Position goal,
                                  Character boxType,
                                  String verdict,
                                  boolean synthetic,
                                  int agentReachBefore,
                                  int agentReachAfter,
                                  int totalReachBefore,
                                  int totalReachAfter,
                                  boolean goalAdjBefore,
                                  boolean goalAdjAfter) {
        int frame = actionIndex + 1;
        String severity = "ACCEPTED".equals(verdict) || "CERTIFICATE_RESOLVED".equals(verdict)
                || "TARGET_ADJ_OPENED".equals(verdict) ? "info" : "warning";
        add(new Event(
                frame,
                "subgoal-eval",
                severity,
                "Subgoal " + verdict,
                phase + " " + subgoal,
                agentId,
                phase,
                subgoal,
                subgoalType,
                goal,
                boxType,
                null,
                null,
                null,
                null,
                null,
                verdict,
                synthetic,
                agentReachBefore,
                agentReachAfter,
                totalReachBefore,
                totalReachAfter,
                goalAdjBefore,
                goalAdjAfter,
                Collections.emptyMap()
        ));
    }

    public void recordDecisionEvent(int frame,
                                    String kind,
                                    String severity,
                                    String title,
                                    String message,
                                    Integer agentId,
                                    String phase,
                                    String subgoal,
                                    String subgoalType,
                                    Position goal,
                                    Character boxType,
                                    String reason,
                                    String verdict,
                                    Map<String, String> details) {
        add(new Event(
                frame,
                kind,
                severity,
                title,
                message,
                agentId,
                phase,
                subgoal,
                subgoalType,
                goal,
                boxType,
                null,
                null,
                null,
                null,
                reason,
                verdict,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                details
        ));
    }

    public PlanTrace copy() {
        PlanTrace copy = new PlanTrace();
        for (List<Event> events : eventsByFrame.values()) {
            for (Event event : events) {
                copy.add(event);
            }
        }
        copy.replacePortfolioAttempts(portfolioAttempts);
        return copy;
    }

    public PlanTrace copyUpTo(int actionCount) {
        PlanTrace copy = copy();
        copy.truncate(actionCount);
        return copy;
    }

    public PlanTrace shifted(int actionOffset) {
        PlanTrace shifted = new PlanTrace();
        for (List<Event> events : eventsByFrame.values()) {
            for (Event event : events) {
                shifted.add(event.withFrame(event.frame() + actionOffset));
            }
        }
        shifted.replacePortfolioAttempts(portfolioAttempts);
        return shifted;
    }

    public PlanTrace remapAgents(int[] originalIds) {
        PlanTrace remapped = new PlanTrace();
        for (List<Event> events : eventsByFrame.values()) {
            for (Event event : events) {
                remapped.add(event.remapAgents(originalIds));
            }
        }
        remapped.replacePortfolioAttempts(portfolioAttempts);
        return remapped;
    }

    public static PlanTrace mergeParallel(Collection<PlanTrace> traces) {
        PlanTrace merged = new PlanTrace();
        if (traces == null) return merged;
        for (PlanTrace trace : traces) {
            if (trace == null) continue;
            for (List<Event> events : trace.eventsByFrame.values()) {
                for (Event event : events) merged.add(event);
            }
        }
        return merged;
    }

    public static PlanTrace concat(PlanTrace first, PlanTrace second, int firstActionCount) {
        PlanTrace combined = new PlanTrace();
        if (first != null) {
            for (List<Event> events : first.eventsByFrame.values()) {
                for (Event event : events) combined.add(event);
            }
        }
        if (second != null) {
            PlanTrace shifted = second.shifted(firstActionCount);
            for (List<Event> events : shifted.eventsByFrame.values()) {
                for (Event event : events) combined.add(event);
            }
        }
        return combined;
    }

    public record PortfolioAttempt(int ordinal,
                                   String phase,
                                   String label,
                                   String strategy,
                                   String orderingMode,
                                   int randomSeed,
                                   long durationMs,
                                   boolean success,
                                   int planSteps,
                                   int unsatCount,
                                   String failedSubgoal,
                                   String failureKind,
                                   int reliefCount,
                                   int suspendedCount,
                                   int finalSatisfiedGoals,
                                   int finalTotalGoals,
                                   int finalSatisfiedBoxGoals,
                                   int finalTotalBoxGoals,
                                   String finalUnsatisfiedGoalsSample,
                                   String finalStateHash) {}

    public record Event(int frame,
                        String kind,
                        String severity,
                        String title,
                        String message,
                        Integer agentId,
                        String phase,
                        String subgoal,
                        String subgoalType,
                        Position goal,
                        Character boxType,
                        Integer stepInSegment,
                        Integer segmentSteps,
                        String action,
                        String actualAction,
                        String reason,
                        String verdict,
                        Boolean synthetic,
                        Integer agentReachBefore,
                        Integer agentReachAfter,
                        Integer totalReachBefore,
                        Integer totalReachAfter,
                        Boolean goalAdjBefore,
                        Boolean goalAdjAfter,
                        Map<String, String> details) {
        public Event {
            details = immutableDetails(details);
        }

        Event withFrame(int nextFrame) {
            return new Event(nextFrame, kind, severity, title, message, agentId, phase, subgoal,
                    subgoalType, goal, boxType, stepInSegment, segmentSteps, action,
                    actualAction, reason, verdict, synthetic, agentReachBefore, agentReachAfter,
                    totalReachBefore, totalReachAfter, goalAdjBefore, goalAdjAfter, details);
        }

        Event remapAgents(int[] originalIds) {
            if (originalIds == null || originalIds.length == 0) return this;
            Integer mappedAgent = agentId;
            if (agentId != null && agentId >= 0 && agentId < originalIds.length) {
                mappedAgent = originalIds[agentId];
            }
            return new Event(frame, kind, severity,
                    remapAgentText(title, originalIds),
                    remapAgentText(message, originalIds),
                    mappedAgent,
                    phase,
                    remapAgentText(subgoal, originalIds),
                    subgoalType, goal, boxType, stepInSegment, segmentSteps, action,
                    actualAction, reason, verdict, synthetic, agentReachBefore, agentReachAfter,
                    totalReachBefore, totalReachAfter, goalAdjBefore, goalAdjAfter, details);
        }

        private static String remapAgentText(String text, int[] originalIds) {
            if (text == null || originalIds == null) return text;
            String mapped = text;
            for (int i = originalIds.length - 1; i >= 0; i--) {
                mapped = mapped.replace("agent" + i, "agent" + originalIds[i]);
            }
            return mapped;
        }

        private static Map<String, String> immutableDetails(Map<String, String> source) {
            if (source == null || source.isEmpty()) {
                return Collections.emptyMap();
            }
            Map<String, String> copy = new LinkedHashMap<>();
            for (Map.Entry<String, String> entry : source.entrySet()) {
                if (entry.getKey() == null || entry.getValue() == null) continue;
                copy.put(entry.getKey(), Objects.toString(entry.getValue()));
            }
            return Collections.unmodifiableMap(copy);
        }
    }
}
