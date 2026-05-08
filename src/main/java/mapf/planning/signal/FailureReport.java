package mapf.planning.signal;

import mapf.domain.Position;
import mapf.planning.strategy.PriorityPlanningStrategy.Subgoal;
import java.util.ArrayList;
import java.util.List;

/**
 * Structured failure signal produced by a planning layer when it cannot complete
 * its task. Per qanda.txt §5.2 ("失败信号自下而上传递") and claudeopus47.txt §3.2.2
 * ("subgoal-level 修复优先于 action-level 约束"), each layer should report a
 * specific actionable cause instead of "no plan found", so an upper layer can
 * decide what to retry (e.g. SubgoalManager swaps ordering, Portfolio picks a
 * different seed).
 *
 * <p>P0a (cheapest path): producers populate this; consumers only log. P0b/P0c
 * will introduce SubgoalManager / action-level constraint consumers.
 *
 * <p>Immutable value object. No behavior, only data + a human-readable summary.
 */
public final class FailureReport {

    /** Coarse cause categorisation. Keep small; expand only when a consumer needs it. */
    public enum Kind {
        /** PP loop exited stuck — no progress for the early-exit window. */
        STUCK_NO_PROGRESS,
        /** Goal not reached but a non-empty partial plan was returned. */
        PARTIAL_PLAN,
        /** Goal not reached and no partial plan was found. */
        NO_PLAN,
        /** Strategy threw an exception. */
        EXCEPTION,
        /** Strategy hit its wall-clock or state budget. */
        BUDGET_EXHAUSTED
    }

    public final Kind kind;
    /** Last subgoal the producer was trying when it gave up. May be null. */
    public final Subgoal lastAttemptedSubgoal;
    /** Snapshot of subgoals still unsatisfied at the failure moment. */
    public final List<Subgoal> unsatisfiedAtFailure;
    /** Goal positions the producer believes are blocking progress (may be empty). */
    public final List<Position> blockedGoals;
    /** Free-form note for logging only. May be null. */
    public final String note;

    private FailureReport(Kind kind,
                          Subgoal lastAttemptedSubgoal,
                          List<Subgoal> unsatisfiedAtFailure,
                          List<Position> blockedGoals,
                          String note) {
        this.kind = kind;
        this.lastAttemptedSubgoal = lastAttemptedSubgoal;
        this.unsatisfiedAtFailure = unsatisfiedAtFailure == null
                ? List.of() : List.copyOf(unsatisfiedAtFailure);
        this.blockedGoals = blockedGoals == null
                ? List.of() : List.copyOf(blockedGoals);
        this.note = note;
    }

    public static FailureReport of(Kind kind, String note) {
        return new FailureReport(kind, null, null, null, note);
    }

    public static FailureReport stuck(Subgoal last, List<Subgoal> unsatisfied, String note) {
        return new FailureReport(Kind.STUCK_NO_PROGRESS, last, unsatisfied, null, note);
    }

    public static FailureReport partial(Subgoal last, List<Subgoal> unsatisfied, String note) {
        return new FailureReport(Kind.PARTIAL_PLAN, last, unsatisfied, null, note);
    }

    public static FailureReport noPlan(Subgoal last, List<Subgoal> unsatisfied, String note) {
        return new FailureReport(Kind.NO_PLAN, last, unsatisfied, null, note);
    }

    /** One-line summary suitable for logging. */
    public String summary() {
        StringBuilder sb = new StringBuilder();
        sb.append(kind);
        if (lastAttemptedSubgoal != null) {
            sb.append(" lastSubgoal=agent").append(lastAttemptedSubgoal.agentId)
              .append("->").append(lastAttemptedSubgoal.boxType)
              .append('@').append(lastAttemptedSubgoal.goalPos);
        }
        if (!unsatisfiedAtFailure.isEmpty()) {
            sb.append(" unsatisfied=").append(unsatisfiedAtFailure.size());
        }
        if (!blockedGoals.isEmpty()) {
            sb.append(" blocked=").append(blockedGoals);
        }
        if (note != null && !note.isEmpty()) {
            sb.append(" (").append(note).append(')');
        }
        return sb.toString();
    }

    @Override
    public String toString() {
        return "FailureReport{" + summary() + "}";
    }

    /** Convenience: snapshot a list defensively. */
    public static List<Subgoal> snapshot(List<Subgoal> src) {
        return src == null ? List.of() : new ArrayList<>(src);
    }
}
