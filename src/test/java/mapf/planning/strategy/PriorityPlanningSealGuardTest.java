package mapf.planning.strategy;

import mapf.domain.Color;
import mapf.domain.Position;
import mapf.domain.State;
import mapf.planning.SearchConfig;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.testutil.TestLevelBuilder;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PriorityPlanningSealGuardTest {
    @Test
    @SuppressWarnings("unchecked")
    void completedGoalIsRejectedWhenItSealsRemainingPocketGoal() throws Exception {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("seal-pocket", 5, 7)
                .borderWalls()
                .fillInteriorWalls()
                .open(1, 1).open(2, 1).open(3, 1)
                .open(1, 2).open(2, 2).open(3, 2)
                .open(1, 3).open(2, 3).open(3, 3)
                .agent(0, Color.PINK, 2, 1)
                .box('K', Color.PINK, 1, 2)
                .box('L', Color.PINK, 2, 2)
                .box('K', Color.PINK, 3, 2)
                .boxGoal('K', 1, 2)
                .boxGoal('L', 2, 2)
                .boxGoal('K', 2, 3)
                .boxGoal('K', 3, 2)
                .build();

        PriorityPlanningStrategy strategy =
                new PriorityPlanningStrategy(new ManhattanHeuristic(), new SearchConfig());

        Field completed = PriorityPlanningStrategy.class.getDeclaredField("completedBoxGoals");
        completed.setAccessible(true);
        ((Set<Position>) completed.get(strategy)).add(Position.of(1, 2));
        ((Set<Position>) completed.get(strategy)).add(Position.of(3, 2));

        PriorityPlanningStrategy.Subgoal justFilled =
                new PriorityPlanningStrategy.Subgoal(0, 'L', Position.of(2, 2), false);
        PriorityPlanningStrategy.Subgoal remaining =
                new PriorityPlanningStrategy.Subgoal(0, 'K', Position.of(2, 3), false);

        Method guard = PriorityPlanningStrategy.class.getDeclaredMethod(
                "findSealRisk",
                PriorityPlanningStrategy.Subgoal.class,
                State.class,
                State.class,
                mapf.domain.Level.class,
                List.class);
        guard.setAccessible(true);

        Object risk = guard.invoke(strategy, justFilled, fixture.state(), fixture.state(),
                fixture.level(), List.of(justFilled, remaining));

        assertNotNull(risk, "filling the middle door goal must be reported as a seal risk");
        assertFalse(allowStaging(risk), "future box goals should be protected by ordering, not pre-staging");
    }

    @Test
    void boxGoalSealRiskIncludesTerminalAgentGoal() throws Exception {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("seal-agent-goal", 5, 7)
                .borderWalls()
                .fillInteriorWalls()
                .open(1, 3)
                .open(2, 1).open(2, 2).open(2, 3).open(2, 4).open(2, 5)
                .open(3, 3)
                .agent(0, Color.BLUE, 2, 5)
                .agent(1, Color.RED, 3, 3)
                .box('B', Color.RED, 1, 3)
                .boxGoal('B', 2, 3)
                .agentGoal(0, 2, 1)
                .build();

        Position[] agents = new Position[] { Position.of(2, 5), Position.of(3, 3) };
        Map<Position, Character> afterBoxes = new HashMap<>();
        afterBoxes.put(Position.of(2, 3), 'B');
        State afterSeal = new State(agents, afterBoxes);

        PriorityPlanningStrategy strategy =
                new PriorityPlanningStrategy(new ManhattanHeuristic(), new SearchConfig());
        PriorityPlanningStrategy.Subgoal justFilled =
                new PriorityPlanningStrategy.Subgoal(1, 'B', Position.of(2, 3), false);
        PriorityPlanningStrategy.Subgoal terminalAgent =
                new PriorityPlanningStrategy.Subgoal(0, '\0', Position.of(2, 1), true);

        Method guard = PriorityPlanningStrategy.class.getDeclaredMethod(
                "findSealRisk",
                PriorityPlanningStrategy.Subgoal.class,
                State.class,
                State.class,
                mapf.domain.Level.class,
                List.class);
        guard.setAccessible(true);

        Object risk = guard.invoke(strategy, justFilled, fixture.state(), afterSeal,
                fixture.level(), List.of(justFilled, terminalAgent));

        assertNotNull(risk, "box-goal closure must account for terminal agent-goal reachability");
        assertTrue(allowStaging(risk), "terminal agent goals should allow pre-staging before the seal");
    }

    private boolean allowStaging(Object risk) throws Exception {
        Field allowStaging = risk.getClass().getDeclaredField("allowStaging");
        allowStaging.setAccessible(true);
        return (boolean) allowStaging.get(risk);
    }
}
