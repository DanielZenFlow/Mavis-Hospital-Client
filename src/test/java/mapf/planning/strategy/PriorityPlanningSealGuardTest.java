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
import java.util.List;
import java.util.Set;

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
                "wouldSealRemainingGoals",
                PriorityPlanningStrategy.Subgoal.class,
                State.class,
                mapf.domain.Level.class,
                List.class);
        guard.setAccessible(true);

        boolean rejected = (boolean) guard.invoke(strategy, justFilled, fixture.state(),
                fixture.level(), List.of(justFilled, remaining));

        assertTrue(rejected, "filling the middle door goal must not seal the remaining pocket goal");
    }
}
