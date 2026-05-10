package mapf.planning.strategy;

import mapf.domain.Action;
import mapf.domain.Color;
import mapf.domain.Direction;
import mapf.domain.Position;
import mapf.domain.State;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.testutil.TestLevelBuilder;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Method;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BoxSearchPlannerContractTest {
    @Test
    void targetBoxMayMoveThroughProtectedPositions() {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("target-protected", 5, 8)
                .borderWalls()
                .agent(0, Color.BLUE, 2, 1)
                .box('A', Color.BLUE, 2, 2)
                .build();

        BoxSearchPlanner planner = new BoxSearchPlanner(new ManhattanHeuristic());
        var path = planner.planBoxDisplacementWithUnfreeze(
                0,
                new Position(2, 2),
                new Position(2, 4),
                'A',
                fixture.state(),
                fixture.level(),
                Set.of(new Position(2, 2)),
                200,
                Set.of(new Position(2, 3)));

        assertFalse(path == null || path.isEmpty(), "target box itself must be allowed through protected cells");
    }

    @Test
    void nonTargetBoxCannotMoveFromOrIntoProtectedPositions() throws Exception {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("protected-nontarget", 5, 8)
                .borderWalls()
                .agent(0, Color.BLUE, 2, 1)
                .box('A', Color.BLUE, 2, 2)
                .box('A', Color.BLUE, 2, 3)
                .build();
        BoxSearchPlanner planner = new BoxSearchPlanner(new ManhattanHeuristic());
        Method method = BoxSearchPlanner.class.getDeclaredMethod(
                "wouldMoveBoxOnProtected",
                Action.class, int.class, State.class, Position.class, Set.class);
        method.setAccessible(true);

        boolean movingProtectedSource = (boolean) method.invoke(
                planner,
                Action.push(Direction.E, Direction.E),
                0,
                fixture.state(),
                new Position(2, 3),
                Set.of(new Position(2, 2)));

        assertTrue(movingProtectedSource, "non-target box must not be moved from a protected parking cell");
    }
}
