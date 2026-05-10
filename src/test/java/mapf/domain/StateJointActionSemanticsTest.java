package mapf.domain;

import mapf.testutil.TestLevelBuilder;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class StateJointActionSemanticsTest {
    @Test
    void conflictingAgentsBothNoOp() {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("conflict", 5, 5)
                .borderWalls()
                .agent(0, Color.BLUE, 2, 1)
                .agent(1, Color.RED, 2, 3)
                .build();

        Action[] joint = {
                Action.move(Direction.E),
                Action.move(Direction.W)
        };

        State next = fixture.state().applyJointAction(joint, fixture.level());

        assertEquals(new Position(2, 1), next.getAgentPosition(0));
        assertEquals(new Position(2, 3), next.getAgentPosition(1));
    }

    @Test
    void inapplicableActionIsNoOpForThatAgentOnly() {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("inapplicable", 5, 6)
                .borderWalls()
                .agent(0, Color.BLUE, 1, 1)
                .agent(1, Color.RED, 3, 1)
                .build();

        Action[] joint = {
                Action.move(Direction.N),
                Action.move(Direction.E)
        };

        State next = fixture.state().applyJointAction(joint, fixture.level());

        assertEquals(new Position(1, 1), next.getAgentPosition(0));
        assertEquals(new Position(3, 2), next.getAgentPosition(1));
    }
}
