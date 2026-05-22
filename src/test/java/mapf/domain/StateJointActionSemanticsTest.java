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

    @Test
    void sanitizeJointActionMasksInapplicableActions() {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("sanitize-inapplicable", 5, 6)
                .borderWalls()
                .agent(0, Color.BLUE, 1, 1)
                .agent(1, Color.RED, 3, 1)
                .build();

        Action[] sanitized = fixture.state().sanitizeJointAction(new Action[] {
                Action.move(Direction.N),
                Action.move(Direction.E)
        }, fixture.level());

        assertEquals(Action.noOp(), sanitized[0]);
        assertEquals(Action.move(Direction.E), sanitized[1]);
    }

    @Test
    void sanitizeJointActionMasksConflictingActions() {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("sanitize-conflict", 5, 5)
                .borderWalls()
                .agent(0, Color.BLUE, 2, 1)
                .agent(1, Color.RED, 2, 3)
                .build();

        Action[] sanitized = fixture.state().sanitizeJointAction(new Action[] {
                Action.move(Direction.E),
                Action.move(Direction.W)
        }, fixture.level());

        assertEquals(Action.noOp(), sanitized[0]);
        assertEquals(Action.noOp(), sanitized[1]);
    }

    @Test
    void sanitizeJointActionMasksAllAgentsInMultiwayConflict() {
        TestLevelBuilder.BuiltLevel fixture = TestLevelBuilder.level("sanitize-multiway-conflict", 5, 5)
                .borderWalls()
                .agent(0, Color.BLUE, 2, 1)
                .agent(1, Color.RED, 2, 3)
                .agent(2, Color.CYAN, 1, 2)
                .build();

        Action[] sanitized = fixture.state().sanitizeJointAction(new Action[] {
                Action.move(Direction.E),
                Action.move(Direction.W),
                Action.move(Direction.S)
        }, fixture.level());

        assertEquals(Action.noOp(), sanitized[0]);
        assertEquals(Action.noOp(), sanitized[1]);
        assertEquals(Action.noOp(), sanitized[2]);
    }
}
