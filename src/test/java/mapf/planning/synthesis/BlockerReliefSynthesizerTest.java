package mapf.planning.synthesis;

import mapf.client.LevelParser;
import mapf.domain.Position;
import mapf.planning.strategy.ArticulationPointFinder;
import mapf.planning.strategy.PriorityPlanningStrategy;
import mapf.testutil.LevelFixtures;
import org.junit.jupiter.api.Test;

import java.nio.file.Path;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BlockerReliefSynthesizerTest {
    @Test
    void smallFixtureNeverSynthesizesUncertifiedReliefTargets() throws Exception {
        LevelParser.ParseResult fixture = LevelFixtures.parse(Path.of("levels", "MAsimple3.lvl"));

        BlockerReliefSynthesizer.ReliefResult result =
                BlockerReliefSynthesizer.synthesizeWithMeta(
                        fixture.initialState, fixture.level, Collections.emptySet());

        for (PriorityPlanningStrategy.Subgoal relief : result.reliefs) {
            assertTrue(relief.isSyntheticRelief(), "NAMO relief must carry an obstruction certificate");
            assertNotNull(relief.reliefCertificate.blockerStart,
                    "obstruction certificate must record the original blocker position");
            assertNotEquals('\0', fixture.initialState.getBoxAt(relief.reliefCertificate.blockerStart),
                    "obstruction certificate blockerStart must point to an initial box");
            Position target = relief.goalPos;
            assertFalse(fixture.initialState.hasBoxAt(target), "relief target must not be initially occupied");
            assertFalse(fixture.level.getBoxGoal(target) != '\0', "relief target must not be a box goal");
            assertNotEquals(-1, relief.agentId, "relief must be assigned to a helper agent");
        }
    }

    @Test
    void isoReliefTargetsDoNotUseArticulationDoorCells() throws Exception {
        LevelParser.ParseResult fixture = LevelFixtures.parse(Path.of("complevels", "ISO.lvl"));

        BlockerReliefSynthesizer.ReliefResult result =
                BlockerReliefSynthesizer.synthesizeWithMeta(
                        fixture.initialState, fixture.level, Collections.emptySet());

        Set<Position> staticDoorCells =
                ArticulationPointFinder.findArticulationPoints(fixture.level, Collections.emptySet());
        Set<Position> currentDoorCells =
                ArticulationPointFinder.findArticulationPoints(
                        fixture.level, new HashSet<>(fixture.initialState.getBoxes().keySet()));
        assertFalse(result.reliefs.isEmpty(), "ISO should synthesize blocker-relief subgoals");
        assertTrue(staticDoorCells.contains(new Position(1, 3)),
                "(1,3) must be recognized as a static door cell in ISO");
        assertTrue(currentDoorCells.contains(new Position(1, 3)),
                "(1,3) must be recognized as a current-state door cell in ISO");

        for (PriorityPlanningStrategy.Subgoal relief : result.reliefs) {
            assertTrue(relief.isSyntheticRelief(), "ISO NAMO relief must carry obstruction evidence");
            assertNotEquals(new Position(1, 3), relief.goalPos,
                    "L@(1,3) is a door cell into the E/G pocket and must not be used as P_temp");
            assertFalse(staticDoorCells.contains(relief.goalPos),
                    "relief target must not use a static door cell when safer parking exists");
        }
    }
}
