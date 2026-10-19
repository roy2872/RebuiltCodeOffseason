package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.HubShiftTracker.Phase;
import frc.robot.HubShiftTracker.State;
import org.junit.jupiter.api.Test;

class HubShiftTrackerTest {
  private static State teleop(double timeRemaining, String gameData, Alliance us) {
    return HubShiftTracker.compute(false, true, timeRemaining, gameData, us);
  }

  @Test
  void bothHubsActiveInAutoTransitionAndEndGame() {
    assertTrue(HubShiftTracker.compute(true, false, 15, "R", Alliance.Red).ourHubActive());
    assertEquals(Phase.TRANSITION, teleop(135, "R", Alliance.Red).phase());
    assertTrue(teleop(135, "R", Alliance.Red).ourHubActive());
    assertEquals(Phase.END_GAME, teleop(20, "R", Alliance.Red).phase());
    assertTrue(teleop(20, "R", Alliance.Red).ourHubActive());
  }

  @Test
  void shiftsAlternateStartingWithTheGameDataAlliance() {
    // Red's hub is inactive in shift 1
    assertEquals(Phase.SHIFT_1, teleop(120, "R", Alliance.Red).phase());
    assertFalse(teleop(120, "R", Alliance.Red).ourHubActive());
    assertTrue(teleop(120, "R", Alliance.Blue).ourHubActive());

    assertEquals(Phase.SHIFT_2, teleop(95, "R", Alliance.Red).phase());
    assertTrue(teleop(95, "R", Alliance.Red).ourHubActive());
    assertFalse(teleop(95, "R", Alliance.Blue).ourHubActive());

    assertEquals(Phase.SHIFT_3, teleop(70, "R", Alliance.Red).phase());
    assertFalse(teleop(70, "R", Alliance.Red).ourHubActive());

    assertEquals(Phase.SHIFT_4, teleop(40, "R", Alliance.Red).phase());
    assertTrue(teleop(40, "R", Alliance.Red).ourHubActive());
  }

  @Test
  void countsDownToTheNextChangeOfOurHub() {
    // Transition, our hub goes inactive at 130
    assertEquals(5.0, teleop(135, "B", Alliance.Blue).secondsUntilChange(), 1e-9);
    // Other alliance inactive first: active through transition and shift 1, inactive at 105
    assertEquals(30.0, teleop(135, "B", Alliance.Red).secondsUntilChange(), 1e-9);
    // Active in shift 4, stays active through the end game
    assertTrue(Double.isInfinite(teleop(40, "R", Alliance.Red).secondsUntilChange()));
  }

  @Test
  void missingGameDataAssumesActive() {
    State empty = teleop(120, "", Alliance.Red);
    assertTrue(empty.ourHubActive());
    assertFalse(empty.gameDataValid());
    assertTrue(teleop(120, "X", Alliance.Red).ourHubActive());
    assertTrue(teleop(120, null, Alliance.Red).ourHubActive());
  }

  @Test
  void acceptsLowercaseGameData() {
    assertFalse(teleop(120, "b", Alliance.Blue).ourHubActive());
  }
}
