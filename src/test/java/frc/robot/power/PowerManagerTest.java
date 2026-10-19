package frc.robot.power;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.power.PowerManager.ModeSelection;
import frc.robot.power.PowerManager.Profile;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class PowerManagerTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  @Test
  void autoModeFollowsTheRobot() {
    assertEquals(Profile.NORMAL, PowerManager.select(ModeSelection.AUTO, false, false, false, false));
    assertEquals(Profile.SCORING, PowerManager.select(ModeSelection.AUTO, false, false, false, true));
    // A sagging battery wins over shooting
    assertEquals(Profile.LOW_BATTERY, PowerManager.select(ModeSelection.AUTO, false, false, true, true));
  }

  @Test
  void defenseFromTheChooserOrTheSwitch() {
    assertEquals(Profile.DEFENSE, PowerManager.select(ModeSelection.DEFENSE, false, false, true, true));
    assertEquals(Profile.DEFENSE, PowerManager.select(ModeSelection.AUTO, true, false, false, false));
  }

  @Test
  void normalModeAndDisablingPinNormal() {
    assertEquals(Profile.NORMAL, PowerManager.select(ModeSelection.NORMAL, false, false, true, true));
    assertEquals(Profile.NORMAL, PowerManager.select(ModeSelection.DEFENSE, true, true, true, true));
  }
}
