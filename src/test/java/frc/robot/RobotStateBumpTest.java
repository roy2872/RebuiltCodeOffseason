package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** On the bump vision is rejected; right after it odometry is trusted less. */
class RobotStateBumpTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  @Test
  void rejectsImagesFromTheBumpAndRecoversAfterLanding() {
    RobotState state = RobotState.mInstance;
    double t = 1000.0;
    for (; t < 1001.0; t += 0.02) state.updateTilt(0.0, t); // flat
    double climbed = t;
    for (; t < 1001.5; t += 0.02) state.updateTilt(12.0, t); // on the bump
    double landed = t;
    assertTrue(state.isTilted());
    for (; t < 1001.7; t += 0.02) state.updateTilt(1.0, t); // back on the carpet

    assertFalse(state.isTilted());
    assertTrue(state.wasTiltedAt(climbed + 0.2), "image from the bump was kept");
    assertFalse(state.wasTiltedAt(climbed - 0.5), "image from before the bump was rejected");
    assertFalse(state.wasTiltedAt(landed + 0.15), "image from after landing was rejected");
    assertTrue(state.isRecoveringFromBump());

    for (; t < 1004.0; t += 0.02) state.updateTilt(0.0, t);
    assertFalse(state.isRecoveringFromBump());
  }
}
