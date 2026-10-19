package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionIO.ObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class VisionFilterTest {
  private static final VisionFilter.Context STILL_ENABLED =
      new VisionFilter.Context(true, 0.0, 0.0, 0.03, false);
  private static final VisionFilter.Context STILL_DISABLED =
      new VisionFilter.Context(false, 0.0, 0.0, 0.03, false);

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  private static PoseObservation observation(
      ObservationType type, int tagCount, double distance, double ambiguity, double onEdge) {
    return new PoseObservation(
        0.0,
        new Pose3d(5.0, 4.0, 0.0, Rotation3d.kZero),
        ambiguity,
        tagCount,
        distance,
        distance,
        0.1,
        onEdge,
        type);
  }

  @Test
  void picksMegatag2WhileEnabledAndMegatag1WhileDisabled() {
    var mt1 = observation(ObservationType.MEGATAG_1, 2, 2.0, 0.0, 0.0);
    var mt2 = observation(ObservationType.MEGATAG_2, 2, 2.0, 0.0, 0.0);
    assertTrue(VisionFilter.rejectReason(mt1, STILL_ENABLED).isPresent());
    assertTrue(VisionFilter.rejectReason(mt2, STILL_ENABLED).isEmpty());
    assertTrue(VisionFilter.rejectReason(mt1, STILL_DISABLED).isEmpty());
    assertTrue(VisionFilter.rejectReason(mt2, STILL_DISABLED).isPresent());
  }

  @Test
  void rejectsFarSingleTagsUnlessMegatag2() {
    assertEquals(
        Optional.of("SingleTagTooFar"),
        VisionFilter.rejectReason(
            observation(ObservationType.PHOTONVISION, 1, 5.0, 0.0, 0.0), STILL_ENABLED));
    assertTrue(
        VisionFilter.rejectReason(
                observation(ObservationType.MEGATAG_2, 1, 5.0, 0.9, 0.0), STILL_ENABLED)
            .isEmpty());
    assertEquals(
        Optional.of("Ambiguity"),
        VisionFilter.rejectReason(
            observation(ObservationType.PHOTONVISION, 1, 2.0, 0.9, 0.0), STILL_ENABLED));
  }

  @Test
  void rejectsMegatag2WhileSpinningFast() {
    var spinning = new VisionFilter.Context(true, 0.0, Math.toRadians(900), 0.03, false);
    assertEquals(
        Optional.of("SpinningTooFast"),
        VisionFilter.rejectReason(observation(ObservationType.MEGATAG_2, 2, 2.0, 0.0, 0.0), spinning));
  }

  @Test
  void trustDropsWithDistanceMotionAgeAndDisturbance() {
    var obs = observation(ObservationType.PHOTONVISION, 2, 2.0, 0.0, 0.0);
    double base = VisionFilter.stdDevs(obs, 1.0, STILL_ENABLED).linear();

    var far = observation(ObservationType.PHOTONVISION, 2, 5.0, 0.0, 0.0);
    assertTrue(VisionFilter.stdDevs(far, 1.0, STILL_ENABLED).linear() > base);

    var moreTags = observation(ObservationType.PHOTONVISION, 4, 2.0, 0.0, 0.0);
    assertTrue(VisionFilter.stdDevs(moreTags, 1.0, STILL_ENABLED).linear() < base);

    var nearEdge = observation(ObservationType.PHOTONVISION, 2, 2.0, 0.0, 0.8);
    assertTrue(VisionFilter.stdDevs(nearEdge, 1.0, STILL_ENABLED).linear() > base);

    var moving = new VisionFilter.Context(true, 3.0, 2.0, 0.03, false);
    assertTrue(VisionFilter.stdDevs(obs, 1.0, moving).linear() > base);

    var old = new VisionFilter.Context(true, 0.0, 0.0, 0.3, false);
    assertTrue(VisionFilter.stdDevs(obs, 1.0, old).linear() > base);

    var disturbed = new VisionFilter.Context(true, 0.0, 0.0, 0.03, true);
    assertTrue(VisionFilter.stdDevs(obs, 1.0, disturbed).linear() > base);
  }

  @Test
  void megatag2NeverCorrectsHeading() {
    var mt2 = observation(ObservationType.MEGATAG_2, 3, 1.5, 0.0, 0.0);
    assertTrue(Double.isInfinite(VisionFilter.stdDevs(mt2, 1.0, STILL_ENABLED).angular()));
  }

  @Test
  void gateRejectsAnOutlierButRecoversWhenTheEstimateIsWrong() {
    var gate = new VisionFilter.InnovationGate();
    Optional<Pose2d> estimate = Optional.of(new Pose2d(5.0, 4.0, Rotation2d.kZero));
    Pose2d close = new Pose2d(5.1, 4.0, Rotation2d.kZero);
    Pose2d far = new Pose2d(7.0, 4.0, Rotation2d.kZero);

    assertTrue(gate.accept(close, estimate, 0.03, true));
    assertFalse(gate.accept(far, estimate, 0.03, true), "a jump of 2 m should be rejected");

    // The same far pose keeps arriving: the estimate is the one that's wrong
    boolean recovered = false;
    for (int i = 0; i < 20 && !recovered; i++) {
      recovered = gate.accept(far, estimate, 0.03, true);
    }
    assertTrue(recovered, "agreeing poses never got through");

    // Never gated while disabled
    assertTrue(new VisionFilter.InnovationGate().accept(far, estimate, 0.03, false));
  }
}
