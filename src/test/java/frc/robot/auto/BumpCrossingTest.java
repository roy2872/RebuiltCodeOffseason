package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class BumpCrossingTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  @Test
  void crossesAtTheClosestDiagonalHeading() {
    for (double degrees : new double[] {0, 30, 89, 100, 180, -170, -60, 271}) {
      double snapped = BumpCrossing.nearestDiagonal(Rotation2d.fromDegrees(degrees)).getDegrees();
      double offDiagonal = Math.abs(Math.IEEEremainder(snapped - 45.0, 90.0));
      assertTrue(offDiagonal < 1e-9, degrees + " deg snapped to " + snapped + ", not 45 + 90k");
      double turn = Math.abs(Math.IEEEremainder(snapped - degrees, 360.0));
      assertTrue(turn <= 45.0 + 1e-9, degrees + " deg turned " + turn + " deg to cross");
    }
  }

  @Test
  void noCrossingInsideAZone() {
    assertTrue(
        BumpCrossing.plan(new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(3.0, 6.0, Rotation2d.kZero))
            .isEmpty());
  }

  @Test
  void crossesTheBlueLineIntoTheNeutralZoneOverTheCloserBump() {
    var crossings =
        BumpCrossing.plan(
            new Pose2d(2.0, 2.0, Rotation2d.k180deg), new Pose2d(7.0, 1.5, Rotation2d.kZero));
    assertEquals(1, crossings.size());
    var crossing = crossings.get(0);
    assertTrue(crossing.approach().getX() < BumpCrossing.BLUE_HUB_LINE_X);
    assertTrue(crossing.exit().getX() > BumpCrossing.BLUE_HUB_LINE_X);
    assertEquals(BumpCrossing.BUMP_Y[0], crossing.approach().getY(), 1e-9, "used the far bump");
    assertEquals(crossing.approach().getY(), crossing.exit().getY(), 1e-9, "crossing isn't straight");
  }

  @Test
  void crossesBothLinesInOrderToReachTheOtherAllianceZone() {
    List<BumpCrossing.Crossing> crossings =
        BumpCrossing.plan(
            new Pose2d(14.0, 6.0, Rotation2d.kZero), new Pose2d(2.0, 6.0, Rotation2d.kZero));
    assertEquals(2, crossings.size());
    assertTrue(crossings.get(0).exit().getX() < BumpCrossing.RED_HUB_LINE_X);
    assertTrue(crossings.get(1).exit().getX() < BumpCrossing.BLUE_HUB_LINE_X);
    assertEquals(BumpCrossing.BUMP_Y[1], crossings.get(0).approach().getY(), 1e-9);
  }

  @Test
  void approachAndExitAreClearOfTheLineAtAnyHeading() {
    var crossing =
        BumpCrossing.plan(new Pose2d(2.0, 4.0, Rotation2d.kZero), new Pose2d(8.0, 4.0, Rotation2d.kZero))
            .get(0);
    double lineHalfDepth = edu.wpi.first.math.util.Units.inchesToMeters(53.0) / 2.0;
    double clearance =
        Math.abs(crossing.approach().getX() - BumpCrossing.BLUE_HUB_LINE_X) - lineHalfDepth;
    assertTrue(clearance > frc.robot.subsystems.drive.DriveConstants.SWEPT_RADIUS);
  }

  @Test
  void simulatedGyroTiltsOnlyOnTheBump() {
    Rotation2d diagonal = Rotation2d.fromDegrees(45);
    double[] flat = BumpCrossing.simulatedTiltDegrees(new Pose2d(2.0, 2.0, diagonal));
    assertEquals(0.0, Math.hypot(flat[0], flat[1]), 1e-9);

    // Climbing: the robot's center halfway up the near ramp
    double maxTilt = 0.0;
    for (double x = -1.5; x <= 1.5; x += 0.02) {
      double[] tilt =
          BumpCrossing.simulatedTiltDegrees(
              new Pose2d(BumpCrossing.BLUE_HUB_LINE_X + x, BumpCrossing.BUMP_Y[0], diagonal));
      double magnitude = Math.hypot(tilt[0], tilt[1]);
      assertTrue(magnitude <= BumpCrossing.RAMP_DEGREES + 1e-6, "steeper than the ramp at " + x);
      maxTilt = Math.max(maxTilt, magnitude);
    }
    assertTrue(maxTilt > 7.0, "never tilted past the threshold: " + maxTilt);

    // Beside the hub, not a bump
    double[] hub =
        BumpCrossing.simulatedTiltDegrees(new Pose2d(BumpCrossing.BLUE_HUB_LINE_X, 4.0345, diagonal));
    assertEquals(0.0, Math.hypot(hub[0], hub[1]), 1e-9);
  }
}
