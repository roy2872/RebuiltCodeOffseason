package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Crossing a hub line over a bump. The robot can't fit in the trench, so this is the only way
 * between the alliance zones and the neutral zone; the pathfinder's navgrid treats each hub line as
 * a wall (tools/generate_navgrid.py).
 *
 * <p>A crossing pathfinds to a spot in front of the bump, turned to a diagonal heading (45 + 90k
 * degrees, whichever is closest to the current heading) so the wheels climb one at a time, then
 * drives straight over it slowly at that heading.
 */
public final class BumpCrossing {
  private BumpCrossing() {}

  public static final double FIELD_LENGTH = 16.541;
  public static final double FIELD_WIDTH = 8.052;
  /** Hub line centers, field coordinates (from maple-sim's Arena2026Rebuilt). */
  public static final double BLUE_HUB_LINE_X = 4.5974;
  public static final double RED_HUB_LINE_X = FIELD_LENGTH - BLUE_HUB_LINE_X;
  // Each hub line: the hub (47 in) in the middle, a bump on either side, 217 in end to end
  private static final double HUB_Y = 4.034536;
  private static final double HUB_HALF_WIDTH = Units.inchesToMeters(47.0) / 2.0;
  private static final double RAMPS_HALF_WIDTH = Units.inchesToMeters(217.0) / 2.0;
  private static final double BUMP_OFFSET_FROM_HUB = (HUB_HALF_WIDTH + RAMPS_HALF_WIDTH) / 2.0;
  /** Bump centers along y, the same on both hub lines (the field is rotationally symmetric). */
  public static final double[] BUMP_Y = {
    HUB_Y - BUMP_OFFSET_FROM_HUB, HUB_Y + BUMP_OFFSET_FROM_HUB
  };
  /** Half the bump's depth along x, from the hub line to the foot of a ramp. */
  public static final double BUMP_HALF_DEPTH = Units.inchesToMeters(53.0) / 2.0;
  /** Ramp angle from the game manual. */
  public static final double RAMP_DEGREES = 15.0;
  /** Start/end of the straight crossing from the line center: clear of the line at any heading. */
  public static final double APPROACH_OFFSET = BUMP_HALF_DEPTH + DriveConstants.SWEPT_RADIUS + 0.1;

  private static final TunableNumber crossingSpeed =
      new TunableNumber("Auto/BumpCrossing/SpeedMetersPerSec", 1.5);
  private static final TunableNumber crossingAccel =
      new TunableNumber("Auto/BumpCrossing/AccelMetersPerSecSq", 2.0);

  /** Constraints for pathfinding between crossings. */
  public static final PathConstraints PATHFIND_CONSTRAINTS =
      new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /**
   * One bump crossing, field coordinates.
   *
   * @param heading the robot's heading the whole way over (45 + 90k degrees)
   */
  public record Crossing(Translation2d approach, Translation2d exit, Rotation2d heading) {
    public Pose2d approachPose() {
      return new Pose2d(approach, heading);
    }

    public Pose2d exitPose() {
      return new Pose2d(exit, heading);
    }
  }

  /** The hub lines between two x positions, in the order they are crossed. */
  public static List<Double> linesBetween(double fromX, double toX) {
    List<Double> lines = new ArrayList<>();
    for (double line : fromX < toX
        ? new double[] {BLUE_HUB_LINE_X, RED_HUB_LINE_X}
        : new double[] {RED_HUB_LINE_X, BLUE_HUB_LINE_X}) {
      if ((fromX - line) * (toX - line) < 0) {
        lines.add(line);
      }
    }
    return lines;
  }

  /** The diagonal heading (45 + 90k degrees) closest to {@code heading}. */
  public static Rotation2d nearestDiagonal(Rotation2d heading) {
    double degrees = heading.getDegrees();
    double snapped = Math.round((degrees - 45.0) / 90.0) * 90.0 + 45.0;
    return Rotation2d.fromDegrees(MathUtil.inputModulus(snapped, -180.0, 180.0));
  }

  /**
   * The crossings needed to get from {@code from} to {@code to}, field coordinates. Each one uses the
   * bump that keeps the trip shortest. Empty when both are in the same zone.
   */
  public static List<Crossing> plan(Pose2d from, Pose2d to) {
    List<Crossing> crossings = new ArrayList<>();
    Rotation2d heading = nearestDiagonal(from.getRotation());
    Translation2d position = from.getTranslation();
    for (double lineX : linesBetween(from.getX(), to.getX())) {
      double bumpY = bestBump(position.getY(), to.getY());
      double direction = Math.signum(to.getX() - lineX);
      Translation2d approach = new Translation2d(lineX - direction * APPROACH_OFFSET, bumpY);
      Translation2d exit = new Translation2d(lineX + direction * APPROACH_OFFSET, bumpY);
      crossings.add(new Crossing(approach, exit, heading));
      position = exit;
    }
    return crossings;
  }

  /** Crossing the hub line nearest to {@code from}, over the closer bump (driver assist). */
  public static Crossing crossNearestLine(Pose2d from) {
    double lineX =
        Math.abs(from.getX() - BLUE_HUB_LINE_X) < Math.abs(from.getX() - RED_HUB_LINE_X)
            ? BLUE_HUB_LINE_X
            : RED_HUB_LINE_X;
    double otherSideX = lineX + Math.signum(lineX - from.getX()) * APPROACH_OFFSET;
    return plan(from, new Pose2d(otherSideX, from.getY(), from.getRotation())).get(0);
  }

  /** Teleop assist: while held, lines up with the nearest bump and drives over it. */
  public static Command crossNearestCommand() {
    return Commands.defer(
            () -> command(crossNearestLine(frc.robot.RobotState.mInstance.getEstimatedPose())),
            Set.of(Drive.mInstance))
        .withName("Cross Nearest Bump");
  }

  private static double bestBump(double fromY, double toY) {
    double best = BUMP_Y[0];
    for (double y : BUMP_Y) {
      if (Math.abs(fromY - y) + Math.abs(toY - y) < Math.abs(fromY - best) + Math.abs(toY - best)) {
        best = y;
      }
    }
    return best;
  }

  /**
   * Pathfinds to the crossing's start at its heading, then drives straight over the bump slowly
   * (1-2 m/s). The crossing ends on the gyro, not the pose: see {@link BumpCrossCommand}.
   */
  public static Command command(Crossing crossing) {
    return AutoBuilder.pathfindToPose(crossing.approachPose(), PATHFIND_CONSTRAINTS)
        // Deferred so it uses the crossing speed tuned at the time it runs
        .andThen(
            Commands.defer(
                () ->
                    new BumpCrossCommand(
                        crossing,
                        MathUtil.clamp(crossingSpeed.getAsDouble(), 1.0, 2.0),
                        crossingAccel.getAsDouble()),
                Set.of(Drive.mInstance)))
        .withName("Cross Bump");
  }

  /** Bump height at {@code position} (0 off the bumps): a ridge along each hub line. */
  public static double bumpHeight(Translation2d position) {
    double fromHub = Math.abs(position.getY() - HUB_Y);
    if (fromHub < HUB_HALF_WIDTH || fromHub > RAMPS_HALF_WIDTH) return 0.0;
    double fromLine =
        Math.min(
            Math.abs(position.getX() - BLUE_HUB_LINE_X), Math.abs(position.getX() - RED_HUB_LINE_X));
    return Math.max(0.0, BUMP_HALF_DEPTH - fromLine) * Math.tan(Math.toRadians(RAMP_DEGREES));
  }

  /**
   * The gyro's {pitch, roll} in degrees for the robot at {@code pose}, from the bump height under each
   * wheel. maple-sim is flat, so the simulated gyro uses this.
   */
  public static double[] simulatedTiltDegrees(Pose2d pose) {
    double[] height = new double[4];
    for (int i = 0; i < 4; i++) {
      Translation2d wheel =
          pose.getTranslation()
              .plus(DriveConstants.moduleTranslations[i].rotateBy(pose.getRotation()));
      height[i] = bumpHeight(wheel);
    }
    // Modules: front left, front right, back left, back right
    Translation2d[] modules = DriveConstants.moduleTranslations;
    double wheelBase = modules[0].getX() - modules[2].getX();
    double trackWidth = modules[0].getY() - modules[1].getY();
    double pitch = Math.atan2((height[0] + height[1] - height[2] - height[3]) / 2.0, wheelBase);
    double roll = Math.atan2((height[0] + height[2] - height[1] - height[3]) / 2.0, trackWidth);
    return new double[] {Math.toDegrees(pitch), Math.toDegrees(roll)};
  }
}
