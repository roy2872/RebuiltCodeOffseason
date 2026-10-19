package frc.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Drives straight over a bump at a constant heading and ends on the gyro, not the pose (the idea is
 * from team 8214's BumpPounceController). The wheels slip and measure distance along the slope, so
 * on the bump the pose is least trusted exactly when a path follower would need it to decide it has
 * arrived.
 *
 * <p>It drives along the crossing at a constant speed, holds the heading and steers back onto the
 * crossing's line. Once it has been tilted and then level for a moment, all four wheels are back on
 * the carpet, and it drives the short distance left to the exit measured from there. Without a
 * working gyro, or if it never tilts, it ends by distance like a path would.
 */
public class BumpCrossCommand extends Command {
  private static final TunableNumber lateralKp =
      new TunableNumber("Auto/BumpCrossing/LateralKp", 2.0);
  private static final TunableNumber maxLateralSpeed =
      new TunableNumber("Auto/BumpCrossing/MaxLateralSpeed", 0.5);
  private static final TunableNumber levelSeconds =
      new TunableNumber("Auto/BumpCrossing/LevelSeconds", 0.2);
  /** Past the planned end with no bump seen: stop rather than keep going. */
  private static final TunableNumber overshootMeters =
      new TunableNumber("Auto/BumpCrossing/OvershootMeters", 0.5);

  /** Tilt only counts this far into the crossing, so a bump of the robot at the start doesn't. */
  private static final double ARM_DISTANCE = 0.25;
  /** From the line, where the farthest wheel leaves the bump. */
  private static final double WHEELS_OFF_DISTANCE =
      BumpCrossing.BUMP_HALF_DEPTH + DriveConstants.moduleTranslations[0].getNorm();
  /** Once level: the rest of the way to the exit. */
  private static final double CLEAR_DISTANCE =
      Math.max(0.0, BumpCrossing.APPROACH_OFFSET - WHEELS_OFF_DISTANCE);

  private final BumpCrossing.Crossing crossing;
  private final double speed;
  private final SlewRateLimiter accelLimiter;
  private final PIDController headingController =
      new PIDController(DriveConstants.PATH_ROTATION_KP, 0.0, 0.0);
  private final Timer timer = new Timer();

  private Translation2d direction;
  private Translation2d start;
  private double plannedDistance;
  private double timeoutSeconds;
  private boolean sawTilt;
  private double levelSince;
  private double landedProgress;
  private String endReason;
  private static String lastEndReason = "";

  public BumpCrossCommand(BumpCrossing.Crossing crossing, double speed, double accel) {
    this.crossing = crossing;
    this.speed = speed;
    this.accelLimiter = new SlewRateLimiter(accel);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(Drive.mInstance);
    setName("Bump Cross");
  }

  @Override
  public void initialize() {
    Translation2d travel = crossing.exit().minus(crossing.approach());
    direction = travel.div(travel.getNorm());
    start = RobotState.mInstance.getEstimatedPose().getTranslation();
    plannedDistance = progress(crossing.exit());
    timeoutSeconds = plannedDistance / speed * 2.0 + 2.0;
    accelLimiter.reset(0.0);
    sawTilt = false;
    levelSince = Double.NaN;
    landedProgress = Double.NaN;
    endReason = "";
    timer.restart();
  }

  @Override
  public void execute() {
    Pose2d pose = RobotState.mInstance.getEstimatedPose();
    double progress = progress(pose.getTranslation());
    double now = timer.get();

    boolean tilted = RobotState.mInstance.isTilted();
    if (tilted && progress >= ARM_DISTANCE) {
      sawTilt = true;
    }
    if (sawTilt && Double.isNaN(landedProgress)) {
      if (tilted) {
        levelSince = Double.NaN;
      } else if (Double.isNaN(levelSince)) {
        levelSince = now;
      } else if (now - levelSince >= levelSeconds.getAsDouble()) {
        landedProgress = progress;
      }
    }

    // Steer back onto the crossing's line, sideways to the direction of travel
    Translation2d normal = new Translation2d(-direction.getY(), direction.getX());
    double crossTrack = pose.getTranslation().minus(crossing.approach()).dot(normal);
    double maxLateral = maxLateralSpeed.getAsDouble();
    double lateral = MathUtil.clamp(-lateralKp.getAsDouble() * crossTrack, -maxLateral, maxLateral);
    double forward = accelLimiter.calculate(speed);
    Translation2d velocity = direction.times(forward).plus(normal.times(lateral));
    double omega =
        headingController.calculate(
            pose.getRotation().getRadians(), crossing.heading().getRadians());

    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), omega, pose.getRotation()));

    Logger.recordOutput("Auto/BumpCrossing/Progress", progress);
    Logger.recordOutput("Auto/BumpCrossing/PlannedDistance", plannedDistance);
    Logger.recordOutput("Auto/BumpCrossing/SawTilt", sawTilt);
    Logger.recordOutput("Auto/BumpCrossing/Landed", !Double.isNaN(landedProgress));
  }

  @Override
  public boolean isFinished() {
    double progress = progress(RobotState.mInstance.getEstimatedPose().getTranslation());
    if (!Double.isNaN(landedProgress)) {
      return finish(progress - landedProgress >= CLEAR_DISTANCE, "Landed");
    }
    if (!sawTilt && !Drive.mInstance.isGyroConnected()) {
      return finish(progress >= plannedDistance, "NoGyroDistance");
    }
    if (progress >= plannedDistance + overshootMeters.getAsDouble()) {
      return finish(true, sawTilt ? "NeverLevel" : "NeverTilted");
    }
    return finish(timer.hasElapsed(timeoutSeconds), "Timeout");
  }

  private boolean finish(boolean done, String reason) {
    if (done) {
      endReason = reason;
    }
    return done;
  }

  @Override
  public void end(boolean interrupted) {
    Drive.mInstance.stop();
    lastEndReason = interrupted ? "Interrupted" : endReason;
    Logger.recordOutput("Auto/BumpCrossing/EndReason", lastEndReason);
  }

  /** Why the last crossing ended: Landed, NoGyroDistance, NeverTilted, NeverLevel or Timeout. */
  public static String lastEndReason() {
    return lastEndReason;
  }

  /** How far along the crossing {@code position} is from where this crossing started. */
  private double progress(Translation2d position) {
    return position.minus(start).dot(direction);
  }
}
