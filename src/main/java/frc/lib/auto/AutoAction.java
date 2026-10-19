package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * One part of an auto routine: optionally drive somewhere, doing something on the way, then do
 * something there. Actions are built once and can be chosen for any part of a routine.
 *
 * <pre>
 *   AutoAction.named("Intake Depot")
 *       .driveTo(new Pose2d(1.3, 6.0, Rotation2d.k180deg))
 *       .whileDriving(superstructure::intakeCommand)
 *       .atTarget(() -> creepForward().withTimeout(1.5), 1.5)
 *       .timeout(5.0)
 *       .build();
 * </pre>
 *
 * <p>Requirements: {@code whileDriving} runs alongside the drive and must not use the drivetrain;
 * {@code atTarget} runs after it and may use anything.
 */
public final class AutoAction {
  private final String name;
  private final Optional<Pose2d> blueTarget;
  private final Optional<String> pathName;
  private final Supplier<Command> whileDriving;
  private final Supplier<Command> atTarget;
  private final double timeoutSec;
  private final double atTargetSeconds;

  private AutoAction(Builder builder) {
    this.name = builder.name;
    this.blueTarget = Optional.ofNullable(builder.blueTarget);
    this.pathName = Optional.ofNullable(builder.pathName);
    this.whileDriving = builder.whileDriving;
    this.atTarget = builder.atTarget;
    this.timeoutSec = builder.timeoutSec;
    this.atTargetSeconds = builder.atTargetSeconds;
  }

  public static Builder named(String name) {
    return new Builder(name);
  }

  public String name() {
    return name;
  }

  /** Where the action drives to, blue alliance. Empty if it drives along a path or stays put. */
  public Optional<Pose2d> blueTarget() {
    return blueTarget;
  }

  /** The pre-drawn path the action follows instead of driving to a pose. */
  public Optional<String> pathName() {
    return pathName;
  }

  /** Time the action spends after driving, for the routine time estimate. */
  public double atTargetSeconds() {
    return atTargetSeconds;
  }

  public double timeoutSec() {
    return timeoutSec;
  }

  /**
   * Where the robot ends up, blue alliance: the target, the end of the path, or empty if it stays
   * where it was.
   */
  public Optional<Pose2d> blueEndPose(DriveAdapter drive) {
    if (pathName.isPresent()) {
      var poses = drive.pathPoses(pathName.get());
      return poses.isEmpty() ? Optional.empty() : Optional.of(poses.get(poses.size() - 1));
    }
    return blueTarget;
  }

  /**
   * The command for this part.
   *
   * @param flip turns a blue-alliance pose into a pose for the current alliance
   */
  public Command toCommand(DriveAdapter drive, java.util.function.UnaryOperator<Pose2d> flip) {
    Command driving =
        pathName.isPresent()
            ? drive.followPath(pathName.get())
            : blueTarget.map(target -> drive.driveTo(flip.apply(target))).orElse(Commands.none());
    Command part = Commands.deadline(driving, whileDriving.get()).andThen(atTarget.get());
    if (timeoutSec > 0.0) {
      part = part.withTimeout(timeoutSec);
    }
    return part.withName("Auto: " + name);
  }

  public static final class Builder {
    private final String name;
    private Pose2d blueTarget = null;
    private String pathName = null;
    private Supplier<Command> whileDriving = Commands::none;
    private Supplier<Command> atTarget = Commands::none;
    private double timeoutSec = 0.0;
    private double atTargetSeconds = 0.0;

    private Builder(String name) {
      this.name = name;
    }

    /** Drive to this pose (blue alliance), avoiding obstacles. */
    public Builder driveTo(Pose2d blueTarget) {
      this.blueTarget = blueTarget;
      this.pathName = null;
      return this;
    }

    /** Follow this pre-drawn path, driving to its start first. */
    public Builder followPath(String pathName) {
      this.pathName = pathName;
      this.blueTarget = null;
      return this;
    }

    /** Runs while driving; must not use the drivetrain. */
    public Builder whileDriving(Supplier<Command> command) {
      this.whileDriving = command;
      return this;
    }

    /**
     * Runs after driving.
     *
     * @param expectedSeconds about how long it takes, for the routine time estimate
     */
    public Builder atTarget(Supplier<Command> command, double expectedSeconds) {
      this.atTarget = command;
      this.atTargetSeconds = expectedSeconds;
      return this;
    }

    /** Ends the whole part after this long, so a stuck part can't use up the rest of auto. */
    public Builder timeout(double seconds) {
      this.timeoutSec = seconds;
      return this;
    }

    public AutoAction build() {
      return new AutoAction(this);
    }
  }
}
