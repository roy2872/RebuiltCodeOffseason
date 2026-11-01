package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.AutoAction;
import frc.lib.auto.AutoRoutineBuilder;
import frc.lib.auto.AutoStartPosition;
import frc.lib.auto.ModularAutoChooser;
import frc.lib.util.AllianceFlipping;
import frc.robot.RobotState;
import frc.robot.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * The REBUILT auto actions, start positions and preset routines, for {@link ModularAutoChooser}.
 *
 * <p>Poses are blue alliance (flipped for red). Blue drives toward +x; the shooter is on the robot's
 * back (-x) and the intake on its front (+x), so heading 180 deg faces the shooter at the hub.
 * TODO: check every pose on the real field.
 */
public final class RebuiltAutos {
  private RebuiltAutos() {}

  // Robot center just behind the starting line, which is the alliance zone edge at the hub line
  private static final double START_X =
      BumpCrossing.BLUE_HUB_LINE_X - 0.6 - DriveConstants.BUMPER_LENGTH_X / 2.0 - 0.05;

  public static final List<AutoStartPosition> START_POSITIONS =
      List.of(
          new AutoStartPosition("Center", new Pose2d(START_X, 4.035, Rotation2d.k180deg)),
          // In line with each bump, for a quick crossing to the neutral zone
          new AutoStartPosition("Left Bump", new Pose2d(START_X, BumpCrossing.BUMP_Y[1], Rotation2d.k180deg)),
          new AutoStartPosition("Right Bump", new Pose2d(START_X, BumpCrossing.BUMP_Y[0], Rotation2d.k180deg)));

  // Scoring spots in the alliance zone, about 2 m from the hub
  private static final Pose2d SCORE_CENTER = new Pose2d(2.6, 4.035, Rotation2d.k180deg);
  private static final Pose2d SCORE_LEFT = new Pose2d(2.8, 5.6, Rotation2d.k180deg);
  private static final Pose2d SCORE_RIGHT = new Pose2d(2.8, 2.5, Rotation2d.k180deg);

  // Depot fuel sits against the alliance wall around y 5.5-6.4. Stop clear of it, intake (+x)
  // toward the wall, then creep in.
  private static final Pose2d DEPOT_APPROACH = new Pose2d(1.4, 5.97, Rotation2d.k180deg);
  // The outpost chute is on the alliance wall near y 0.67; wait in front of it with the intake out
  private static final Pose2d OUTPOST = new Pose2d(1.2, 1.0, Rotation2d.k180deg);

  // Neutral zone fuel: x 7.36-8.18, y 1.72-6.26. Sweeps run along y with the intake leading.
  private static final double SWEEP_X = 7.77;
  private static final double SWEEP_LOW_Y = 1.2;
  private static final double SWEEP_MID_Y = 4.035;
  private static final double SWEEP_HIGH_Y = 6.85;
  private static final double SWEEP_SPEED = 1.5; // m/s, what the intake can take in

  private static final double SHOOT_SECONDS = 3.0;

  /** Registers the actions, start positions and presets. */
  public static ModularAutoChooser createChooser() {
    RebuiltDriveAdapter drive = new RebuiltDriveAdapter();
    registerNamedCommands();
    return new ModularAutoChooser(
        5,
        START_POSITIONS,
        actions(drive),
        presets(drive),
        drive,
        AllianceFlipping::apply,
        RebuiltAutos::ending);
  }

  private static List<AutoAction> actions(RebuiltDriveAdapter drive) {
    return List.of(
        AutoAction.named("Shoot Here").atTarget(() -> shoot(true), SHOOT_SECONDS).build(),
        shootFrom("Shoot From Center", SCORE_CENTER),
        shootFrom("Shoot From Left", SCORE_LEFT),
        shootFrom("Shoot From Right", SCORE_RIGHT),
        AutoAction.named("Intake Depot")
            .driveTo(DEPOT_APPROACH)
            .whileDriving(SuperStructure.mInstance::intakeCommand)
            .atTarget(() -> creepWhileIntaking(0.5, 1.5), 1.5)
            .timeout(8.0)
            .build(),
        AutoAction.named("Intake Outpost")
            .driveTo(OUTPOST)
            .whileDriving(SuperStructure.mInstance::intakeCommand)
            .atTarget(() -> SuperStructure.mInstance.intakeCommand().withTimeout(3.0), 3.0)
            .timeout(9.0)
            .build(),
        sweep(drive, "Sweep Neutral Full", SWEEP_LOW_Y, SWEEP_HIGH_Y),
        sweep(drive, "Sweep Neutral Right Half", SWEEP_LOW_Y, SWEEP_MID_Y),
        sweep(drive, "Sweep Neutral Left Half", SWEEP_HIGH_Y, SWEEP_MID_Y),
        AutoAction.named("Ferry").atTarget(() -> shoot(false), SHOOT_SECONDS).build(),
        AutoAction.named("Wait 1 s").atTarget(() -> Commands.waitSeconds(1.0), 1.0).build(),
        AutoAction.named("Wait 3 s").atTarget(() -> Commands.waitSeconds(3.0), 3.0).build());
  }

  private static Map<String, Supplier<Command>> presets(RebuiltDriveAdapter drive) {
    Map<String, Supplier<Command>> presets = new LinkedHashMap<>();
    presets.put("Do Nothing", Commands::none);
    presets.put(
        "Shoot Preload (Center)",
        () ->
            AutoRoutineBuilder.build(
                    START_POSITIONS.get(0),
                    List.of(AutoAction.named("Shoot Here").atTarget(() -> shoot(true), SHOOT_SECONDS).build()),
                    drive,
                    AllianceFlipping::apply,
                    RebuiltAutos::ending)
                .command());
    return presets;
  }

  /** Drives to a scoring spot spinning up on the way, then shoots. */
  private static AutoAction shootFrom(String name, Pose2d spot) {
    return AutoAction.named(name)
        .driveTo(spot)
        .whileDriving(
            () ->
                Commands.runOnce(() -> RobotState.mInstance.setShootType(true))
                    .andThen(SuperStructure.mInstance.spinUpCommand()))
        .atTarget(() -> shoot(true), SHOOT_SECONDS)
        .timeout(10.0)
        .build();
  }

  /** Crosses into the neutral zone if needed, then drives through a column of fuel intaking. */
  private static AutoAction sweep(RebuiltDriveAdapter drive, String name, double fromY, double toY) {
    // Heading so the intake (+x) leads the motion
    Rotation2d heading = toY > fromY ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg;
    Pose2d start = new Pose2d(SWEEP_X, fromY, heading);
    Pose2d end = new Pose2d(SWEEP_X, toY, heading);
    double sweepSeconds = Math.abs(toY - fromY) / SWEEP_SPEED + 0.5;
    return AutoAction.named(name)
        .driveTo(start)
        .atTarget(
            () ->
                Commands.deadline(
                    drive.driveStraight(
                        AllianceFlipping.apply(start),
                        AllianceFlipping.apply(end),
                        AllianceFlipping.apply(heading),
                        SWEEP_SPEED),
                    SuperStructure.mInstance.intakeCommand()),
            sweepSeconds)
        .timeout(12.0)
        .build();
  }

  /** Shoots for a fixed time (there is no fuel sensor yet), then stops the mechanisms. */
  private static Command shoot(boolean atHub) {
    return Commands.runOnce(() -> RobotState.mInstance.setShootType(atHub))
        .andThen(SuperStructure.mInstance.shoot(true).withTimeout(SHOOT_SECONDS))
        .andThen(SuperStructure.mInstance.stopCommand())
        .withName(atHub ? "Auto Shoot" : "Auto Ferry");
  }

  /** Drives slowly forward (intake first) while intaking, e.g. into the depot. */
  private static Command creepWhileIntaking(double speed, double seconds) {
    return Commands.deadline(
        Drive.mInstance
            .run(() -> Drive.mInstance.runVelocity(new ChassisSpeeds(speed, 0.0, 0.0)))
            .withTimeout(seconds)
            .finallyDo(Drive.mInstance::stop),
        SuperStructure.mInstance.intakeCommand());
  }

  private static Command ending() {
    return Commands.parallel(
            Drive.mInstance.runOnce(Drive.mInstance::stop), SuperStructure.mInstance.stopCommand())
        .withName("Auto: stop");
  }

  /** The same building blocks for autos drawn in the PathPlanner GUI. */
  private static void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", shoot(true));
    NamedCommands.registerCommand("Ferry", shoot(false));
    NamedCommands.registerCommand("Intake", SuperStructure.mInstance.intakeCommand());
    NamedCommands.registerCommand(
        "SpinUp",
        Commands.runOnce(() -> RobotState.mInstance.setShootType(true))
            .andThen(SuperStructure.mInstance.spinUpCommand()));
    NamedCommands.registerCommand("StopAll", ending());
  }
}
