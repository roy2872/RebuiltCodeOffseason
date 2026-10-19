package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.sim.FuelSim;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Connects the robot's mechanisms to {@link FuelSim}: fuel is picked up by the intake on the +x side,
 * carried in the hopper, and shot out of the -x side with the flywheel speed and hood angle the
 * mechanisms actually reached, so missed setpoints show up as missed shots.
 */
public class RobotFuelSim {
  // TODO: measure on the robot
  private static final int CAPACITY = 40;
  // Where held fuel is drawn, only for the 3D view. It runs from this far in from the shooter-side edge
  // of the robot to the intake-side edge, plus however far the intake is open, across the inside of the
  // robot. Fuel that doesn't fit (a full load with the intake stowed) isn't drawn, though
  // FieldSimulation/FuelHeld still counts it.
  private static final double HOPPER_BACK_FROM_ROBOT_EDGE = 0.30; // m
  private static final double BUMPER_THICKNESS = 0.08; // m, TODO: measure
  private static final double HOPPER_FLOOR = 0.20; // m
  private static final double HOPPER_CEILING = 0.70; // m
  private static final int PRELOAD = 8; // The most a robot may start the match with
  private static final double BUMPER_HEIGHT = 0.2; // m, fuel above this passes over the robot
  private static final double INTAKE_WIDTH = 0.66; // m, centered on the robot
  // Fuel shot per second with the feeder at FEED_VOLTAGE. Scales with feeder voltage.
  private static final double FEED_BALLS_PER_SECOND = 10.0;
  private static final double PURGE_BALLS_PER_SECOND = 6.0;
  private static final double PURGE_SPEED = 3.0; // m/s
  // Below this main wheel surface speed the flywheel can't throw fuel, so the feeder just jams
  private static final double MIN_LAUNCH_SURFACE_SPEED = 2.0; // m/s

  // Lets the sim be turned off from the dashboard, since it costs a lot of loop time and logging
  private final LoggedNetworkBoolean enabled =
      new LoggedNetworkBoolean("/SmartDashboard/Fuel Sim Enabled", true);
  private boolean wasEnabled = true;
  private final FuelSim fuelSim = new FuelSim();
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final double bumperLength =
      DriveConstants.mapleSimConfig.bumperLengthX.in(Meters);
  private final double bumperWidth =
      DriveConstants.mapleSimConfig.bumperWidthY.in(Meters);

  private int fuelHeld = 0;
  // Fraction of the next fuel already pushed into the shooter or out of the intake
  private double launchProgress = 0.0;
  private double purgeProgress = 0.0;

  /**
   * @param poseSupplier the true (simulated) robot pose
   * @param fieldSpeedsSupplier the true field-relative chassis speeds
   */
  public RobotFuelSim(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;

    fuelSim.registerRobot(bumperWidth, bumperLength, BUMPER_HEIGHT, poseSupplier, fieldSpeedsSupplier);
    fuelSim.registerIntake(
        bumperLength / 2,
        bumperLength / 2 + IntakeDeployConstants.VISUALIZATION_TRAVEL.getX(),
        -INTAKE_WIDTH / 2,
        INTAKE_WIDTH / 2,
        this::canIntake,
        () -> fuelHeld++);
    fuelSim.enableAirResistance();
    fuelSim.setLoggingFrequency(50.0); // Every loop, so shots look smooth
    fuelSim.start();
    reset();

    SmartDashboard.putData(
        "Reset Fuel", Commands.runOnce(this::reset).ignoringDisable(true).withName("Reset Fuel"));
  }

  /** Puts the starting fuel back on the field, preloads the robot, and zeroes the hub scores. */
  public void reset() {
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
    fuelHeld = PRELOAD;
    launchProgress = 0.0;
    purgeProgress = 0.0;
  }

  /** Fuel in the robot, ready to shoot. */
  public int getFuelHeld() {
    return fuelHeld;
  }

  /** Sets the fuel in the robot (e.g. empty for a self test). */
  public void setFuelHeld(int count) {
    fuelHeld = Math.max(0, Math.min(CAPACITY, count));
  }

  /** Puts one fuel on the floor just in front of the intake, as if someone set it there. */
  public void placeFuelInFrontOfIntake() {
    Pose2d robot = poseSupplier.get();
    Translation2d spot =
        robot.getTranslation()
            .plus(new Translation2d(bumperLength / 2 + 0.12, 0.0).rotateBy(robot.getRotation()));
    fuelSim.spawnFuel(
        new Translation3d(spot.getX(), spot.getY(), FieldConstants.fuelDiameter / 2), new Translation3d());
  }

  /** Call every loop, after the drivetrain sim has stepped. */
  public void periodic() {
    if (!enabled.get()) {
      if (wasEnabled) {
        fuelSim.logNoFuel();
        Logger.recordOutput("FieldSimulation/Fuel/Held", new Translation3d[0]);
      }
      wasEnabled = false;
      return;
    }
    wasEnabled = true;

    updateShooter();
    updatePurge();
    fuelSim.updateSim();

    Logger.recordOutput("FieldSimulation/FuelHeld", fuelHeld);
    Logger.recordOutput("FieldSimulation/Fuel/Held", heldFuelPositions());
    Logger.recordOutput("FieldSimulation/HopperFull", fuelHeld >= CAPACITY);
    Logger.recordOutput("FieldSimulation/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("FieldSimulation/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
  }

  /** Collects whenever the rollers run forward, even while the intake is still deploying. */
  private boolean canIntake() {
    return fuelHeld < CAPACITY && voltageSetpoint(IntakeRollers.mInstance.getSetpoint()) > 0;
  }

  private void updateShooter() {
    double feedFraction =
        voltageSetpoint(Feeder.mInstance.getSetpoint()) / FeederConstants.FEED_VOLTAGE.in(Volts);
    double mainWheelSpeed = Shooter.mInstance.getVelocity().in(RotationsPerSecond)
        * Math.PI * ShooterConstants.FLYWHEEL_DIAMETER.in(Meters);
    if (feedFraction <= 0 || mainWheelSpeed < MIN_LAUNCH_SURFACE_SPEED || fuelHeld == 0) {
      launchProgress = 0.0;
      return;
    }

    launchProgress += FEED_BALLS_PER_SECOND * feedFraction * TimedRobot.kDefaultPeriod;
    // The hood angle is measured from vertical, so the ball leaves at 90 degrees minus it
    double launchAngle = Math.toRadians(90.0 - Hood.mInstance.getPosition().in(Degrees));
    while (launchProgress >= 1.0 && fuelHeld > 0) {
      launchProgress -= 1.0;
      fuelHeld--;
      Launch launch =
          launch(poseSupplier.get(), fieldSpeedsSupplier.get(), mainWheelSpeed, launchAngle);
      fuelSim.spawnFuel(launch.position(), launch.velocity(), launch.spin());
    }
  }

  /** Running the intake rollers backwards pushes fuel out the front. */
  private void updatePurge() {
    if (voltageSetpoint(IntakeRollers.mInstance.getSetpoint()) >= 0 || fuelHeld == 0) {
      purgeProgress = 0.0;
      return;
    }

    purgeProgress += PURGE_BALLS_PER_SECOND * TimedRobot.kDefaultPeriod;
    while (purgeProgress >= 1.0 && fuelHeld > 0) {
      purgeProgress -= 1.0;
      fuelHeld--;
      Pose2d robot = poseSupplier.get();
      ChassisSpeeds speeds = fieldSpeedsSupplier.get();
      // Just past the bumper, so the robot doesn't push it straight back
      Translation2d exit = robot.getTranslation()
          .plus(new Translation2d(bumperLength / 2 + 0.1, 0.0).rotateBy(robot.getRotation()));
      Translation2d velocity = new Translation2d(PURGE_SPEED, robot.getRotation())
          .plus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
      fuelSim.spawnFuel(
          new Translation3d(exit.getX(), exit.getY(), BUMPER_HEIGHT / 2),
          new Translation3d(velocity.getX(), velocity.getY(), 0.0));
    }
  }

  /** Voltage of a voltage setpoint, or 0 for any other kind. */
  @SuppressWarnings("rawtypes")
  private static double voltageSetpoint(Setpoint setpoint) {
    return setpoint.mode == Mode.VOLTAGE ? setpoint.baseUnits : 0.0;
  }

  /**
   * Slots for held fuel in the robot frame, filled bottom layer first. Each layer is a grid of touching
   * balls, and every other layer is shifted half a ball so it sits in the gaps of the one below.
   *
   * @param backX back edge of the hopper (robot frame)
   * @param frontX front edge of the hopper, which moves out with the intake
   * @param halfWidth half the hopper's width, centered on the robot
   * @return up to count slots, fewer if they don't all fit
   */
  private static Translation3d[] hopperSlots(double backX, double frontX, double halfWidth, int count) {
    double radius = FuelSim.FUEL_RADIUS;
    double diameter = 2 * radius;
    // Rounded so float error can't drop a ball that exactly fits
    int columns = (int) Math.floor((frontX - backX) / diameter + 1e-6);
    int rows = (int) Math.floor(2 * halfWidth / diameter + 1e-6);
    Translation3d[] slots = new Translation3d[count];
    int i = 0;
    for (int layer = 0; i < count; layer++) {
      // Nested balls sit lower than a full diameter above the layer below
      double z = HOPPER_FLOOR + radius + layer * diameter * 0.85;
      if (z + radius > HOPPER_CEILING + 1e-6) break;
      boolean shifted = layer % 2 == 1;
      double offset = shifted ? radius : 0.0;
      // A shifted layer has room for one fewer ball each way
      for (int row = 0; row < rows - (shifted ? 1 : 0) && i < count; row++) {
        for (int column = 0; column < columns - (shifted ? 1 : 0) && i < count; column++) {
          slots[i++] = new Translation3d(
              backX + radius + offset + column * diameter,
              -halfWidth + radius + offset + row * diameter,
              z);
        }
      }
    }
    return Arrays.copyOf(slots, i);
  }

  /**
   * Field positions of the fuel in the robot, for the 3D view. Fuel spreads forward into the intake as
   * it opens, so a stowed intake stacks it higher.
   */
  private Translation3d[] heldFuelPositions() {
    double intakeExtension = IntakeDeploy.mInstance.getPosition()
        .minus(IntakeDeployConstants.INTAKE_STOWED_ANGLE)
        .in(Rotations) * IntakeDeployConstants.VISUALIZATION_EXTENSION_PER_ROTATION.in(Meters);
    Translation3d[] slots = hopperSlots(
        -bumperLength / 2 + HOPPER_BACK_FROM_ROBOT_EDGE,
        bumperLength / 2 + Math.max(intakeExtension, 0.0),
        bumperWidth / 2 - BUMPER_THICKNESS,
        fuelHeld);

    Pose2d robot = poseSupplier.get();
    Translation3d[] positions = new Translation3d[slots.length];
    for (int i = 0; i < slots.length; i++) {
      Translation2d onField =
          robot.getTranslation().plus(slots[i].toTranslation2d().rotateBy(robot.getRotation()));
      positions[i] = new Translation3d(onField.getX(), onField.getY(), slots[i].getZ());
    }
    return positions;
  }

  /** Where a shot fuel starts, how fast it goes, and how it spins, all field-relative. */
  public record Launch(Translation3d position, Translation3d velocity, Translation3d spin) {}

  /**
   * Same model as the shooting table generator: the ball leaves at the average surface speed of the
   * main and hood wheels and spins backwards from their speed difference. Slip scales both by
   * ShooterConstants.BALL_SPEED_FRACTION.
   *
   * @param robot robot pose on the field
   * @param fieldSpeeds field-relative chassis speeds, added to the ball's velocity
   * @param mainWheelSpeed main flywheel surface speed, m/s
   * @param launchAngle ball launch angle above horizontal, radians
   */
  public static Launch launch(
      Pose2d robot, ChassisSpeeds fieldSpeeds, double mainWheelSpeed, double launchAngle) {
    double ballSpeedFraction = ShooterConstants.BALL_SPEED_FRACTION.getAsDouble();
    double hoodWheelSpeed = ShooterConstants.HOOD_WHEEL_SPEED_RATIO * mainWheelSpeed;
    double exitSpeed = ShooterConstants.ballSpeedForMainWheelSpeed(mainWheelSpeed);
    double spinRate =
        (mainWheelSpeed - hoodWheelSpeed) * ballSpeedFraction / (2.0 * FuelSim.AERO_RADIUS);

    Translation3d onRobot = ShooterConstants.LAUNCH_POSITION_ON_ROBOT;
    Translation2d offset = onRobot.toTranslation2d().rotateBy(robot.getRotation());
    Translation2d position = robot.getTranslation().plus(offset);
    // The shooter moves with the chassis, including the part of its speed from turning
    Translation2d shooterVelocity = new Translation2d(
        fieldSpeeds.vxMetersPerSecond - fieldSpeeds.omegaRadiansPerSecond * offset.getY(),
        fieldSpeeds.vyMetersPerSecond + fieldSpeeds.omegaRadiansPerSecond * offset.getX());

    Rotation2d direction = robot.getRotation().plus(Rotation2d.k180deg); // Shooter faces -x
    double horizontalSpeed = exitSpeed * Math.cos(launchAngle);
    Translation3d velocity = new Translation3d(
        horizontalSpeed * direction.getCos() + shooterVelocity.getX(),
        horizontalSpeed * direction.getSin() + shooterVelocity.getY(),
        exitSpeed * Math.sin(launchAngle));
    // Backspin: the axis is (direction x up), so the top of the ball moves back toward the robot
    Translation3d spin =
        new Translation3d(direction.getSin(), -direction.getCos(), 0.0).times(spinRate);

    return new Launch(new Translation3d(position.getX(), position.getY(), onRobot.getZ()), velocity, spin);
  }
}
