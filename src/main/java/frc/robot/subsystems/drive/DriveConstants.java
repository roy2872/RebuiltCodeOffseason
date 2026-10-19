package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {

  // Single source of truth for the drivetrain's physical constants (setpoint generator,
  // PathPlanner, maple-sim). Keep deploy/pathplanner/settings.json in sync.
  public static final double robotMassKg = 52.0; // TODO: weigh with battery and bumpers
  // m(L^2+W^2)/12 for a ~0.89 m square robot. TODO: measure (e.g. from a SysId spin test)
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2; // TODO: measure (typical tread on carpet 1.1-1.3)

  public static final double MAX_ACCELERATION = 10.0; // m/s^2
  public static final double MAX_FRONT_ACCEL = 8.0; // m/s^2
  public static final double MAX_SIDE_ACCEL = 8.0; // m/s^2
  public static final double MAX_SKID_ACCEL = 7.0;
  public static final double MAX_ROTATIONAL_ACCEL = 12.0;

  public static final double MAX_SHOOT_ON_THE_MOVE_SKID_ACCEL = 5.0;
  public static final double MAX_SHOOT_ON_THE_MOVE_ROTATION_ACCEL = 7.0; // [rad/s^2]

  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = 0.71;
  public static final double wheelBase = 0.67;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  // Same as TunerConstants (kWheelRadius). TODO: refine with wheelRadiusCharacterization
  public static final Distance WHEEL_RADIUS = Units.Inches.of(2.0);

  // Robot footprint, bumper to bumper. TODO: measure. Keep tools/generate_navgrid.py and
  // deploy/pathplanner/settings.json in sync.
  public static final double BUMPER_LENGTH_X = 0.89; // m
  public static final double BUMPER_WIDTH_Y = 0.89; // m
  // How far the fully deployed intake reaches past the front (+x) bumper
  public static final double INTAKE_MAX_EXTENSION = 0.26; // m
  /** Radius of the circle the robot sweeps when it turns in place, intake fully deployed. */
  public static final double SWEPT_RADIUS =
      Math.hypot(BUMPER_LENGTH_X / 2.0 + INTAKE_MAX_EXTENSION, BUMPER_WIDTH_Y / 2.0);

  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5.2);
  public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(MAX_SPEED.magnitude() / Math.hypot(moduleTranslations[0].getX(), moduleTranslations[0].getY()));

  public static final DCMotor driveGearbox = DCMotor.getFalcon500(1);
  public static final DCMotor turnGearbox = DCMotor.getFalcon500(1);
  public static final double driveMotorReduction = 5.9; // Gear ratio for drive motor
  public static final double turnMotorReduction = 18.75; // Gear ratio for turn motor

  // PathPlanner path following feedback (on top of the path's feedforward speeds)
  public static final double PATH_TRANSLATION_KP = 5.0;
  public static final double PATH_ROTATION_KP = 5.0;

  public static final double DEADBAND = 0.1; // Deadband for joystick inputs
  public static final double ASSISTED_DRIVE_PERCENTAGE = 0.25;
  public static final double SHOOT_ON_THE_MOVE_DRIVE_PERCENTAGE = 0.3;

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Mass.ofRelativeUnits(robotMassKg, Kilogram))
          .withBumperSize(Meters.of(BUMPER_LENGTH_X), Meters.of(BUMPER_WIDTH_Y))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  WHEEL_RADIUS,
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));
}
