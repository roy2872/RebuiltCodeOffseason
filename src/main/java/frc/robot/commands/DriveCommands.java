package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipping;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class DriveCommands {
  public static final double deadband = 0.1;
  private static final double ffStartDelay = 2.0; // Secs
  private static final double ffRampRate = 0.1; // Volts/Sec
  private static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  private static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2

  private static final TunableNumber driveTowerAngleKp =
      new TunableNumber("DriveCommands/Tower/kP", 6.0);
  private static final TunableNumber driveTowerAngleKd =
      new TunableNumber("DriveCommands/Tower/kD", 0.05);
  private static final TunableNumber driveLaunchKp =
      new TunableNumber("DriveCommands/Launching/kP", 8.0);
  private static final TunableNumber driveLaunchKd =
      new TunableNumber("DriveCommands/Launching/kD", 0.5);
//   private static final TunableNumber driveYawLaunchToleranceDeg =
//       new TunableNumber("DriveCommands/Launching/YawToleranceDeg", 10.0);
//   private static final TunableNumber drivePitchLaunchToleranceDeg =
//       new TunableNumber("DriveCommands/Launching/PitchToleranceDeg", 5.0);
//   private static final TunableNumber driveRollLaunchToleranceDeg =
//       new TunableNumber("DriveCommands/Launching/RollToleranceDeg", 5.0);
//   private static final TunableNumber driveYawPassToleranceDeg =
//       new TunableNumber("DriveCommands/Passing/YawToleranceDeg", 15.0);
//   private static final TunableNumber drivePitchPassToleranceDeg =
//       new TunableNumber("DriveCommands/Passing/PitchToleranceDeg", 5.0);
//   private static final TunableNumber driveRollPassToleranceDeg =
//       new TunableNumber("DriveCommands/Passing/RollToleranceDeg", 5.0);

//   private static final TunableNumber lockMetersPerSecondThreshold =
//       new TunableNumber("DriveCommands/Launching/LockMetersPerSecThreshold", 0.1);
//   private static final TunableNumber lockOmegaRadsPerSecThreshold =
//       new TunableNumber("DriveCommands/Launching/LockOmegaRadsPerSecThreshold", 0.15);

//   private static final TunableNumber driveLaunchMaxPolarVelocityRadPerSec =
//       new TunableNumber("DriveCommands/Launching/MaxPolarVelocityRadPerSec", 0.4);
//   private static final TunableNumber driveLauncherCORMinErrorDeg =
//       new TunableNumber("DriveCommands/Launching/DriveLauncherCORMinErrorDeg", 15.0);
//   private static final TunableNumber driveLauncherCORMaxErrorDeg =
//       new TunableNumber("DriveCommands/Launching/DriveLauncherCORMaxErrorDeg", 30.0);

  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, deadband);
    return omega * omega * Math.signum(omega);
  }

  public static ChassisSpeeds getSpeedsFromJoysticks(
      double driverX, double driverY, double driverOmega) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(driverX, driverY).times(DriveConstants.MAX_SPEED.magnitude());

    // Calculate angular velocity
    double omega = getOmegaFromJoysticks(driverOmega);

    return new ChassisSpeeds(
        linearVelocity.getX(), linearVelocity.getY(), omega * DriveConstants.MAX_ANGULAR_VELOCITY.magnitude());
  }

  /**
   * Field or robot relative drive command using two joysticks (controlling linear and angular
   * velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              getSpeedsFromJoysticks(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble());
          drive.runVelocity(
              robotRelative.getAsBoolean()
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? RobotState.mInstance.getRotation().plus(Rotation2d.kPi)
                          : RobotState.mInstance.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveUnderTower(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          Rotation2d nearestLockedRotation =
              RobotState.mInstance.getRotation().getRadians() > 0
                  ? Rotation2d.kCCW_90deg
                  : Rotation2d.kCW_90deg;
          double omegaOutput =
              nearestLockedRotation.minus(RobotState.mInstance.getRotation()).getRadians()
                      * driveTowerAngleKp.getAsDouble()
                  - RobotState.mInstance.getFieldVelocity().omegaRadiansPerSecond
                      * driveTowerAngleKd.getAsDouble();

          // Calculate speeds
          Translation2d fieldRelativeLinearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(DriveConstants.MAX_SPEED.magnitude());
          if (AllianceFlipping.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
          }
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeLinearVelocity.getX(),
                  fieldRelativeLinearVelocity.getY(),
                  omegaOutput,
                  RobotState.mInstance.getRotation());
          drive.runVelocity(speeds);
          RobotState.mInstance
              .setRobotSetpointVelocity(
                  ChassisSpeeds.discretize(
                      new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0),
                      Constants.CYCLE_TIME));
        },
        drive);
  }

public static Command autoAlignAngle(Drive drive, Supplier<Rotation2d> targetAngle) {
  return Commands.run(
      () -> {
        Rotation2d currentRotation = RobotState.mInstance.getRotation();
        Rotation2d angleErrorRad = targetAngle.get().minus(currentRotation);
        double omegaOutput =
            angleErrorRad.getRadians() * driveLaunchKp.getAsDouble()
                - RobotState.mInstance.getFieldVelocity().omegaRadiansPerSecond * driveLaunchKd.getAsDouble();
        ChassisSpeeds targetSpeeds =
            new ChassisSpeeds(
                0,
                0,
                omegaOutput);

        drive.runVelocity(targetSpeeds);
      },
      drive);
}



  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(ffStartDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * ffRampRate;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(wheelRadiusMaxVelocity);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = RobotState.mInstance.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = RobotState.mInstance.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.00000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}