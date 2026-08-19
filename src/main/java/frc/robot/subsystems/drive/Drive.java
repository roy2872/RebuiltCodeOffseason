package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.accelLimitsLib;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

public class Drive extends SubsystemBase {

  public SwerveDriveSimulation driveSimulation = null;
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using backup gyro as fallback.",
      AlertType.kError);
  private final SwerveSetpointGenerator setpointGenerator;
    private static final double ROBOT_MASS_KG = 55;
  private static final double ROBOT_MOI = 1.2;
  private static final double WHEEL_COF = 1.5;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getFalcon500(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          DriveConstants.moduleTranslations);
  private SwerveSetpoint previousSetpoint;
    // private static final TunableNumber coastWaitTime =
  // new TunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  // private static final TunableNumber coastMetersPerSecondThreshold =
  // new TunableNumber("Drive/CoastMetersPerSecThreshold", .05);

  private final Timer lastMovementTimer = new Timer();
  private boolean lastEnabled = false;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  @AutoLogOutput(key = "Drive/CoastRequest")
  private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

    private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public static final Drive mInstance = new Drive();
  private Drive() {
    switch (Constants.currentMode) {
      case REAL -> {
        this.gyroIO = new GyroIONavX();
        modules[0] = new Module(new ModuleIOTalonFX(TunerConstants.FrontLeft), 0);
        modules[1] = new Module(new ModuleIOTalonFX(TunerConstants.FrontRight), 1);
        modules[2] = new Module(new ModuleIOTalonFX(TunerConstants.BackLeft), 2);
        modules[3] = new Module(new ModuleIOTalonFX(TunerConstants.BackRight), 3);
      }
      case SIM -> {
        driveSimulation = new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        this.gyroIO = new GyroIOSim(driveSimulation.getGyroSimulation());
        modules[0] = new Module(new ModuleIOSim(driveSimulation.getModules()[0]), 0);
        modules[1] = new Module(new ModuleIOSim(driveSimulation.getModules()[1]), 1);
        modules[2] = new Module(new ModuleIOSim(driveSimulation.getModules()[2]), 2);
        modules[3] = new Module(new ModuleIOSim(driveSimulation.getModules()[3]), 3);
      }
      default -> {
        this.gyroIO = new GyroIO() {
        };
        modules[0] = new Module(new ModuleIO() {
        }, 0);
        modules[1] = new Module(new ModuleIO() {
        }, 1);
        modules[2] = new Module(new ModuleIO() {
        }, 2);
        modules[3] = new Module(new ModuleIO() {
        }, 3);
      }
    }

    lastMovementTimer.start();
    for (var module : modules) {
      module.stop();
    }
    setpointGenerator =
        new SwerveSetpointGenerator(
            PP_CONFIG, // The robot configuration. This is the same config used for generating
            // trajectories and running path following commands.
            Units.rotationsToRadians(
                5.0) 
            );
                previousSetpoint =
    new SwerveSetpoint(
      getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(PP_CONFIG.numModules));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // // Update odometry
    // boolean driveConnected = true;
    // for (var module : modules) {
    //   driveConnected &= module.isConnected();
    // }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) { // can add here skid detection
      modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    //   // Apply update
    //   poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    // }

    RobotState.mInstance
        .addOdometryObservation(
            new OdometryObservation(
                modulePositions, Optional.of(rawGyroRotation), RobotController.getFPGATime() * 1e-6));

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    RobotState.mInstance.setRobotVelocity(getChassisSpeeds());

    if (DriverStation.isEnabled() && !lastEnabled) {
      coastRequest = CoastRequest.AUTOMATIC;
    }
  }

  // @Override
  // public void periodicAfterScheduler() {
  // for (var module : modules) {
  // module.periodicAfterScheduler();
  // }
  // }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.CYCLE_TIME.baseUnitMagnitude());
    // SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );
    SwerveModuleState[] setpointStates = previousSetpoint.moduleStates(); 
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_SPEED);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Publish setpoints
    RobotState.mInstance.setRobotSetpointVelocity(discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
    RobotState.mInstance.setRobotSetpointVelocity(new ChassisSpeeds());
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Stops the drive and turns the modules to an O arrangement to resist movement.
   */
  public void stopWithO() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle().plus(Rotation2d.kCW_90deg);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }
}