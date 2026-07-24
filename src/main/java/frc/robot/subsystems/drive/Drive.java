package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public SwerveDriveSimulation driveSimulation = null;
  public static final Drive mInstance = new Drive();
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using backup gyro as fallback.", AlertType.kError);
  private final Alert backupGyroDisconnectedAlert =
      new Alert("Disconnected backup gyro, using primary.", AlertType.kInfo);
  private final Alert gyroAndBackupGyroDisconnectedAlert =
      new Alert(
          "Disconnected gyro and backup gyro, using kinematics as fallback.", AlertType.kError);

  // private static final TunableNumber coastWaitTime =
  //     new TunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  // private static final TunableNumber coastMetersPerSecondThreshold =
  //     new TunableNumber("Drive/CoastMetersPerSecThreshold", .05);

  private final Timer lastMovementTimer = new Timer();
  private boolean lastEnabled = false;

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  @AutoLogOutput(key = "Drive/CoastRequest")
  private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

  private Drive() {
    switch(Constants.currentMode) {
      case REAL -> {
          buildDriveSubsystem(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
      }
      case SIM ->{
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
          buildDriveSubsystem(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
      }
      default -> {
          buildDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }


    }
  }

  public void buildDriveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
      modules[0] = new Module(flModuleIO, 0);
      modules[1] = new Module(frModuleIO, 1);
      modules[2] = new Module(blModuleIO, 2);
      modules[3] = new Module(brModuleIO, 3);
      lastMovementTimer.start();
      for (var module : modules) {
        module.stop();
      }
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

    // Update odometry
    boolean driveConnected = true;
    for (var module : modules) {
      driveConnected &= module.isConnected();
    }
    if (driveConnected) {
      // RobotState.getInstance()
      //     .addOdometryObservation(
      //         new OdometryObservation(
      //             Timer.getTimestamp(),
      //             getModulePositions(),
      //             Optional.ofNullable(gyroInputs.connected ? gyroInputs.rollPosition : null),
      //             Optional.ofNullable(gyroInputs.connected ? gyroInputs.pitchPosition : null),
      //             Optional.ofNullable(gyroInputs.connected ? gyroInputs.yawPosition : null),
      //             Optional.ofNullable(
      //                 backupGyroInputs.connected ? backupGyroInputs.rollPosition : null),
      //             Optional.ofNullable(
      //                 backupGyroInputs.connected ? backupGyroInputs.pitchPosition : null),
      //             Optional.ofNullable(
      //                 backupGyroInputs.connected ? backupGyroInputs.yawPosition : null)));
    }
    RobotState.mInstance.setRobotVelocity(getChassisSpeeds());

    // Update gyro alerts
    gyroDisconnectedAlert.set(
        !gyroInputs.connected);


    if (DriverStation.isEnabled() && !lastEnabled) {
      coastRequest = CoastRequest.AUTOMATIC;
    }
  }

  // @Override
  // public void periodicAfterScheduler() {
  //   for (var module : modules) {
  //     module.periodicAfterScheduler();
  //   }
  // }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.CYCLE_TIME);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
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
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Stops the drive and turns the modules to an O arrangement to resist movement. */
  public void stopWithO() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle().plus(Rotation2d.kCW_90deg);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
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

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }
}