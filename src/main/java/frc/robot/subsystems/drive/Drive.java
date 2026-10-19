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
import frc.robot.auto.BumpCrossing;
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
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.lib.util.AllianceFlipping;
import frc.lib.util.LocalADStarAK;
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
  // Physical constants come from DriveConstants so the setpoint generator, PathPlanner and
  // maple-sim all agree. Keep deploy/pathplanner/settings.json in sync when changing them.
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          DriveConstants.robotMassKg,
          DriveConstants.robotMOI,
          new ModuleConfig(
              DriveConstants.WHEEL_RADIUS.in(edu.wpi.first.units.Units.Meters),
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              DriveConstants.wheelCOF,
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
        // Only the hubs are solid: the robot drives over the bumps (maple-sim can't model the slope)
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        driveSimulation = new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        this.gyroIO =
            new GyroIOSim(
                driveSimulation.getGyroSimulation(),
                () -> BumpCrossing.simulatedTiltDegrees(driveSimulation.getSimulatedDriveTrainPose()));
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
    // Starts only if real hardware registered signals with it
    PhoenixOdometryThread.getInstance().start();
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
    configurePathPlanner();
  }

  /** PathPlanner path following and on-the-fly pathfinding (navgrid: tools/generate_navgrid.py). */
  private void configurePathPlanner() {
    AutoBuilder.configure(
        RobotState.mInstance::getEstimatedPose,
        RobotState.mInstance::resetPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(DriveConstants.PATH_TRANSLATION_KP, 0.0, 0.0),
            new PIDConstants(DriveConstants.PATH_ROTATION_KP, 0.0, 0.0)),
        PP_CONFIG,
        AllianceFlipping::shouldFlip,
        this);
    // AdvantageKit-compatible AD*: logged inputs, so replay plans the same paths
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("Auto/ActivePath", poses.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Auto/TargetPose", pose));
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

    // Before odometry: on the bump, wheel distance is measured along the slope
    RobotState.mInstance.updateTilt(getTiltDegrees(), RobotController.getFPGATime() * 1e-6);

    // Update odometry with every high-rate sample received since the last cycle, each at its own
    // timestamp. Every module and the gyro are sampled together; use only samples all of them have.
    int sampleCount = Integer.MAX_VALUE;
    for (var module : modules) {
      sampleCount = Math.min(sampleCount, module.getOdometryTimestamps().length);
    }
    boolean useGyroSamples = gyroInputs.connected && gyroInputs.odometryYawPositions.length > 0;
    if (useGyroSamples) {
      sampleCount = Math.min(sampleCount, gyroInputs.odometryYawPositions.length);
    }
    Logger.recordOutput("Drive/OdometrySampleCount", sampleCount);

    if (sampleCount == 0) {
      // No queued samples (thread not running, or a log recorded before high-rate odometry):
      // fall back to one sample from this cycle's inputs
      SwerveModulePosition[] modulePositions = getModulePositions();
      addOdometrySample(
          modulePositions,
          gyroInputs.connected ? Optional.of(gyroInputs.yawPosition) : Optional.empty(),
          RobotController.getFPGATime() * 1e-6);
    } else {
      double[] sampleTimestamps = modules[0].getOdometryTimestamps();
      for (int i = 0; i < sampleCount; i++) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        }
        Optional<Rotation2d> gyroYaw =
            useGyroSamples
                ? Optional.of(gyroInputs.odometryYawPositions[i])
                : gyroInputs.connected ? Optional.of(gyroInputs.yawPosition) : Optional.empty();
        addOdometrySample(modulePositions, gyroYaw, sampleTimestamps[i]);
      }
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    ChassisSpeeds measuredSpeeds = getChassisSpeeds();
    RobotState.mInstance.setRobotVelocity(measuredSpeeds);
    RobotState.mInstance.setGyroYawRateRadPerSec(
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : measuredSpeeds.omegaRadiansPerSecond);

    if (DriverStation.isEnabled() && !lastEnabled) {
      coastRequest = CoastRequest.AUTOMATIC;
    }
  }

  /**
   * Adds one odometry sample to RobotState.
   *
   * @param gyroYaw the gyro yaw for this sample, or empty to integrate the heading from the wheels
   */
  private void addOdometrySample(
      SwerveModulePosition[] modulePositions, Optional<Rotation2d> gyroYaw, double timestamp) {
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) { // can add here skid detection
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    if (gyroYaw.isPresent()) {
      // Use the real gyro angle
      rawGyroRotation = gyroYaw.get();
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    RobotState.mInstance.addOdometryObservation(
        new OdometryObservation(modulePositions, Optional.of(rawGyroRotation), timestamp));
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
            speeds, // The desired target speeds, not discretized: the generator discretizes itself
            Constants.CYCLE_TIME.in(edu.wpi.first.units.Units.Seconds) // The loop time of the robot code
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

  /** How far the robot is tilted from flat in any direction, 0 without a gyro. */
  public double getTiltDegrees() {
    return gyroInputs.connected
        ? Math.hypot(gyroInputs.pitchDegrees, gyroInputs.rollDegrees)
        : 0.0;
  }

  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }

  /** Whether every drive and turn motor is connected. */
  public boolean areModulesConnected() {
    for (var module : modules) {
      if (!module.isConnected()) return false;
    }
    return true;
  }

  /** Self test: every module to the same angle and wheel speed, no kinematics or setpoint limits. */
  public void runModuleTest(Rotation2d angle, double speedMetersPerSec) {
    for (var module : modules) {
      module.runSetpoint(new SwerveModuleState(speedMetersPerSec, angle));
    }
  }

  /**
   * Self test: whether every module points along {@code angle}, either way round (modules flip
   * instead of turning past 90 degrees).
   */
  public boolean modulesAlong(Rotation2d angle, double toleranceDegrees) {
    for (var module : modules) {
      double error = Math.abs(Math.IEEEremainder(module.getAngle().minus(angle).getDegrees(), 180.0));
      if (error > toleranceDegrees) return false;
    }
    return true;
  }

  /** Self test: the slowest module's wheel speed. */
  public double slowestModuleSpeed() {
    double slowest = Double.POSITIVE_INFINITY;
    for (var module : modules) {
      slowest = Math.min(slowest, Math.abs(module.getVelocityMetersPerSec()));
    }
    return slowest;
  }

  /** Changes every drive motor's current limits while running (power management). */
  public void setDriveCurrentLimits(double statorAmps, double supplyAmps) {
    for (var module : modules) {
      module.setDriveCurrentLimits(statorAmps, supplyAmps);
    }
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