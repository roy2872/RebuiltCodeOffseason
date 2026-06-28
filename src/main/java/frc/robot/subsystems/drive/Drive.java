// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.lib.accelLimitsLib.*;
import static frc.robot.Constants.FIELD_WIDTH;
import static frc.robot.subsystems.drive.DriveConstants.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.util.LocalADStarAK;
import frc.lib.util.SwerveUtils;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public enum DriveStates {
    FIELD_DRIVE,
    ROBOT_DRIVE,
    SHOOT_DRIVE,
    SHOOT_ON_THE_MOVE,
    AUTO_ALIGN,
    AUTO_ALIGN_ANGLE,
    AUTONOMOUS,
    SLOWLY_FORWARD,
    PATH_AND_SHOOT,
    CHOREO_PATH_FOLLOWING,
    X_LOCK,
    IDLE
  }

  @AutoLogOutput(key = "Drive/DriveState")
  private DriveStates driveState = DriveStates.FIELD_DRIVE;

  // TunerConstants doesn't include these constants, so they are declared locally
  // static final double ODOMETRY_FREQUENCY =
  //     new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private static final double ROBOT_MASS_KG = 53;
  private static final double ROBOT_MOI = 4.0;
  private static final double WHEEL_COF = 1.0;
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
          getModuleTranslations());

  // static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  // private SwerveDrivePoseEstimator poseEstimator =
  //     new SwerveDrivePoseEstimator(
  //         kinematics,
  //         rawGyroRotation,
  //         lastModulePositions,
  //         new Pose2d()); // can switch this with potential kalman filter

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  private final DoubleSupplier xJoystickVelocity, yJoystickVelocity, rJoystickVelocity;
  private final Trigger xLockOverrideButton;
  private final Trigger alignToBumpOverrideButton;

  private Supplier<Pose2d> autoAlignTarget;
  private Supplier<Rotation2d> autoAlignAngleTarget;
  private Supplier<Rotation2d> shootDriveTargetAngle;
  private Supplier<Rotation2d> shootMoveDriveTargetAngle;
  private Supplier<Translation2d> shootMoveDriveTargetTranslation;

  private final PIDController linearVelocityController = new PIDController(4.3, 0, 0); // kp 1.2
  private final PIDController rotationController = new PIDController(0.14, 0, 0); // kp 0.8
  private final PIDController choreoXController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController choreoYController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController choreoHeadingController = new PIDController(7.5, 0.0, 0.0);
  private final Timer choreoTimer = new Timer();
  private final SendableChooser<String> choreoPathChooser = new SendableChooser<>();
  private Optional<Trajectory<SwerveSample>> selectedChoreoTrajectory = Optional.empty();
  private String selectedChoreoPathName = "";
  private String activeChoreoPathName = "";
  private boolean choreoPathFinished = true;
  private boolean choreoOwnsDriveState = false;
  private boolean choreoMirrorLeftRight = false;
  private double maxAutoAlignVelocity = 6.0;
  private double maxAutoAlignAngularVelocity = 100.0;

  private SysIdRoutine driveRoutine;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack,
      DoubleSupplier xJoystickVelocity,
      DoubleSupplier yJoystickVelocity,
      DoubleSupplier rJoystickVelocity,
      Trigger xLockOverrideButton,
      Trigger alignToBumpOverrideButton) {

    rotationController.enableContinuousInput(0, 360);
    rotationController.setTolerance(2);
    choreoHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    // PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        RobotState.getInstance()::getEstimatedPose,
        RobotState.getInstance()::resetPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    RobotConfig config = PP_CONFIG;
    setpointGenerator =
        new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating
            // trajectories and running path following commands.
            Units.rotationsToRadians(
                10.0) // The max rotation velocity of a swerve module in radians per second. This
            // should probably be stored in your Constants file
            );
    // Initialize the previous setpoint to the robot's current speeds & module states
    ChassisSpeeds currentSpeeds =
        getChassisSpeeds(); // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates =
        getModuleStates(); // Method to get the current swerve module states
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));

    this.xJoystickVelocity = xJoystickVelocity;
    this.yJoystickVelocity = yJoystickVelocity;
    this.rJoystickVelocity = rJoystickVelocity;
    this.xLockOverrideButton = xLockOverrideButton;
    this.alignToBumpOverrideButton = alignToBumpOverrideButton;

    shootDriveTargetAngle = () -> Rotation2d.fromDegrees(RobotState.getInstance().getShootingInfo().get(2));
    shootMoveDriveTargetAngle = () -> Rotation2d.fromDegrees(RobotState.getInstance().getShootOnTheMoveScoringInfo().get(2));

    String[] availableChoreoPaths = Choreo.availableTrajectories();
    Arrays.sort(availableChoreoPaths);
    if (availableChoreoPaths.length > 0) {
      choreoPathChooser.setDefaultOption(availableChoreoPaths[0], availableChoreoPaths[0]);
      for (int i = 1; i < availableChoreoPaths.length; i++) {
        choreoPathChooser.addOption(availableChoreoPaths[i], availableChoreoPaths[i]);
      }
      setSelectedChoreoPath(availableChoreoPaths[0]);
    }
    SmartDashboard.putData("Drive/Choreo Path", choreoPathChooser);

    maxAutoAlignVelocity = getMaxLinearSpeedMetersPerSec();
    maxAutoAlignAngularVelocity = getMaxAngularSpeedRadPerSec();

    driveRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.per(Second).of(1),
      Volts.of(4),
      Seconds.of(7),
      (state) -> Logger.recordOutput("Drive/DriveSysidState", state.toString())
    ), 
    new SysIdRoutine.Mechanism(
      (volt) -> this.runCharacterization(volt.in(Volts)),
      null,
      this
    ));
    }
    
    
  @Override
  public void periodic() {
    // odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    // odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
    // // Update odometry
    // double[] sampleTimestamps =
    //     modules[0].getOdometryTimestamps(); // All signals are sampled together
    // int sampleCount = sampleTimestamps.length;
    // for (int i = 0; i < sampleCount; i++) {
    //   // Read wheel positions and deltas from each module
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

    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                modulePositions, Optional.of(rawGyroRotation), RobotController.getFPGATime() * 1e-6));

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    if(xLockOverrideButton.getAsBoolean()) xLock();
     else stateMachine();
  }

  public void teleopInit() {
    driveState = DriveStates.FIELD_DRIVE;
  }

  public void autonomousInit() {
    driveState = DriveStates.AUTONOMOUS;
  }

  private void stateMachine() {

    switch (driveState) {
      case IDLE:
        stop();
        break;

      case FIELD_DRIVE:
        fieldCentricJoystickDrive(
            xJoystickVelocity.getAsDouble(),
            yJoystickVelocity.getAsDouble(),
            rJoystickVelocity.getAsDouble());
        break;

      case ROBOT_DRIVE: // Robot drives autonomously
        robotCentricJoystickDrive(
            xJoystickVelocity.getAsDouble(),
            yJoystickVelocity.getAsDouble(),
            rJoystickVelocity.getAsDouble());
        break;

      case SHOOT_DRIVE: // Field drive but with assistance from the robot
        shootDriveTargetAngle =
            shootDriveTargetAngle != null ? shootDriveTargetAngle : () -> new Rotation2d();
          double rotVal =
            rotationController.calculate(
                (RobotState.getInstance().getEstimatedPose().getRotation().getDegrees() % 360 + 360)
                    % 360,
                (shootDriveTargetAngle.get().getDegrees() % 360 + 360) % 360);
        runVelocity(
            new ChassisSpeeds(
                0, 0, rotVal));  
        break;

      case SHOOT_ON_THE_MOVE:
        shootMoveDriveTargetAngle =
            shootMoveDriveTargetAngle != null ? shootMoveDriveTargetAngle : () -> new Rotation2d();
          double rotationVal =
            rotationController.calculate(
                (RobotState.getInstance().getEstimatedPose().getRotation().getDegrees() % 360 + 360)
                    % 360,
                (shootMoveDriveTargetAngle.get().getDegrees() % 360 + 360) % 360);
        fieldCentricJoystickDriveShooting(xJoystickVelocity.getAsDouble(), yJoystickVelocity.getAsDouble(), rotationVal);
        break;
        
      case PATH_AND_SHOOT:
        pathAndShoot();
        break;

      case CHOREO_PATH_FOLLOWING:
        updateChoreoPath();
        break;

      case AUTO_ALIGN:
        autoAlign();
        break;

      case AUTO_ALIGN_ANGLE:
       autoAlignAngle();
        break;
      
      case X_LOCK:
        xLock();
        break;
        
      default:
        System.out.println("Drive subsystem is really broken");
        break;
    }
  }

  public void setDriveState(DriveStates state) {
    if (driveState == state) return;
    driveState = state;
  }

  private void xLock() {
    modules[0].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + 90 * 0)));
    modules[1].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + 90 * 1)));
    modules[2].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + 90 * 3)));
    modules[3].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + 90 * 2)));
  }

  private double getAlignToBumpVelocity() {
    double yaw = RobotState.getInstance().getEstimatedPose().getRotation().getDegrees();
    double closestAngle = 
      SwerveUtils.getClosestDiagonalAngle(yaw);
    Rotation2d targetAngle = Rotation2d.fromDegrees(closestAngle);
    double rot =
            rotationController.calculate(
                (yaw % 360 + 360)
                    % 360,
                (targetAngle.getDegrees() % 360 + 360) % 360);
    rot = Math.signum(rot) * Math.min(Math.abs(rot), maxAutoAlignAngularVelocity);
    return rot;    
  }

  private void fieldCentricJoystickDrive(double vx, double vy, double vr) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(vx, vy);

    linearVelocity =
        linearVelocity.getDistance(new Translation2d()) < 0.07
            ? new Translation2d()
            : linearVelocity;

    double omega = MathUtil.applyDeadband(vr, DriveConstants.DEADBAND);
    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
            omega * getMaxAngularSpeedRadPerSec());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? RobotState.getInstance()
                    .getEstimatedPose()
                    .getRotation()
                    .plus(new Rotation2d(Math.PI))
                : RobotState.getInstance().getEstimatedPose().getRotation()));
  }

  private void fieldCentricJoystickDriveShooting(double vx, double vy, double vr) {
    // could add here acceleration and velocity limits
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(vx, vy);

    linearVelocity =
        linearVelocity.getDistance(new Translation2d()) < 0.07
            ? new Translation2d()
            : linearVelocity;

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec() * SHOOT_ON_THE_MOVE_DRIVE_PERCENTAGE,
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec() * SHOOT_ON_THE_MOVE_DRIVE_PERCENTAGE,
            vr); 

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    ChassisSpeeds wantedVelocitiesRobotOriented = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? RobotState.getInstance()
                    .getEstimatedPose()
                    .getRotation()
                    .plus(new Rotation2d(Math.PI))
                : RobotState.getInstance().getEstimatedPose().getRotation());
    // wantedVelocitiesRobotOriented = applyShootOnTheMoveLimit(wantedVelocitiesRobotOriented, getChassisSpeeds()); not working rn
    runVelocity(wantedVelocitiesRobotOriented);
  }

  private void robotCentricJoystickDrive(double vx, double vy, double vr) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(vx, vy);

    double omega = MathUtil.applyDeadband(vr, DriveConstants.DEADBAND);
    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    runVelocity(
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
            omega * getMaxAngularSpeedRadPerSec()));
  }

  private void pathAndShoot() {
        shootMoveDriveTargetTranslation =
            shootMoveDriveTargetTranslation != null ? shootMoveDriveTargetTranslation : () -> new Translation2d(3, 3);

        Translation2d distance =
            shootMoveDriveTargetTranslation.get().minus(RobotState.getInstance().getEstimatedPose().getTranslation());
        double linearVelocity =
            linearVelocityController.calculate(0, Math.hypot(distance.getX(), distance.getY()));
        linearVelocity = Math.min(linearVelocity, maxAutoAlignVelocity);
        Translation2d linearVelocityTranslation =
            new Translation2d(
                linearVelocity, new Rotation2d(Math.atan2(distance.getY(), distance.getX())));
        double rot =
            rotationController.calculate(
                (RobotState.getInstance().getEstimatedPose().getRotation().getDegrees() % 360 + 360)
                    % 360,
                (shootMoveDriveTargetAngle.get().getDegrees() % 360 + 360) % 360);
        rot = Math.signum(rot) * Math.min(Math.abs(rot), maxAutoAlignAngularVelocity);
        runVelocityFieldRelative(
            new ChassisSpeeds(
                linearVelocityTranslation.getX(), linearVelocityTranslation.getY(), rot));
  }

  public void runVelocity(ChassisSpeeds speeds) {

    if(alignToBumpOverrideButton.getAsBoolean()) speeds.omegaRadiansPerSecond = getAlignToBumpVelocity();
    RobotState.getInstance().addDriveSpeeds(speeds);
    // speeds = accelLimitsLib.applyAccLimits(speeds, getChassisSpeeds());
    // // Calculate module setpoints
    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", previousSetpoint.moduleStates());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(previousSetpoint.moduleStates()[i]);
    }
  }

  public void runVelocityFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {

    Rotation2d robotRotation =
        RobotState.getInstance().getEstimatedPose().getRotation();

    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond,
            fieldRelativeSpeeds.omegaRadiansPerSecond,
            robotRotation);

    runVelocity(robotRelativeSpeeds);
}

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
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
      headings[i] = getModuleTranslations()[i].getAngle();
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

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  private void autoAlignAngle() {
        if(autoAlignAngleTarget == null) return ;
        Rotation2d targetAngle = autoAlignAngleTarget.get();
        double rot =
            rotationController.calculate(
                (RobotState.getInstance().getEstimatedPose().getRotation().getDegrees() % 360 + 360)
                    % 360,
                (targetAngle.getDegrees() % 360 + 360) % 360);
        rot = Math.signum(rot) * Math.min(Math.abs(rot), maxAutoAlignAngularVelocity);
        runVelocityFieldRelative(
            new ChassisSpeeds(
                0, 0, rot));
  }

  // Ensure this is called in your constructor or initialization to fix the angle wrapping issue:
// rotationController.enableContinuousInput(0, 360);

public void autoAlign() {
    // 1. Safe Supplier check (ensures the supplier itself and its provided value aren't null)
    if (autoAlignTarget == null || autoAlignTarget.get() == null) {
        return; // Use 'return' if this is a standard method, or 'break' if inside a loop
    }

    Pose2d targetPose = autoAlignTarget.get();
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    // 2. Calculate distance translation
    Translation2d distance = targetPose.getTranslation().minus(currentPose.getTranslation());
    double currentDistance = Math.hypot(distance.getX(), distance.getY());

    // 3. Corrected PID arguments: calculate(measurement, setpoint)
    // We measure the current distance, and our target goal is 0.
    double linearVelocity = linearVelocityController.calculate(currentDistance, 0);
    linearVelocity = Math.min(linearVelocity, maxAutoAlignVelocity);

    Translation2d linearVelocityTranslation = new Translation2d(
        -linearVelocity, 
        new Rotation2d(Math.atan2(distance.getY(), distance.getX()))
    );

    // 4. Cleaned up rotation angles (Ensure continuous input is enabled on the controller)
    double currentAngle = (currentPose.getRotation().getDegrees() % 360 + 360) % 360;
    double targetAngle = (targetPose.getRotation().getDegrees() % 360 + 360) % 360;

    double rot = rotationController.calculate(currentAngle, targetAngle);
    
    // 5. Clean clamping using WPILib MathUtil
    rot = edu.wpi.first.math.MathUtil.clamp(rot, -maxAutoAlignAngularVelocity, maxAutoAlignAngularVelocity);

    // 6. Drive
    runVelocityFieldRelative(
        new ChassisSpeeds(
            linearVelocityTranslation.getX(), 
            linearVelocityTranslation.getY(), 
            rot
        )
    );
}

  private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public void updateChoreoPath() {

    double t = choreoTimer.get();
    Trajectory<SwerveSample> trajectory = selectedChoreoTrajectory.get();
    if (t > trajectory.getTotalTime()) {
      stopChoreoPath();
      return;
    }

    boolean mirrorForRedAlliance =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Optional<SwerveSample> sample = trajectory.sampleAt(t, mirrorForRedAlliance);
    if (sample.isEmpty()) {
      stopChoreoPath();
      return;
    }

    SwerveSample currentSample = sample.get();
    if (choreoMirrorLeftRight) {
      currentSample = mirrorSampleLeftRight(currentSample);
    }

    followTrajectory(currentSample);
  }

  private void followTrajectory(SwerveSample sample) {
    Pose2d pose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            sample.vx + choreoXController.calculate(pose.getX(), sample.x),
            sample.vy + choreoYController.calculate(pose.getY(), sample.y),
            sample.omega
                + choreoHeadingController.calculate(pose.getRotation().getRadians(), sample.heading));
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, pose.getRotation());
    runVelocity(robotRelativeSpeeds);
  }

  public boolean setSelectedChoreoPath(String pathName, boolean switchSideLeftRight) {
    if (pathName == null || pathName.isBlank()) {
      return false;
    }

    Optional<Trajectory<SwerveSample>> loadedTrajectory = Choreo.<SwerveSample>loadTrajectory(pathName);
    if (loadedTrajectory.isEmpty()) {
      DriverStation.reportError("Could not load Choreo trajectory: " + pathName, false);
      return false;
    }

    choreoMirrorLeftRight = switchSideLeftRight;
    selectedChoreoTrajectory = loadedTrajectory;
    selectedChoreoPathName = pathName;
    Logger.recordOutput("Drive/Choreo/SelectedPath", selectedChoreoPathName);
    return true;
  }

  public boolean setSelectedChoreoPath(String pathName) {
    return setSelectedChoreoPath(pathName, false);
  }

  private boolean startSelectedChoreoPath(boolean resetToInitialPose) {
    if (selectedChoreoTrajectory.isEmpty()) {
      DriverStation.reportWarning("No valid Choreo trajectory selected.", false);
      return false;
    }

    if (resetToInitialPose) {
      boolean mirrorForRedAlliance =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      selectedChoreoTrajectory
        .get()
        .getInitialPose(mirrorForRedAlliance)
        .ifPresent(pose -> {

        if (!choreoMirrorLeftRight) {
          pose = new Pose2d(
            pose.getX(),
            FIELD_WIDTH - pose.getY(),
            new Rotation2d(-pose.getRotation().getRadians())
        );
      }

      RobotState.getInstance().resetPose(pose);
    });
    }

    choreoXController.reset();
    choreoYController.reset();
    choreoHeadingController.reset();
    choreoTimer.restart();
    choreoPathFinished = false;
    activeChoreoPathName = selectedChoreoPathName;
    Logger.recordOutput("Drive/Choreo/ActivePath", activeChoreoPathName);
    setState(DriveStates.CHOREO_PATH_FOLLOWING);
    return true;
  }

  public boolean startChoreoPath(String pathName, boolean resetToInitialPose, boolean switchSideLeftRight) {
    if (!setSelectedChoreoPath(pathName, switchSideLeftRight)) {
      return false;
    }
    return startSelectedChoreoPath(resetToInitialPose);
  }

  public void stopChoreoPath() {
    choreoTimer.stop();
    choreoPathFinished = true;
    choreoOwnsDriveState = false;
    activeChoreoPathName = "";
    Logger.recordOutput("Drive/Choreo/ActivePath", activeChoreoPathName);
    stop();
    if (driveState == DriveStates.CHOREO_PATH_FOLLOWING && choreoOwnsDriveState) {
      driveState = DriveStates.IDLE;
    }
  }

  public boolean isChoreoPathActive() {
    return !choreoPathFinished;
  }

  public boolean isChoreoPathFinished() {
    return choreoPathFinished;
  }

private SwerveSample mirrorSampleLeftRight(SwerveSample s) {

  double mirroredY = FIELD_WIDTH - s.y;

  return new SwerveSample(
      s.t,
      s.x,
      mirroredY,
      -s.heading,
      s.vx,
      -s.vy,
      -s.omega,
      s.ax,
      -s.ay,
      -s.alpha, null, null
  );
}

  public void setState(DriveStates state) {
    if (state == driveState) return;
    driveState = state;
    Logger.recordOutput("Drive/DriveState", driveState.toString());
  }

  /**
   * Automatically aligns the robot to a target pose using a simple PID controller. The target pose is
   * provided as a Supplier, so it can be updated in real time
   * @param targetPose
   * @param maxVelocity [m/s]
   * @param maxAngularVelocity [rad/s]
   */
  public void setStateAutoAlign(Supplier<Pose2d> targetPose, double maxVelocity, double maxAngularVelocity) {
    autoAlignTarget = targetPose;
    this.maxAutoAlignVelocity = maxVelocity;
    this.maxAutoAlignAngularVelocity = maxAngularVelocity;
    if (driveState == DriveStates.AUTO_ALIGN) return;
    driveState = DriveStates.AUTO_ALIGN;
  }

  public void setStateAutoAlign(Supplier<Pose2d> targetPose) {
    autoAlignTarget = targetPose;
    this.maxAutoAlignVelocity = getMaxLinearSpeedMetersPerSec();
    this.maxAutoAlignAngularVelocity = getMaxAngularSpeedRadPerSec();
    if (driveState == DriveStates.AUTO_ALIGN) return;
    driveState = DriveStates.AUTO_ALIGN;
  }

  public void setStateAutoAlignAngle(Supplier<Rotation2d> targetRotation) {
    autoAlignAngleTarget = targetRotation;
    this.maxAutoAlignAngularVelocity = getMaxAngularSpeedRadPerSec();
    driveState = DriveStates.AUTO_ALIGN_ANGLE;
  }

  public void setStatePathAndShoot(Supplier<Translation2d> targetTranslation, double maxVelocity, double maxAngularVelocity) {
     shootMoveDriveTargetTranslation = targetTranslation;
     this.maxAutoAlignVelocity = maxVelocity;
     this.maxAutoAlignAngularVelocity = maxAngularVelocity;
    shootMoveDriveTargetTranslation = targetTranslation;
    if (driveState == DriveStates.PATH_AND_SHOOT) return;
    driveState = DriveStates.PATH_AND_SHOOT;
  }

    public void setStatePathAndShoot(Supplier<Translation2d> targetTranslation) {
      setStatePathAndShoot(targetTranslation, getMaxLinearSpeedMetersPerSec(), getMaxAngularSpeedRadPerSec());
    }

  public DriveStates getState() {
    return driveState;
  }

  public boolean isAtAlignSetpoint(double tolerance, double angleToleranceDegrees) {
    Pose2d currentPose =
        RobotState.getInstance().getEstimatedPose() != null
            ? RobotState.getInstance().getEstimatedPose()
            : new Pose2d(1, 1, new Rotation2d());
    Pose2d targetPose =
        autoAlignTarget != null && autoAlignTarget.get() != null
            ? autoAlignTarget.get()
            : new Pose2d(1, 1, new Rotation2d());

    double difference = Math.abs(currentPose.getRotation().getDegrees() - (targetPose.getRotation().getDegrees()));
    difference = (difference + 360.0) % 360;
    if(difference > 180) difference = 360 - difference;
    
    return (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < tolerance)
        && (difference < angleToleranceDegrees);
  }

  public boolean isAtShootDriveSetpoint(double toleranceDeg) {
    Pose2d currentPose =
    RobotState.getInstance().getEstimatedPose() != null
        ? RobotState.getInstance().getEstimatedPose()
        : new Pose2d(1, 1, new Rotation2d());
    double difference = Math.abs(currentPose.getRotation().getDegrees() - (shootDriveTargetAngle.get().getDegrees()));
    difference = (difference + 360.0) % 360;
    if(difference > 180) difference = 360 - difference;
    return difference <= toleranceDeg;
  }

    public boolean isAtShootOnTheMoveSetpoint(double toleranceDeg) {
    Pose2d currentPose =
    RobotState.getInstance().getEstimatedPose() != null
        ? RobotState.getInstance().getEstimatedPose()
        : new Pose2d(1, 1, new Rotation2d());
    double difference = Math.abs(currentPose.getRotation().getDegrees() - (shootMoveDriveTargetAngle.get().getDegrees()));
    difference = (difference + 360.0) % 360;
    if(difference > 180) difference = 360 - difference;
    return difference <= toleranceDeg;
  }

  public boolean isAtAutoAlignAngleSetpoint(double toleranceDeg) {
    Pose2d currentPose =
    RobotState.getInstance().getEstimatedPose() != null
        ? RobotState.getInstance().getEstimatedPose()
        : new Pose2d(1, 1, new Rotation2d());
    double difference = Math.abs(currentPose.getRotation().getDegrees() - (autoAlignAngleTarget.get().getDegrees()));
    difference = (difference + 360.0) % 360;
    if(difference > 180) difference = 360 - difference;
    return difference <= toleranceDeg;
  }

  public boolean isAtPathAndShootSetpointAngle(double toleranceDeg) {
    Pose2d currentPose =
    RobotState.getInstance().getEstimatedPose() != null
        ? RobotState.getInstance().getEstimatedPose()
        : new Pose2d(1, 1, new Rotation2d());
    double difference = Math.abs(currentPose.getRotation().getDegrees() - (shootMoveDriveTargetAngle.get().getDegrees()));
    difference = (difference + 360.0) % 360;
    if(difference > 180) difference = 360 - difference;
    return difference < toleranceDeg;
  }

   public boolean isAtPathAndShootSetpointTranslation(double tolerance) {
    Pose2d currentPose =
    RobotState.getInstance().getEstimatedPose() != null
        ? RobotState.getInstance().getEstimatedPose()
        : new Pose2d(1, 1, new Rotation2d());
    return currentPose.getTranslation().getDistance(shootMoveDriveTargetTranslation.get()) < tolerance;
   }

   public Command driveMotorSysIdCommand(boolean quasistatic, SysIdRoutine.Direction direction) {
    return Commands.either(
      driveRoutine.quasistatic(direction), 
      driveRoutine.dynamic(direction), 
      () -> quasistatic
      );
   }
}
