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

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.SimSparkMaxIO;
import frc.lib.subsystems.SparkMaxIO;
import frc.lib.util.AllianceFlipping;
import frc.lib.util.TableLoader;
import frc.robot.SuperStructure.StructureIntakeStates;
import frc.robot.SuperStructure.SuperStructureStates;
import frc.robot.autonomous.DepotAuto;
import frc.robot.controllers.ControllerInterface;
import frc.robot.controllers.DummyController;
import frc.robot.controllers.SimulationController;
import frc.robot.controllers.SingleXboxController;
import frc.robot.controllers.TwoControllers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeDeployConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final RobotState robotState;
  // // Subsystems
  private final BeltDrive beltDrive;
  private final Drive drive;
  private final Hood hood;
  // private final Hopper hopper;
  private final Intake intake;
  private final Leds leds;
  private final Shooter shooter;
  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null;

  private final SuperStructure structure;

  // Controller
  private final ControllerInterface controller;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    Constants.RobotState.shooterTableData = TableLoader.loadFromCSV("shootingSolutions.csv");
    robotState = RobotState.getInstance();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // controller = new twoco();
        controller = new TwoControllers();
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {},
                controller::xVelocityAnalog,
                controller::yVelocityAnalog,
                controller::rotationVelocityAnalog,
                controller.xLockOverride());

        beltDrive =
            new BeltDrive(
                BeltDriveConstants.BELT_DRIVE_CONFIG,
                new SparkMaxIO(BeltDriveConstants.BELT_DRIVE_CONFIG),
                new SparkMaxIO(BeltDriveConstants.BELT_FOLLOWER_CONFIG.config),
                new SparkMaxIO(BeltDriveConstants.BELT_FOLLOWER_CONFIG_LEFT.config));
        hood = new Hood(HoodConstants.HOOD_CONFIG, new SparkMaxIO(HoodConstants.HOOD_CONFIG));
        // hopper =
        //     new Hopper(
        //         HopperConstants.HOPPER_CONFIG, 
        //         new DummyMotorIO(HopperConstants.HOPPER_CONFIG), 
        //         // new SparkMaxIO(HopperConstants.HOPPER_CONFIG), 
        //         controller.purgeIntakeButton()
        //       );
        intake =
            new Intake(
                IntakeRollerConstants.INTAKE_ROLLER_CONFIG,
                new SparkMaxIO(IntakeRollerConstants.INTAKE_ROLLER_CONFIG),
                IntakeDeployConstants.INTAKE_DEPLOY_CONFIG,
                new SparkMaxIO(IntakeDeployConstants.INTAKE_DEPLOY_CONFIG));
        leds = new Leds();
        shooter =
          new Shooter(
            ShooterConstants.MAIN_WHEEL_MOTOR_CONFIG, 
            ShooterConstants.HOOD_WHEEL_MOTOR_CONFIG, 
            new MotorIO[] {new SparkMaxIO(ShooterConstants.MAIN_WHEEL_MOTOR_CONFIG), new SparkMaxIO(ShooterConstants.MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG.config)}, 
            new MotorIO[] {new SparkMaxIO(ShooterConstants.HOOD_WHEEL_MOTOR_CONFIG), new SparkMaxIO(ShooterConstants.HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG.config)}
            );
        vision =
            new Vision(
                robotState::addVisionObservation,
                new VisionIO[] {
                  // new VisionIOPhoton("BLcamera", VisionConstants.robotToBLcamera)
                  
                //   ,
                //   new VisionIOPhoton("BRcamera", VisionConstants.robotToBRcamera),
                  new VisionIOLimelight("limelight-tsachi", () -> RobotState.getInstance().getEstimatedPose().getRotation())
                });
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        controller = new SimulationController();

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                (robotPose) -> driveSimulation.getSimulatedDriveTrainPose(),
                controller::xVelocityAnalog,
                controller::yVelocityAnalog,
                controller::rotationVelocityAnalog,
                controller.xLockOverride());

        beltDrive =
            new BeltDrive(
          BeltDriveConstants.BELT_DRIVE_CONFIG, 
          new SimSparkMaxIO(BeltDriveConstants.BELT_DRIVE_CONFIG),
          new SimSparkMaxIO(BeltDriveConstants.BELT_FOLLOWER_CONFIG.config),
          new SimSparkMaxIO(BeltDriveConstants.BELT_FOLLOWER_CONFIG_LEFT.config));

        hood = new Hood(HoodConstants.HOOD_CONFIG, new SimSparkMaxIO(HoodConstants.HOOD_CONFIG));
        // hopper =
        //     new Hopper(
        //         HopperConstants.HOPPER_CONFIG, new SimSparkMaxIO(HopperConstants.HOPPER_CONFIG), controller.purgeIntakeButton());
        intake =
            new Intake(
                IntakeRollerConstants.INTAKE_ROLLER_CONFIG,
                new SimSparkMaxIO(IntakeRollerConstants.INTAKE_ROLLER_CONFIG),
                IntakeDeployConstants.INTAKE_DEPLOY_CONFIG,
                new SimSparkMaxIO(IntakeDeployConstants.INTAKE_DEPLOY_CONFIG));
        leds = new Leds();
        shooter =
          new Shooter(
            ShooterConstants.MAIN_WHEEL_MOTOR_CONFIG, 
            ShooterConstants.HOOD_WHEEL_MOTOR_CONFIG, 
            new MotorIO[] {new SimSparkMaxIO(ShooterConstants.MAIN_WHEEL_MOTOR_CONFIG), new SimSparkMaxIO(ShooterConstants.MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG.config)}, 
            new MotorIO[] {new SimSparkMaxIO(ShooterConstants.HOOD_WHEEL_MOTOR_CONFIG), new SimSparkMaxIO(ShooterConstants.HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG.config)}
            );
        vision =
            new Vision(
                robotState::addVisionObservation,
                new VisionIO[] {
                  new VisionIOPhotonSim(
                      "BLcamera",
                      VisionConstants.robotToBLcamera,
                      driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonSim(
                      "BRcamera",
                      VisionConstants.robotToBRcamera,
                      driveSimulation::getSimulatedDriveTrainPose)
                  // new VisionIOTest()
                });
        break;

      default:
        // Replayed robot, disable IO implementations
        controller = new SimulationController();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {},
                controller::xVelocityAnalog,
                controller::yVelocityAnalog,
                controller::rotationVelocityAnalog,
                controller.xLockOverride());
        beltDrive = new BeltDrive(
          BeltDriveConstants.BELT_DRIVE_CONFIG, 
          new SimSparkMaxIO(BeltDriveConstants.BELT_DRIVE_CONFIG), 
          new SimSparkMaxIO(BeltDriveConstants.BELT_FOLLOWER_CONFIG.config),
          new SimSparkMaxIO(BeltDriveConstants.BELT_FOLLOWER_CONFIG_LEFT.config));

        hood = new Hood(HoodConstants.HOOD_CONFIG, new SimSparkMaxIO(HoodConstants.HOOD_CONFIG));
        // hopper =
        //     new Hopper(
        //         HopperConstants.HOPPER_CONFIG, new SimSparkMaxIO(HopperConstants.HOPPER_CONFIG), controller.purgeIntakeButton());
        intake =
            new Intake(
                IntakeRollerConstants.INTAKE_ROLLER_CONFIG,
                new SimSparkMaxIO(IntakeRollerConstants.INTAKE_ROLLER_CONFIG),
                IntakeDeployConstants.INTAKE_DEPLOY_CONFIG,
                new SimSparkMaxIO(IntakeDeployConstants.INTAKE_DEPLOY_CONFIG));
        leds = new Leds();
        shooter =
          new Shooter(
            ShooterConstants.MAIN_WHEEL_MOTOR_CONFIG, 
            ShooterConstants.HOOD_WHEEL_MOTOR_CONFIG, 
            new MotorIO[] {new SimSparkMaxIO(ShooterConstants.MAIN_WHEEL_MOTOR_CONFIG), new SimSparkMaxIO(ShooterConstants.MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG.config)}, 
            new MotorIO[] {new SimSparkMaxIO(ShooterConstants.HOOD_WHEEL_MOTOR_CONFIG), new SimSparkMaxIO(ShooterConstants.HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG.config)}
            );
        vision = new Vision(robotState::addVisionObservation, new VisionIO[] {});
        break;
    }

    structure =
        new SuperStructure(
            beltDrive, drive, hood, intake, leds, shooter, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.print("Doing nothing"));
    autoChooser.addOption("Depot Middle Auto", new DepotAuto(beltDrive, drive, hood, intake, leds, shooter));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> RobotState.getInstance().resetPose(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                RobotState.getInstance()
                    .resetPose(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            AllianceFlipping.apply(Rotation2d.fromDegrees(0))));

    controller.resetGyroButton().onTrue(Commands.runOnce(resetGyro).ignoringDisable(true).alongWith(
    Commands.print("reset gyro")
    ));
    controller.shootCloseButton().onTrue(structure.shootCloseButtonCommand());
    controller.shootButton().onTrue(structure.shootButtonCommand());
    controller.intakeButton().whileTrue(structure.setIntakeStateCommand(StructureIntakeStates.INTAKING));
    controller.intakeButton().whileFalse(structure.setIntakeStateCommand(StructureIntakeStates.CLOSED));
    controller.fetchButton().onTrue(structure.fetchButtonCommand());
    // controller.fetchButton().onFalse(structure.setWantedStateCommand(SuperStructureStates.TRAVEL));
    controller.purgeIntakeButton().onTrue(structure.purgeIntakeButtonTrueCommand());
    controller.purgeIntakeButton().onFalse(structure.purgeIntakeButtonFalseCommand());
    // controller.intakeButton().onTrue(Commands.run(() ->intake.setState(IntakeStates.SHUFFLE), intake));
    // controller.intakeButton().onFalse(Commands.run(() -> intake.setState(IntakeStates.CLOSED), intake));
    // controller.intakeButton().onTrue(Commands.run(() -> intake.setState(IntakeStates.CLOSED), intake));
    // controller.purgeIntakeButton().onTrue(shooter.shooterHoodSysidRoutine(true, Direction.kReverse));
    // controller.intakeButton().onTrue(drive.driveMotorSysIdCommand(true, Direction.kForward));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return Commands.print("aodkpoakf");
  }

  public void periodic() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    // SmartDashboard.putData(
    //     "rgtrh",
    //     new AlignAlgaeReefCommand(drive, arm, elevator, gripper, false, () -> false, () ->
    // true));
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    RobotState.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d()));
    // SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }

  public void autonomousInit() {
    if(!hood.wasHoodReset()) 
      hood.startBootSequence();
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Reset the simulation to the initial pose
    // driveSimulation.setSimulationWorldPose(RobotState.getInstance().getEstimatedPose());
  }

  public void disabledInit() {
    hood.stopBootSequence();
  }
  
  public void teleopInit() {
    if(!hood.wasHoodReset()) 
      hood.startBootSequence();
  }
}
