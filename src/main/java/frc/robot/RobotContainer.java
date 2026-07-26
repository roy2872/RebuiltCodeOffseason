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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipping;
import frc.lib.util.TableLoader;
import frc.robot.commands.DriveCommands;
import frc.robot.controllers.ControllerInterface;
import frc.robot.controllers.DummyController;
import frc.robot.controllers.QXController;
import frc.robot.controllers.SimulationController;
import frc.robot.controllers.TwoControllers;
import frc.robot.subsystems.drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final ControllerInterface controller;

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    Constants.RobotState.shooterTableData = TableLoader.loadFromCSV("shootingSolutions.csv");
    Constants.RobotState.fetchingTableData = TableLoader.loadDoubleMapFromCSV("fetchingSolutions.csv");
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // controller = new twoco();
        controller = new TwoControllers(new QXController(0), new DummyController(1));
        break;
      case SIM:
        controller = new SimulationController(0);
        break;

      default:
        // Replayed robot, disable IO implementations
        controller = new SimulationController(0);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.print("Doing nothing"));
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> RobotState.mInstance.resetPose(Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                RobotState.mInstance
                    .resetPose(
                        new Pose2d(
                            RobotState.mInstance.getEstimatedPose().getTranslation(),
                            AllianceFlipping.apply(Rotation2d.fromDegrees(0))));

    Drive.mInstance.setDefaultCommand(
      DriveCommands.joystickDrive(Drive.mInstance, 
        controller::xVelocityAnalog, controller::yVelocityAnalog, controller::rotationVelocityAnalog, () -> false));

    controller.resetGyroButton().onTrue(Commands.runOnce(resetGyro).ignoringDisable(true).alongWith(
    Commands.print("reset gyro")
    ));

    controller.shootButton()
      .whileTrue(Commands.runOnce(() -> RobotState.mInstance.setShootType(true))
      .andThen(SuperStructure.mInstance.shoot())).onFalse(SuperStructure.mInstance.stopShooting());

    controller.intakeButton()
      .onTrue(SuperStructure.mInstance.intakeCommand())
      .onFalse(SuperStructure.mInstance.idleIntakeRollers());

    controller.fetchButton()
      .whileTrue(Commands.runOnce(() -> RobotState.mInstance.setShootType(false))
      .andThen(SuperStructure.mInstance.shoot()))
      .onFalse(SuperStructure.mInstance.stopShooting());

    controller.purgeIntakeButton().onTrue(SuperStructure.mInstance.intakeExhaustCommand())
      .onFalse(SuperStructure.mInstance.idleIntakeRollers());

    // controller.alignToBumpButton().

    // controller.alignToTowerButton().
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    RobotState.mInstance.periodic();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    RobotState.mInstance.resetPose(new Pose2d(3, 3, new Rotation2d()));
    // SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose());
  }

  public void autonomousInit() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Reset the simulation to the initial pose
    // driveSimulation.setSimulationWorldPose(RobotState.getInstance().getEstimatedPose());
  }

  public void disabledInit() {
  }
  
  public void teleopInit() {

  }
}
