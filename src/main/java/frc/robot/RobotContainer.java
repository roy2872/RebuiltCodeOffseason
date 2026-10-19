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
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.auto.ModularAutoChooser;
import frc.lib.util.AllianceFlipping;
import frc.robot.auto.BumpCrossing;
import frc.robot.auto.RebuiltAutos;
import frc.robot.power.PowerManager;
import java.util.Optional;
import frc.lib.util.TableLoader;
import frc.lib.visualization.RobotVisualizer;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SelfTest;
import frc.robot.controllers.ControllerInterface;
import frc.robot.controllers.SwitchableController;
import frc.robot.simulation.RobotFuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.CameraConfig;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.subsystems.vision.VisionRoiController.RobotMotion;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final ControllerInterface controller;

  private final Vision vision;
  private final ModularAutoChooser autoChooser;
  private final SelfTest selfTest;
  private RobotFuelSim fuelSim = null; // Sim only

  /** Simulator state that robot code acts on. Always false / zero on the real robot. */
  @AutoLog
  public static class SimulationInputs {
    public boolean isSimulation = false;
    public Pose2d groundTruthPose = Pose2d.kZero;
  }

  private final SimulationInputsAutoLogged simInputs = new SimulationInputsAutoLogged();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    Constants.RobotState.shooterTableData = TableLoader.loadFromCSV("hubSolutions.csv");
    Constants.RobotState.fetchingTableData = TableLoader.loadDoubleMapFromCSV("fetchingSolutions.csv");
    RobotVisualization.configure();
    // Picked on the dashboard: Xbox by default on the robot, the simulation mapping otherwise
    controller = new SwitchableController();
    if (Constants.currentMode == Constants.Mode.SIM) {
      fuelSim = new RobotFuelSim(
          Drive.mInstance.driveSimulation::getSimulatedDriveTrainPose,
          Drive.mInstance.driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);
    }

    vision =
        new Vision(
            (pose, timestamp, stdDevs) -> {
              // Still read and logged by Vision, just not fused
              if (!DashboardOverrides.DISABLE_VISION.get()) {
                RobotState.mInstance.addVisionObservation(pose, timestamp, stdDevs);
              }
            },
            limelight(VisionConstants.tsachiCamera, VisionIOLimelight.Options.DEFAULT));

    // The real robot has no fuel sensor; in sim the fuel sim counts the fuel for the self test
    selfTest =
        new SelfTest(
            vision, fuelSim == null ? Optional.empty() : Optional.of(fuelSim::getFuelHeld));

    // Set up auto routines: Auto/Start, Auto/Part 1..5 and Auto/Preset on the dashboard
    autoChooser = RebuiltAutos.createChooser();
    configureButtonBindings();
  }

  /**
   * A Limelight on the robot. In simulation it becomes a PhotonVision sim camera with the same optics
   * and mounting, so localization can be tested without the hardware.
   */
  private static VisionIO limelight(CameraConfig config, VisionIOLimelight.Options options) {
    return switch (Constants.currentMode) {
      case REAL ->
          new VisionIOLimelight(
              config, options, RobotState.mInstance::getRotation, RobotContainer::getRobotMotion);
      case SIM ->
          new VisionIOPhotonSim(
              config,
              VisionIOPhoton.Options.DEFAULT,
              VisionIOPhotonSim.SimOptions.DEFAULT,
              Drive.mInstance.driveSimulation::getSimulatedDriveTrainPose,
              RobotState.mInstance::getRotation);
      // Replayed robot: no hardware, inputs come from the log. Keeps the name so the log keys match.
      default -> new VisionIO() {
        @Override
        public String getName() {
          return config.name();
        }
      };
    };
  }

  private static RobotMotion getRobotMotion() {
    ChassisSpeeds speeds = RobotState.mInstance.getRobotVelocity();
    return new RobotMotion(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        RobotState.mInstance.getGyroYawRateRadPerSec(),
        speeds.omegaRadiansPerSecond);
  }

  private void configureButtonBindings() {

    final Runnable resetGyro =
        () -> RobotState.mInstance.resetPose(
            simInputs.isSimulation
                ? simInputs.groundTruthPose
                : new Pose2d(
                    RobotState.mInstance.getEstimatedPose().getTranslation(),
                    AllianceFlipping.apply(Rotation2d.fromDegrees(0))));

    Drive.mInstance.setDefaultCommand(
      DriveCommands.joystickDrive(Drive.mInstance, 
        controller::xVelocityAnalog, controller::yVelocityAnalog, controller::rotationVelocityAnalog,
        DashboardOverrides.ROBOT_RELATIVE::get));

    controller.resetGyroButton().onTrue(Commands.runOnce(resetGyro).ignoringDisable(true).alongWith(
    Commands.print("reset gyro")
    ));

    bindShoot(controller.shootButton(), true);

    controller.intakeButton().whileTrue(SuperStructure.mInstance.intakeCommand());

    controller.closeIntakeButton().whileTrue(SuperStructure.mInstance.stopCloseIntakeCommand());

    controller.intakeForceHomeButton().onTrue(SuperStructure.mInstance.intakeForceHomeCommand());

    bindShoot(controller.fetchButton(), false);

    controller.purgeIntakeButton().whileTrue(SuperStructure.mInstance.intakeExhaustCommand());

    // While held: line up diagonally with the nearest bump and drive over it slowly
    controller.alignToBumpButton().whileTrue(BumpCrossing.crossNearestCommand());

    DashboardOverrides.bindOneShot(
        DashboardOverrides.INTAKE_FORCE_HOME, SuperStructure.mInstance.intakeForceHomeCommand());
    DashboardOverrides.bindOneShot(
        DashboardOverrides.HOOD_FORCE_HOME, SuperStructure.mInstance.hoodForceHomeCommand());
    DashboardOverrides.bindOneShot(DashboardOverrides.RUN_SELF_TEST, selfTest.command());
  }

  /**
   * Shoots while {@code button} is held. With Disable Shoot Auto Align on, the shot doesn't take the
   * drive, so the driver keeps driving and aims by hand.
   *
   * @param atHub true to score in the hub, false to ferry
   */
  private static void bindShoot(Trigger button, boolean atHub) {
    Trigger autoAlignOff = new Trigger(DashboardOverrides.DISABLE_SHOOT_AUTO_ALIGN::get);
    button.onTrue(Commands.runOnce(() -> RobotState.mInstance.setShootType(atHub)));
    button.and(autoAlignOff.negate()).whileTrue(SuperStructure.mInstance.shoot(true));
    button.and(autoAlignOff).whileTrue(SuperStructure.mInstance.shoot(false));
    button.onFalse(SuperStructure.mInstance.stopShooting());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getRoutine();
  }

  public void periodic() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    DashboardOverrides.periodic();
    HubShiftTracker.mInstance.periodic();
    autoChooser.periodic();
    PowerManager.mInstance.periodic();
    RobotState.mInstance.periodic();
    RobotVisualizer.mInstance.periodic();
  }

  /**
   * Logs the simulator state that robot code acts on. In replay it is read back from the log, so a
   * log recorded in simulation replays the same way. Call before anything reads {@link #simInputs}.
   */
  public void updateSimulationInputs() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      simInputs.isSimulation = true;
      simInputs.groundTruthPose = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose();
    }
    Logger.processInputs("Simulation", simInputs);
  }

  public void resetSimulation() {
    updateSimulationInputs();
    if (!simInputs.isSimulation) return;

    // Move the simulated robot too. Resetting only the estimate leaves it offset from the true pose, and
    // vision never corrects heading, so aiming would be off by however far the robot had turned.
    Pose2d resetPose = new Pose2d(3, 3, new Rotation2d());
    if (Constants.currentMode == Constants.Mode.SIM) {
      Drive.mInstance.driveSimulation.setSimulationWorldPose(resetPose);
    }
    RobotState.mInstance.resetPose(resetPose);
    // SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    fuelSim.periodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose());
  }

  public void autonomousInit() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    fuelSim.reset();
    // Reset the simulation to the initial pose
    // driveSimulation.setSimulationWorldPose(RobotState.getInstance().getEstimatedPose());
  }

  public void disabledInit() {
  }
  
  /** The pit self test, run when Test mode starts. */
  public Command getSelfTestCommand() {
    return selfTest.command();
  }

  public void teleopInit() {
    HubShiftTracker.mInstance.onTeleopInit();
  }
}
