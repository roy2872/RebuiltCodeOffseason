package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** High-rate odometry: several samples per loop, and the estimate follows the simulated robot. */
class DriveOdometrySimTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
  }

  @Test
  void odometryFollowsTheSimulatedRobotWhileDrivingAndTurning() {
    // Open floor in the neutral zone, so the robot doesn't hit a field element while its wheels spin
    Pose2d start = new Pose2d(6.0, 2.0, Rotation2d.kZero);
    Drive.mInstance.driveSimulation.setSimulationWorldPose(start);
    RobotState.mInstance.resetPose(start);
    step();

    for (int i = 0; i < 100; i++) { // 2 s
      Drive.mInstance.runVelocity(new ChassisSpeeds(1.5, 0.5, 1.0));
      step();
    }
    Drive.mInstance.stop();

    Pose2d truth = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose();
    Pose2d odometry = RobotState.mInstance.getOdometryPose();
    double error = truth.getTranslation().getDistance(odometry.getTranslation());
    assertTrue(error < 0.1, "odometry is " + error + " m from the true pose");
    assertEquals(
        truth.getRotation().getDegrees(), odometry.getRotation().getDegrees(), 3.0, "heading drifted");
  }

  private static void step() {
    SimHooks.stepTiming(0.02);
    CommandScheduler.getInstance().run();
    SimulatedArena.getInstance().simulationPeriodic();
  }
}
