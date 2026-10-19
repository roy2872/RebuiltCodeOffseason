package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Drives from the alliance zone into the neutral zone in the physics sim. */
class BumpCrossingSimTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
  }

  @Test
  void crossesOnlyOverABumpDiagonallyAndSlowly() throws InterruptedException {
    Pose2d start = new Pose2d(2.5, 3.0, Rotation2d.k180deg);
    Drive.mInstance.driveSimulation.setSimulationWorldPose(start);
    RobotState.mInstance.resetPose(start);
    step();

    Pose2d target = new Pose2d(7.0, 2.0, Rotation2d.kZero);
    Command command = new RebuiltDriveAdapter().driveTo(target);
    CommandScheduler.getInstance().schedule(command);

    boolean sawCrossing = false;
    boolean sawTilt = false;
    Pose2d last = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose();
    for (int i = 0; i < 1000 && command.isScheduled(); i++) { // up to 20 s
      step();
      Pose2d pose = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose();
      sawTilt |= RobotState.mInstance.isTilted();
      if (Math.abs(pose.getX() - BumpCrossing.BLUE_HUB_LINE_X) < 0.6) {
        sawCrossing = true;
        double bumpError =
            Math.min(
                Math.abs(pose.getY() - BumpCrossing.BUMP_Y[0]),
                Math.abs(pose.getY() - BumpCrossing.BUMP_Y[1]));
        assertTrue(bumpError < 0.4, "on the hub line away from a bump, at " + pose);
        double offDiagonal =
            Math.abs(Math.IEEEremainder(pose.getRotation().getDegrees() - 45.0, 90.0));
        assertTrue(offDiagonal < 10.0, "not diagonal on the bump, at " + pose);
        double speed = pose.getTranslation().getDistance(last.getTranslation()) / 0.02;
        assertTrue(speed < 2.2, "crossed at " + speed + " m/s");
      }
      last = pose;
    }

    assertTrue(sawCrossing, "never crossed the hub line");
    assertTrue(sawTilt, "the gyro never saw the bump");
    assertEquals("Landed", BumpCrossCommand.lastEndReason(), "the crossing didn't end on the gyro");
    assertTrue(!RobotState.mInstance.isTilted(), "still on the bump");
    assertTrue(!command.isScheduled(), "didn't arrive in 20 s");
    Pose2d end = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose();
    assertTrue(
        end.getTranslation().getDistance(target.getTranslation()) < 0.3, "ended at " + end);
  }

  private static void step() throws InterruptedException {
    SimHooks.stepTiming(0.02);
    CommandScheduler.getInstance().run();
    SimulatedArena.getInstance().simulationPeriodic();
    // The pathfinder plans on its own thread in real time
    Thread.sleep(2);
  }
}
