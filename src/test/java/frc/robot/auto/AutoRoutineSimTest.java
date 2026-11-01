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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.AutoAction;
import frc.lib.auto.AutoRoutineBuilder;
import frc.lib.auto.AutoStartPosition;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** A routine run in the physics sim, starting from wherever the sim robot was left. */
class AutoRoutineSimTest {
  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
  }

  @Test
  void drivesToTheTargetFromTheSelectedStart() throws InterruptedException {
    // Where disabledInit leaves the sim robot: not the auto start, and turned the other way
    Pose2d leftover = new Pose2d(3.0, 3.0, Rotation2d.kZero);
    Drive.mInstance.driveSimulation.setSimulationWorldPose(leftover);
    RobotState.mInstance.resetPose(leftover);
    step();

    Pose2d target = new Pose2d(2.0, 5.5, Rotation2d.k180deg);
    var start = new AutoStartPosition("Center", new Pose2d(3.5, 4.035, Rotation2d.k180deg));
    Command routine =
        AutoRoutineBuilder.build(
                start,
                List.of(AutoAction.named("Drive").driveTo(target).build()),
                new RebuiltDriveAdapter(),
                pose -> pose,
                Commands::none)
            .command();
    CommandScheduler.getInstance().schedule(routine);
    for (int i = 0; i < 500 && routine.isScheduled(); i++) { // up to 10 s
      step();
    }

    Pose2d end = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose();
    assertTrue(!routine.isScheduled(), "didn't arrive in 10 s, at " + end);
    assertTrue(end.getTranslation().getDistance(target.getTranslation()) < 0.3, "ended at " + end);
    assertEquals(180.0, Math.abs(end.getRotation().getDegrees()), 5.0, "ended facing " + end.getRotation());
  }

  private static void step() throws InterruptedException {
    SimHooks.stepTiming(0.02);
    CommandScheduler.getInstance().run();
    SimulatedArena.getInstance().simulationPeriodic();
    Thread.sleep(2); // the pathfinder plans on its own thread
  }
}
