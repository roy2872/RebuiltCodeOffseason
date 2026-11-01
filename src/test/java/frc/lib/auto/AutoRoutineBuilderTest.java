package frc.lib.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The framework against a fake drivetrain that records what it was asked to do. */
class AutoRoutineBuilderTest {
  private final List<String> log = new ArrayList<>();
  private final FakeDrive drive = new FakeDrive();

  private class FakeDrive implements DriveAdapter {
    @Override
    public Command driveTo(Pose2d target) {
      return Commands.runOnce(() -> log.add("drive " + target.getX()));
    }

    @Override
    public Command followPath(String pathName) {
      return Commands.runOnce(() -> log.add("path " + pathName));
    }

    @Override
    public List<Pose2d> pathPoses(String pathName) {
      return pathName.equals("Known")
          ? List.of(new Pose2d(1, 0, Rotation2d.kZero), new Pose2d(5, 0, Rotation2d.kZero))
          : List.of();
    }

    @Override
    public void resetPose(Pose2d pose) {
      log.add("reset " + pose.getX());
    }

    @Override
    public Command stop() {
      return Commands.runOnce(() -> log.add("stop"));
    }

    @Override
    public double estimateDriveSeconds(Pose2d from, Pose2d to) {
      return from.getTranslation().getDistance(to.getTranslation()); // 1 m/s
    }
  }

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @BeforeEach
  void clear() {
    CommandScheduler.getInstance().cancelAll();
    log.clear();
  }

  private AutoAction action(String name, double x, double seconds) {
    return AutoAction.named(name)
        .driveTo(new Pose2d(x, 0, Rotation2d.kZero))
        .atTarget(() -> Commands.runOnce(() -> log.add("do " + name)), seconds)
        .build();
  }

  private void run(Command command) {
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < 50 && command.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
    }
  }

  @Test
  void runsThePartsInOrderAfterResettingThePose() {
    var start = new AutoStartPosition("Start", new Pose2d(1, 0, Rotation2d.kZero));
    var routine =
        AutoRoutineBuilder.build(
            start,
            List.of(action("A", 3, 1.0), action("B", 6, 2.0)),
            drive,
            pose -> pose,
            () -> Commands.runOnce(() -> log.add("end")));
    run(routine.command());
    assertEquals(List.of("reset 1.0", "drive 3.0", "do A", "drive 6.0", "do B", "end"), log);
    // 2 m + 1 s + 3 m + 2 s
    assertEquals(8.0, routine.estimatedSeconds(), 1e-9);
    assertEquals(3, routine.previewPoses().size());
    assertTrue(routine.warnings().isEmpty());
  }

  @Test
  void flipsPosesForTheCurrentAlliance() {
    var start = new AutoStartPosition("Start", new Pose2d(1, 0, Rotation2d.kZero));
    var routine =
        AutoRoutineBuilder.build(
            start,
            List.of(action("A", 3, 0.0)),
            drive,
            pose -> new Pose2d(10 - pose.getX(), pose.getY(), pose.getRotation()),
            Commands::none);
    run(routine.command());
    assertEquals(List.of("reset 9.0", "drive 7.0", "do A"), log);
  }

  @Test
  void aTimedOutPartDoesNotStopTheRoutine() {
    var start = new AutoStartPosition("Start", new Pose2d(0, 0, Rotation2d.kZero));
    var stuck = AutoAction.named("Stuck").atTarget(Commands::idle, 0.0).timeout(0.1).build();
    var routine =
        AutoRoutineBuilder.build(
            start, List.of(stuck, action("After", 1, 0.0)), drive, pose -> pose, Commands::none);
    CommandScheduler.getInstance().schedule(routine.command());
    for (int i = 0; i < 20; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
    }
    assertTrue(log.contains("do After"), "the part after a timeout never ran: " + log);
  }

  @Test
  void warnsAboutRoutinesLongerThanAuto() {
    var start = new AutoStartPosition("Start", new Pose2d(0, 0, Rotation2d.kZero));
    var routine =
        AutoRoutineBuilder.build(
            start, List.of(action("Far", 15, 6.0)), drive, pose -> pose, Commands::none);
    assertFalse(routine.warnings().isEmpty());
  }

  @Test
  void warnsAboutMissingPaths() {
    var start = new AutoStartPosition("Start", new Pose2d(0, 0, Rotation2d.kZero));
    var routine =
        AutoRoutineBuilder.build(
            start,
            List.of(
                AutoAction.named("Known").followPath("Known").build(),
                AutoAction.named("Missing").followPath("Missing").build()),
            drive,
            pose -> pose,
            Commands::none);
    assertEquals(1, routine.warnings().size());
    // Ends at the end of the known path
    assertEquals(5.0, routine.previewPoses().get(1).getX(), 1e-9);
  }
}
