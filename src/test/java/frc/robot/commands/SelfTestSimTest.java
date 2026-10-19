package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.simulation.RobotFuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Arrays;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * A healthy simulated robot passes the self test. The swerve and fuel steps run separately: in sim
 * the swerve steps drive the robot (its wheels can't be off the ground) and would knock the fuel away.
 */
class SelfTestSimTest {
  private static RobotFuelSim fuelSim;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
    fuelSim =
        new RobotFuelSim(
            Drive.mInstance.driveSimulation::getSimulatedDriveTrainPose,
            Drive.mInstance.driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);
  }

  @Test
  void mechanismsAndSwervePass() {
    place();
    SelfTest selfTest = newSelfTest();
    selfTest.setWheelsOffGround(true);
    selfTest.setFuelPath(false);
    String[] results = run(selfTest, () -> {});
    assertPassed(results, "Hood Home", "Intake Deploy", "Feeder", "Shooter Spin Up", "Swerve Drive");
  }

  @Test
  void intakesOneFuelAndShootsItSlowly() {
    place();
    fuelSim.setFuelHeld(0);
    SelfTest selfTest = newSelfTest();
    selfTest.setWheelsOffGround(false);
    selfTest.setFuelPath(true);
    String[] results =
        run(
            selfTest,
            () -> {
              // Like a person: once asked, set one fuel at the intake and press Fuel Placed
              if (selfTest.isWaitingForFuel()) {
                fuelSim.placeFuelInFrontOfIntake();
                selfTest.setFuelPlaced(true);
              }
            });
    assertPassed(results, "Fuel Intake One", "Fuel Slow Spin Up", "Fuel Slow Shot");
    assertEquals(0, fuelSim.getFuelHeld(), "the fuel is still in the robot");
  }

  /** Open floor, away from the field's fuel. */
  private static void place() {
    Pose2d spot = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    Drive.mInstance.driveSimulation.setSimulationWorldPose(spot);
    RobotState.mInstance.resetPose(spot);
  }

  private static SelfTest newSelfTest() {
    // No cameras: the vision checks have nothing to fail on
    Vision vision = new Vision((pose, time, stdDevs) -> {});
    return new SelfTest(vision, Optional.of(fuelSim::getFuelHeld));
  }

  private static String[] run(SelfTest selfTest, Runnable eachLoop) {
    Command test = selfTest.command();
    CommandScheduler.getInstance().schedule(test);
    for (int i = 0; i < 3000 && test.isScheduled(); i++) { // up to 60 s
      eachLoop.run();
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      SimulatedArena.getInstance().simulationPeriodic();
      fuelSim.periodic();
    }
    String[] results = SmartDashboard.getStringArray("SelfTest/Results", new String[0]);
    assertTrue(!test.isScheduled(), "self test never finished: " + Arrays.toString(results));
    // The intake rollers' TalonSRX has no physics sim, so they never turn in simulation
    var failed =
        Arrays.stream(results)
            .filter(r -> r.contains("FAILED") && !r.startsWith("Intake Rollers"))
            .toList();
    assertTrue(failed.isEmpty(), "failed: " + failed + " in " + Arrays.toString(results));
    return results;
  }

  private static void assertPassed(String[] results, String... steps) {
    for (String step : steps) {
      assertTrue(
          Arrays.stream(results).anyMatch(r -> r.startsWith(step + ": PASSED")),
          step + " didn't pass: " + Arrays.toString(results));
    }
  }
}
