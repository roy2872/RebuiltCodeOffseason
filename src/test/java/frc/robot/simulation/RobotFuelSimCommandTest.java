package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.sim.FuelSim;
import frc.lib.util.TableLoader;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Runs the robot's real intake and shoot commands against the drivetrain and fuel sims. */
class RobotFuelSimCommandTest {
  private static RobotFuelSim sim;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // Constants must load before any subsystem, as on the robot
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
    Constants.RobotState.shooterTableData = TableLoader.loadFromCSV("hubSolutions.csv");
    Constants.RobotState.fetchingTableData = TableLoader.loadDoubleMapFromCSV("fetchingSolutions.csv");
    sim = new RobotFuelSim(
        Drive.mInstance.driveSimulation::getSimulatedDriveTrainPose,
        Drive.mInstance.driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);
  }

  @Test
  void intakeThenShoot() {
    // Drive into the neutral zone fuel while holding intake, starting it at the same time
    place(new Pose2d(6.0, 4.0, Rotation2d.kZero));
    int preloaded = sim.getFuelHeld();
    CommandScheduler.getInstance().schedule(
        SuperStructure.mInstance.intakeCommand(),
        Commands.run(() -> Drive.mInstance.runVelocity(new ChassisSpeeds(1.5, 0, 0)), Drive.mInstance));
    run(2.0);
    CommandScheduler.getInstance().cancelAll();
    int intaked = sim.getFuelHeld() - preloaded;
    assertTrue(intaked >= 20, "only picked up " + intaked + " fuel driving through the neutral zone");

    // Shoot at the blue hub from about 2.5 m, starting turned away so it has to aim
    place(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    FuelSim.Hub.BLUE_HUB.resetScore();
    RobotState.mInstance.setShootType(true);
    RobotState.mInstance.setManualShooting(false);
    int held = sim.getFuelHeld();
    CommandScheduler.getInstance().schedule(SuperStructure.mInstance.shoot());
    run(4.0);
    CommandScheduler.getInstance().cancelAll();
    int shot = held - sim.getFuelHeld();
    int scored = FuelSim.Hub.BLUE_HUB.getScore();
    assertTrue(shot >= 10, "shoot() only shot " + shot + " fuel in 4 s");
    assertTrue(scored > 0, "none of the " + shot + " shots scored");
  }

  @Test
  void fetchShootsTowardOwnAlliance() {
    // The robot was turned when it was last reset, like after a disable in sim
    Drive.mInstance.driveSimulation.setSimulationWorldPose(new Pose2d(8.0, 2.0, Rotation2d.fromDegrees(16)));
    run(0.1);
    place(new Pose2d(8.0, 2.0, Rotation2d.fromDegrees(90)));
    sim.reset();
    RobotState.mInstance.setShootType(false);
    int held = sim.getFuelHeld();
    CommandScheduler.getInstance().schedule(SuperStructure.mInstance.shoot());
    run(2.0);
    CommandScheduler.getInstance().cancelAll();

    // Facing 0 degrees puts the -x shooter toward the blue wall
    double heading = Drive.mInstance.driveSimulation.getSimulatedDriveTrainPose().getRotation().getDegrees();
    assertEquals(0.0, heading, 2.0, "fetch aimed the real robot wrong");
    assertEquals(held, held - sim.getFuelHeld(), "fetch did not shoot everything it held");
  }

  private static void place(Pose2d pose) {
    Drive.mInstance.driveSimulation.setSimulationWorldPose(pose);
    RobotState.mInstance.resetPose(pose);
    run(0.1);
  }

  private static void run(double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      RobotState.mInstance.periodic();
      SimulatedArena.getInstance().simulationPeriodic();
      sim.periodic();
    }
  }
}
