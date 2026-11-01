package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.sim.FuelSim;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Drives the real mechanism sims through an intake-then-shoot cycle. */
class RobotFuelSimMechanismTest {
  private static Pose2d pose = Pose2d.kZero;
  private static ChassisSpeeds speeds = new ChassisSpeeds();

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // Constants must load before any subsystem, as on the robot
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
  }

  @Test
  void intakesThenShoots() {
    RobotFuelSim sim = new RobotFuelSim(() -> pose, () -> speeds);
    int preloaded = sim.getFuelHeld();

    // Deploy short of the neutral zone fuel, then drive through it
    pose = new Pose2d(6.5, 4.0, Rotation2d.kZero);
    IntakeDeploy.mInstance.applySetpoint(IntakeDeploy.DEPLOYED);
    IntakeRollers.mInstance.applySetpoint(IntakeRollers.INTAKE);
    run(sim, 1.0);
    assertEquals(preloaded, sim.getFuelHeld(), "picked up fuel before reaching any");
    speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    for (int i = 0; i < 125; i++) {
      pose = new Pose2d(6.5 + 0.02 * i, 4.0, Rotation2d.kZero);
      run(sim, 0.02);
    }
    IntakeRollers.mInstance.applySetpoint(IntakeRollers.IDLE);
    speeds = new ChassisSpeeds();
    int intaked = sim.getFuelHeld() - preloaded;
    assertTrue(intaked >= 20, "only picked up " + intaked + " fuel driving through the neutral zone");

    // Spin up for the shooting table's 2.3 m shot, then feed
    Translation2d hub = FuelSim.Hub.BLUE_HUB.getCenter();
    pose = new Pose2d(
        hub.getX() - 2.3 + ShooterConstants.LAUNCH_POSITION_ON_ROBOT.getX(), hub.getY(), Rotation2d.k180deg);
    double flywheelRps = 13.0 / (Math.PI * ShooterConstants.FLYWHEEL_DIAMETER.in(Meters));
    Shooter.mInstance.applySetpoint(Setpoint.withVelocitySetpoint(RotationsPerSecond.of(flywheelRps)));
    Hood.mInstance.applySetpoint(Setpoint.withMotionMagicSetpoint(Degrees.of(90.0 - 77.5)));
    run(sim, 1.5);
    int held = sim.getFuelHeld();
    Feeder.mInstance.applySetpoint(Feeder.FEED_VOLTAGE);
    run(sim, 0.5);
    assertTrue(sim.getFuelHeld() < held, "feeding did not shoot");
    run(sim, 5.0);
    assertEquals(0, sim.getFuelHeld(), "feeding did not empty the robot");
    Feeder.mInstance.applySetpoint(Feeder.IDLE);
  }

  private static void run(RobotFuelSim sim, double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      SimHooks.stepTiming(0.02);
      IntakeDeploy.mInstance.periodic();
      IntakeRollers.mInstance.periodic();
      Hood.mInstance.periodic();
      Shooter.mInstance.periodic();
      Feeder.mInstance.periodic();
      sim.periodic();
    }
  }
}
