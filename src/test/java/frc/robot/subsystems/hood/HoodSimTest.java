package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Drives the real Hood subsystem against its physics sim. */
class HoodSimTest {
  private static final double TOLERANCE_DEGREES = 1.0;
  private static Hood hood;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // Constants must load before any subsystem, as on the robot, or the static init cycle with
    // Ports leaves port IDs null
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
    hood = Hood.mInstance;
    run(0.2);
  }

  @Test
  void movesBetweenSetpoints() {
    assertEquals(HoodConstants.HOOD_STARTING_ANGLE.in(Degrees), hood.getPosition().in(Degrees), TOLERANCE_DEGREES);

    goTo(Hood.MAX_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    // Mid-range, so the controller (not the sim's hard stop) has to hold it
    Angle middle = Degrees.of(20.0);
    goTo(Setpoint.withMotionMagicSetpoint(middle), middle);
    goTo(Hood.STOWED, HoodConstants.HOOD_MIN_ANGLE);
  }

  /** shoot() aims with motion-profiled setpoints that change every loop. */
  @Test
  void followsChangingAim() {
    hood.followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(Degrees.of(25.0))).schedule();
    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      hood.periodic();
    }
    assertEquals(25.0, hood.getPosition().in(Degrees), TOLERANCE_DEGREES);
    CommandScheduler.getInstance().cancelAll();
    goTo(Hood.STOWED, HoodConstants.HOOD_MIN_ANGLE);
  }

  /** Plain position setpoints use a different Spark slot (2) than motion profiles (0). */
  @Test
  void followsPositionSetpoints() {
    Angle aim = Degrees.of(25.0);
    goTo(Setpoint.withPositionSetpoint(aim), aim);
    goTo(Hood.STOWED, HoodConstants.HOOD_MIN_ANGLE);
  }

  private static void goTo(Setpoint<Angle> setpoint, Angle expected) {
    hood.applySetpoint(setpoint);
    run(1.0);
    assertEquals(expected.in(Degrees), hood.getPosition().in(Degrees), TOLERANCE_DEGREES);
  }

  private static void run(double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      SimHooks.stepTiming(0.02);
      hood.periodic();
    }
  }
}
