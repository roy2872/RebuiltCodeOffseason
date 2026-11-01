package frc.robot.subsystems.intakedeploy;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Drives the real IntakeDeploy subsystem against its physics sim. */
class IntakeDeploySimTest {
  private static final double TOLERANCE_ROTATIONS = 0.2;
  private static final double MAX_TRAVEL_SECONDS = 1.0;
  private static IntakeDeploy intake;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // Constants must load before any subsystem, see HoodSimTest
    assertEquals(Constants.Mode.SIM, Constants.currentMode);
    intake = IntakeDeploy.mInstance;
    run(0.2);
  }

  @Test
  void opensAndClosesQuickly() {
    goTo(IntakeDeploy.DEPLOYED, IntakeDeployConstants.INTAKE_DEPLOYED_ANGLE);
    goTo(IntakeDeploy.STOWED, IntakeDeployConstants.INTAKE_STOWED_ANGLE);
  }

  private static void goTo(Setpoint<Angle> setpoint, Angle expected) {
    intake.applySetpoint(setpoint);
    run(MAX_TRAVEL_SECONDS);
    assertEquals(expected.in(Rotations), intake.getPosition().in(Rotations), TOLERANCE_ROTATIONS);
  }

  private static void run(double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      SimHooks.stepTiming(0.02);
      intake.periodic();
    }
  }
}
