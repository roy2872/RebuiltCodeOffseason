package frc.lib.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulates a position-controlled, single-jointed rotary mechanism such as a hood, pivot, or arm.
 */
public class ServoMechanismSim extends MechanismSim {
  private final SingleJointedArmSim sim;

  /**
   * Creates a servo mechanism simulation from the provided constants.
   *
   * @param constants Constants that describe the simulated mechanism.
   */
  public ServoMechanismSim(ServoMechanismSimConstants constants) {
    super(constants.gearing);
    sim =
        new SingleJointedArmSim(
            constants.motor,
            constants.gearing,
            constants.momentOfInertia,
            constants.armLength,
            constants.minAngle.in(Units.Radians),
            constants.maxAngle.in(Units.Radians),
            constants.simulateGravity,
            constants.startingAngle.in(Units.Radians));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    sim.setInputVoltage(voltage.in(Units.Volts));
  }

  @Override
  public Angle getPosition() {
    return Units.Radians.of(sim.getAngleRads());
  }

  @Override
  public AngularVelocity getVelocity() {
    return Units.RadiansPerSecond.of(sim.getVelocityRadPerSec());
  }

  @Override
  public Current getStatorCurrent() {
    return Units.Amps.of(sim.getCurrentDrawAmps());
  }

  @Override
  protected void update(Time deltaTime) {
    sim.update(deltaTime.in(Units.Seconds));
  }

  @Override
  public void setState(Angle angle, AngularVelocity velocity) {
    sim.setState(angle.in(Units.Radians), velocity.in(Units.RadiansPerSecond));
  }

  /** Constants for creating a {@link ServoMechanismSim}. */
  public static class ServoMechanismSimConstants {
    /** Motor model, including the number of motors driving the mechanism. */
    public DCMotor motor;

    /** Motor rotations per mechanism rotation. */
    public double gearing;

    /** Moment of inertia of the mechanism in kg-m^2. */
    public double momentOfInertia;

    /** Distance from the pivot to the mechanism's center of mass in meters. */
    public double armLength;

    /** Minimum permitted mechanism angle. */
    public Angle minAngle;

    /** Maximum permitted mechanism angle. */
    public Angle maxAngle;

    /** Whether gravity should be included in the model. */
    public boolean simulateGravity;

    /** Initial mechanism angle. */
    public Angle startingAngle;
  }
}
