package frc.lib.visualization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import java.util.function.Supplier;

/**
 * Helpers that turn a {@link MotorSubsystem}'s position into the values a {@link VisualizedJoint}
 * needs, so robot code does not have to write the same lambdas for every subsystem.
 */
public final class MotorSubsystemJoints {
  private MotorSubsystemJoints() {}

  /**
   * Measured position of the subsystem, in the units its MotorIO reports.
   *
   * @param subsystem Subsystem to read.
   * @return Supplier of the current position.
   */
  public static Supplier<Angle> measured(MotorSubsystem<?, ?> subsystem) {
    return subsystem::getPosition;
  }

  /**
   * Commanded position of the subsystem. Falls back to the measured position when it is not in
   * position control (idle, voltage, velocity), so the ghost sits on the real mechanism.
   *
   * @param subsystem Subsystem to read.
   * @return Supplier of the commanded position.
   */
  public static Supplier<Angle> setpoint(MotorSubsystem<?, ?> subsystem) {
    return () -> {
      MotorIO<?> io = subsystem.getIO();
      return io.getSetpoint().mode.isPositionControl()
          ? io.unitType.of(io.getSetpointDoubleInUnits())
          : io.getPosition();
    };
  }

  /**
   * Converts a reported position to the joint angle shown in 3D.
   *
   * @param position Reported position (e.g. from {@link #measured}).
   * @param zero Reported position at which the part is in its modeled (CAD) pose.
   * @param scale Joint radians per reported radian. Use -1 to flip direction, or a ratio if the
   *     subsystem reports motor rotations.
   * @return Supplier of the joint angle.
   */
  public static Supplier<Angle> angle(Supplier<Angle> position, Angle zero, double scale) {
    return () -> position.get().minus(zero).times(scale);
  }

  /**
   * Converts a reported position to the joint extension shown in 3D.
   *
   * @param position Reported position (e.g. from {@link #measured}).
   * @param zero Reported position at which the part is in its modeled (CAD) pose.
   * @param distancePerRotation Distance traveled per reported rotation.
   * @return Supplier of the joint extension.
   */
  public static Supplier<Distance> distance(Supplier<Angle> position, Angle zero, Distance distancePerRotation) {
    return () -> Meters.of(position.get().minus(zero).in(Rotations) * distancePerRotation.in(Meters));
  }
}
