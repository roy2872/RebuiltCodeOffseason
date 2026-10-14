package frc.lib.visualization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * One moving (or fixed) part of the robot, described independently of any specific subsystem.
 *
 * <p>All geometry is given in the <b>robot frame with every joint at zero</b> (WPILib convention: +X
 * forward, +Y left, +Z up, origin on the floor at the robot center). This matches how a part is
 * positioned when it is exported in place from the full robot CAD assembly, so the same numbers work
 * for the 3D model and for the 2D side view.
 *
 * <p>A joint can be attached to a parent joint (e.g. a wrist on an arm), in which case it inherits
 * the parent's motion. Create joints with {@link #rotary}, {@link #linear} or {@link #fixed} and
 * register them with {@link RobotVisualizer#register}.
 */
public final class VisualizedJoint {
  /** Unit vector along the robot's X axis (roll). */
  public static final Translation3d X_AXIS = new Translation3d(1, 0, 0);
  /** Unit vector along the robot's Y axis (pitch). Positive rotation tips +X down toward the floor. */
  public static final Translation3d Y_AXIS = new Translation3d(0, 1, 0);
  /** Unit vector along the robot's Z axis (yaw / turret). */
  public static final Translation3d Z_AXIS = new Translation3d(0, 0, 1);

  private static final Color8Bit DEFAULT_COLOR = new Color8Bit(255, 170, 0);

  enum Type {
    FIXED,
    ROTARY,
    LINEAR
  }

  final String name;
  final VisualizedJoint parent;
  final Type type;
  final Translation3d origin;
  final Translation3d axis;
  final DoubleSupplier measured;
  DoubleSupplier setpoint = null;

  Translation3d lineStart = null;
  Translation3d lineEnd = null;
  Color8Bit lineColor = DEFAULT_COLOR;

  private VisualizedJoint(
      String name,
      VisualizedJoint parent,
      Type type,
      Translation3d origin,
      Translation3d axis,
      DoubleSupplier measured) {
    this.name = name;
    this.parent = parent;
    this.type = type;
    this.origin = origin;
    this.axis = axis.getNorm() > 0 ? axis.div(axis.getNorm()) : axis;
    this.measured = measured;
  }

  /**
   * A part that rotates about a fixed axis (pivot, hood, wrist, turret, flywheel, roller).
   *
   * @param name Name used for telemetry and the 2D view.
   * @param parent Joint this part is mounted on, or null if mounted on the drivetrain.
   * @param pivot Point the part rotates about, in the zeroed robot frame.
   * @param axis Rotation axis (right-hand rule), e.g. {@link #Y_AXIS} or {@code Y_AXIS.unaryMinus()}.
   * @param angle Current angle of the part, where zero is the pose it was modeled in.
   * @return The new joint.
   */
  public static VisualizedJoint rotary(
      String name, VisualizedJoint parent, Translation3d pivot, Translation3d axis, Supplier<Angle> angle) {
    return new VisualizedJoint(name, parent, Type.ROTARY, pivot, axis, () -> angle.get().baseUnitMagnitude());
  }

  /**
   * A part that slides along a fixed direction (elevator stage, telescope, linear slide).
   *
   * @param name Name used for telemetry and the 2D view.
   * @param parent Joint this part is mounted on, or null if mounted on the drivetrain.
   * @param base Bottom of travel, in the zeroed robot frame. Used as the default 2D line start.
   * @param direction Direction of travel, e.g. {@link #Z_AXIS} for a vertical elevator.
   * @param extension Current distance traveled from the modeled pose.
   * @return The new joint.
   */
  public static VisualizedJoint linear(
      String name, VisualizedJoint parent, Translation3d base, Translation3d direction, Supplier<Distance> extension) {
    return new VisualizedJoint(
        name, parent, Type.LINEAR, base, direction, () -> extension.get().baseUnitMagnitude());
  }

  /**
   * A part with no motion of its own that should still be shown as a separate model, e.g. a bracket
   * that rides on a parent joint.
   *
   * @param name Name used for telemetry.
   * @param parent Joint this part is mounted on, or null if mounted on the drivetrain.
   * @return The new joint.
   */
  public static VisualizedJoint fixed(String name, VisualizedJoint parent) {
    return new VisualizedJoint(name, parent, Type.FIXED, Translation3d.kZero, Translation3d.kZero, () -> 0.0);
  }

  /**
   * Also shows where the part is being commanded to, as a "ghost". Use the same units as the measured
   * supplier ({@link Angle} for rotary, {@link Distance} for linear). Joints without a setpoint follow
   * their measured value in the ghost.
   *
   * @param setpoint Commanded position of the part.
   * @return This joint, for chaining.
   */
  public VisualizedJoint withSetpoint(Supplier<? extends Measure<?>> setpoint) {
    this.setpoint = () -> setpoint.get().baseUnitMagnitude();
    return this;
  }

  /**
   * Draws this part as a line in the 2D side view (no CAD needed). Both points are in the zeroed robot
   * frame and move with the part.
   *
   * @param start One end of the line, usually the pivot or base.
   * @param end The other end, e.g. the tip of an arm at zero.
   * @param color Line color.
   * @return This joint, for chaining.
   */
  public VisualizedJoint withSideViewLine(Translation3d start, Translation3d end, Color8Bit color) {
    this.lineStart = start;
    this.lineEnd = end;
    this.lineColor = color;
    return this;
  }

  /**
   * Draws this part as a line in the 2D side view, starting at its pivot (rotary) or base (linear).
   *
   * @param end Tip of the part at zero, in the zeroed robot frame.
   * @param color Line color.
   * @return This joint, for chaining.
   */
  public VisualizedJoint withSideViewLine(Translation3d end, Color8Bit color) {
    return withSideViewLine(origin, end, color);
  }

  /** The joint's name. */
  public String getName() {
    return name;
  }

  /**
   * Motion of this joint relative to its parent, for a value in base units (radians or meters).
   * Rotary joints rotate about {@link #origin} so that models exported in place stay aligned.
   */
  Transform3d localTransform(double value) {
    switch (type) {
      case ROTARY:
        Rotation3d rotation = new Rotation3d(VecBuilder.fill(axis.getX(), axis.getY(), axis.getZ()), value);
        return new Transform3d(origin.minus(origin.rotateBy(rotation)), rotation);
      case LINEAR:
        return new Transform3d(axis.times(value), Rotation3d.kZero);
      default:
        return Transform3d.kZero;
    }
  }
}
