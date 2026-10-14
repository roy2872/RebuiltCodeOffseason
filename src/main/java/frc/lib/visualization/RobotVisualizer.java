package frc.lib.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Logs robot mechanism poses for AdvantageScope, independent of which subsystems a robot has.
 *
 * <p>Outputs (under {@code <key>/}):
 *
 * <ul>
 *   <li>{@code Components/Measured} - Pose3d[] of every registered joint, robot-relative, in
 *       registration order. Drag it onto the robot in the 3D Field tab as "Component" to animate
 *       a custom robot model (index i is drawn with {@code model_i.glb}).
 *   <li>{@code Components/Setpoint} - same, but driven by setpoints. Use on a "Ghost" robot.
 *   <li>{@code Components/Names} - names in index order, to help keep config.json in sync.
 *   <li>{@code SideView} - a Mechanism2d drawn from joints with a side view line. Works with no CAD.
 * </ul>
 *
 * <p>Everything is derived from data that subsystems already log, so if logging is turned off during
 * matches the poses can still be regenerated afterwards with AdvantageKit replay.
 */
public class RobotVisualizer {
  public static final RobotVisualizer mInstance = new RobotVisualizer("RobotVisualizer");

  private static final double LINE_WEIGHT = 6.0;
  private static final Color8Bit SETPOINT_COLOR = new Color8Bit(120, 120, 120);

  private final String key;
  private final List<VisualizedJoint> joints = new ArrayList<>();
  private final Map<VisualizedJoint, Integer> indices = new HashMap<>();
  private final Map<VisualizedJoint, LoggedMechanismLigament2d[]> lines = new HashMap<>();

  private double sideViewWidth = 2.0;
  private double sideViewHeight = 2.0;
  private LoggedMechanism2d sideView = null;

  private BooleanSupplier enabled = () -> true;
  private int period = 1;
  private int cycle = 0;
  private boolean loggedNames = false;

  /**
   * Creates a visualizer. Most robots only need {@link #mInstance}.
   *
   * @param key Log key prefix for all outputs.
   */
  public RobotVisualizer(String key) {
    this.key = key;
  }

  /**
   * Adds a joint. Its index in the component arrays is the order it is registered in, and a joint's
   * parent must be registered before it.
   *
   * @param joint Joint to add.
   * @return The same joint, so it can be used as a parent.
   */
  public VisualizedJoint register(VisualizedJoint joint) {
    if (joint.parent != null && !indices.containsKey(joint.parent)) {
      throw new IllegalArgumentException(
          "Register parent '" + joint.parent.name + "' before child '" + joint.name + "'");
    }
    indices.put(joint, joints.size());
    joints.add(joint);
    if (joint.lineStart != null) {
      addSideViewLine(joint);
    }
    loggedNames = false;
    return joint;
  }

  /**
   * Sets the size of the 2D side view canvas. The robot center is placed at the horizontal middle.
   * Call before registering joints.
   *
   * @param widthMeters Canvas width.
   * @param heightMeters Canvas height.
   */
  public void configureSideView(double widthMeters, double heightMeters) {
    sideViewWidth = widthMeters;
    sideViewHeight = heightMeters;
  }

  /**
   * Only computes and logs while this returns true. Useful to cut logging at competition, see
   * {@link #setPeriod} for a lighter option.
   *
   * @param enabled Condition for logging.
   */
  public void setEnabled(BooleanSupplier enabled) {
    this.enabled = enabled;
  }

  /**
   * Logs once every {@code cycles} loops instead of every loop. AdvantageKit only sends values that
   * changed, so skipped loops cost no bandwidth.
   *
   * @param cycles Number of loops between updates (1 = every loop).
   */
  public void setPeriod(int cycles) {
    period = Math.max(1, cycles);
  }

  /** The log key prefix of all outputs, e.g. to filter them from NetworkTables. */
  public String getKey() {
    return key;
  }

  /** Computes and logs all poses. Call once per loop, after subsystems have updated their inputs. */
  public void periodic() {
    if (joints.isEmpty() || !enabled.getAsBoolean() || (cycle++ % period) != 0) {
      return;
    }

    if (!loggedNames) {
      Logger.recordOutput(key + "/Components/Names", joints.stream().map(j -> j.name).toArray(String[]::new));
      loggedNames = true;
    }

    Pose3d[] measured = computePoses(false);
    Logger.recordOutput(key + "/Components/Measured", measured);

    boolean hasSetpoints = joints.stream().anyMatch(j -> j.setpoint != null);
    Pose3d[] setpoint = hasSetpoints ? computePoses(true) : null;
    if (hasSetpoints) {
      Logger.recordOutput(key + "/Components/Setpoint", setpoint);
    }

    if (sideView != null) {
      updateSideView(measured, setpoint);
      Logger.recordOutput(key + "/SideView", sideView);
    }
  }

  /** Robot-relative pose of each joint. Parents are always earlier in the list than children. */
  private Pose3d[] computePoses(boolean useSetpoints) {
    Pose3d[] poses = new Pose3d[joints.size()];
    for (int i = 0; i < poses.length; i++) {
      VisualizedJoint joint = joints.get(i);
      double value = (useSetpoints && joint.setpoint != null)
          ? joint.setpoint.getAsDouble()
          : joint.measured.getAsDouble();
      Pose3d parentPose = joint.parent == null ? Pose3d.kZero : poses[indices.get(joint.parent)];
      poses[i] = parentPose.transformBy(joint.localTransform(value));
    }
    return poses;
  }

  private void addSideViewLine(VisualizedJoint joint) {
    if (sideView == null) {
      sideView = new LoggedMechanism2d(sideViewWidth, sideViewHeight);
    }
    LoggedMechanismLigament2d measuredLine = sideView
        .getRoot(joint.name, 0, 0)
        .append(new LoggedMechanismLigament2d(joint.name, 0, 0, LINE_WEIGHT, joint.lineColor));
    LoggedMechanismLigament2d setpointLine = joint.setpoint == null ? null : sideView
        .getRoot(joint.name + " Setpoint", 0, 0)
        .append(new LoggedMechanismLigament2d(joint.name + " Setpoint", 0, 0, LINE_WEIGHT / 2, SETPOINT_COLOR));
    lines.put(joint, new LoggedMechanismLigament2d[] {measuredLine, setpointLine});
  }

  private void updateSideView(Pose3d[] measured, Pose3d[] setpoint) {
    for (Map.Entry<VisualizedJoint, LoggedMechanismLigament2d[]> entry : lines.entrySet()) {
      VisualizedJoint joint = entry.getKey();
      int index = indices.get(joint);
      LoggedMechanismLigament2d[] ligaments = entry.getValue();
      drawLine(joint.name, joint, measured[index], ligaments[0]);
      if (ligaments[1] != null) {
        drawLine(joint.name + " Setpoint", joint, setpoint[index], ligaments[1]);
      }
    }
  }

  /** Projects the joint's line onto the robot's X-Z plane (viewed from the robot's left side). */
  private void drawLine(String rootName, VisualizedJoint joint, Pose3d pose, LoggedMechanismLigament2d ligament) {
    Translation3d start = applyPose(pose, joint.lineStart);
    Translation3d end = applyPose(pose, joint.lineEnd);
    double dx = end.getX() - start.getX();
    double dz = end.getZ() - start.getZ();
    LoggedMechanismRoot2d root = sideView.getRoot(rootName, 0, 0);
    root.setPosition(start.getX() + sideViewWidth / 2.0, start.getZ());
    ligament.setLength(Math.hypot(dx, dz));
    ligament.setAngle(Math.toDegrees(Math.atan2(dz, dx)));
  }

  private static Translation3d applyPose(Pose3d pose, Translation3d point) {
    return pose.getTranslation().plus(point.rotateBy(pose.getRotation()));
  }
}
