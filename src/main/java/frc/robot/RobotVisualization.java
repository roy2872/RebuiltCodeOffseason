package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.visualization.MotorSubsystemJoints;
import frc.lib.visualization.RobotVisualizer;
import frc.lib.visualization.VisualizedJoint;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;

/**
 * Lists this robot's moving parts for {@link RobotVisualizer}. This is the only robot-specific
 * visualization file: on a new robot, replace the joints below and the geometry constants they use.
 *
 * <p>The registration order is the component index in AdvantageScope, so it must match the
 * model_INDEX.glb files and the "components" array in ascope_assets/Robot_Rebuilt/config.json.
 */
public final class RobotVisualization {
  /**
   * Publishes live-editable offsets and pivots for every part (see RobotVisualizer#enableCalibration).
   * Set to false once the calibrated numbers are copied into the constants.
   */
  private static final boolean CALIBRATION = false;

  // Hood position at which the model is drawn in its CAD pose. Lowering it turns the model down.
  private static final Angle HOOD_MODEL_ZERO =
      HoodConstants.HOOD_STARTING_ANGLE.minus(HoodConstants.VISUALIZATION_ZERO_OFFSET);

  private RobotVisualization() {}

  /** Registers all joints. Call once at startup, after the subsystems exist. */
  public static void configure() {
    // The 2D side view is only for sim/replay, like 6328 does, to keep real robot logs lean
    boolean drawSideView = Constants.currentMode != Constants.Mode.REAL;
    RobotVisualizer visualizer = RobotVisualizer.mInstance;
    if (CALIBRATION) {
      visualizer.enableCalibration();
    }

    // Component 0 (model_0.glb)
    VisualizedJoint intake = VisualizedJoint.linear(
            "Intake Deploy",
            null,
            IntakeDeployConstants.VISUALIZATION_BASE,
            IntakeDeployConstants.VISUALIZATION_DIRECTION,
            MotorSubsystemJoints.distance(
                MotorSubsystemJoints.measured(IntakeDeploy.mInstance),
                IntakeDeployConstants.INTAKE_STOWED_ANGLE,
                IntakeDeployConstants.VISUALIZATION_EXTENSION_PER_ROTATION))
        .withSetpoint(MotorSubsystemJoints.distance(
            MotorSubsystemJoints.setpoint(IntakeDeploy.mInstance),
            IntakeDeployConstants.INTAKE_STOWED_ANGLE,
            IntakeDeployConstants.VISUALIZATION_EXTENSION_PER_ROTATION));
    if (drawSideView) {
      intake.withSideViewLine(
          IntakeDeployConstants.VISUALIZATION_BASE,
          IntakeDeployConstants.VISUALIZATION_TIP,
          new Color8Bit(0, 170, 255));
    }
    visualizer.register(intake);

    // Component 1 (model_1.glb)
    VisualizedJoint hood = VisualizedJoint.rotary(
            "Hood",
            null,
            HoodConstants.VISUALIZATION_PIVOT,
            HoodConstants.VISUALIZATION_AXIS,
            MotorSubsystemJoints.angle(
                MotorSubsystemJoints.measured(Hood.mInstance), HOOD_MODEL_ZERO, 1.0))
        .withSetpoint(MotorSubsystemJoints.angle(
            MotorSubsystemJoints.setpoint(Hood.mInstance), HOOD_MODEL_ZERO, 1.0));
    if (drawSideView) {
      hood.withSideViewLine(HoodConstants.VISUALIZATION_TIP, new Color8Bit(255, 170, 0));
    }
    visualizer.register(hood);
  }
}
