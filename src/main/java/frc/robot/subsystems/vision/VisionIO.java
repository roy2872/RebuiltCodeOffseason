package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    // Robot headings trusted enough for RobotState. Part of the inputs so replay sees them too.
    public YawObservation[] yawObservations = new YawObservation[0];
    // Robot motion is unpredictable (pushed, wheels slipping), from the crop controller
    public boolean disturbed = false;
  }

  // record visionIOData(
  //   boolean connected,
  //   TargetObservation latestTargetObservation,
  //   PoseObservation[] poseObservations,
  //   int[] tagIds
  // ){}

  public default void updateInputs(VisionIOInputs inputs) {}

  public default String getName() {
    return null;
  }

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public static record YawObservation(double timestamp, Rotation2d yaw) {}

  /** How a pose was solved, which decides how far it can be trusted. */
  public static enum ObservationType {
    /** Limelight full 3D solve: heading included, single tags can flip. */
    MEGATAG_1,
    /** Limelight solve using the robot's gyro heading: no heading, but stable with one tag. */
    MEGATAG_2,
    /** PhotonVision solve (multi-tag or single-tag strategies): heading included. */
    PHOTONVISION
  }

  /**
   * @param ambiguity worst single-tag ambiguity among the tags used (0 when not meaningful)
   * @param onEdgePercentage how close the outermost tag is to the image border: 0 center, 1 edge
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double minimumTagDistance,
      double measurementFOM,
      double onEdgePercentage,
      ObservationType type) {}
}