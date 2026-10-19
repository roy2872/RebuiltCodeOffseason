package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Robot to camera transforms. Limelights must also have this mounting in their web UI; the code
  // uses it for the crop controller and to place the simulated camera.
  public static Transform3d robotToBLcamera =
      new Transform3d(
          -0.285, 0.32, 0.15, new Rotation3d(0.0, -25 * Math.PI / 180, -60 * Math.PI / 180 + Math.PI));
  public static Transform3d robotToBRcamera =
      new Transform3d(-0.285, -0.32, 0.2, new Rotation3d(0.0, -25 * Math.PI / 180, 60 * Math.PI / 180 + Math.PI));

  // TODO: measure. From the note below: 0.266 m back, 0.47 m up, centered. Pitch assumed to be
  // 25 deg up (65 deg from vertical) and facing forward; must match the Limelight web UI.
  public static final Transform3d robotToTsachiCamera =
      new Transform3d(-0.266, 0.0, 0.47, new Rotation3d(0.0, Math.toRadians(-25.0), Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.4;
  public static double maxZError = 0.75;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };
// limelight: 65 pitch, 0.47 m z, 0.266 m backwards in x, 0 y
      public static final double ON_EDGE_THRESHOLD = 0.92;

  /**
   * Limelights: use MegaTag2 (gyro heading + tags) for translation while enabled, and MegaTag1 only
   * while disabled, when the heading may not be set yet. False uses MegaTag1 all the time.
   */
  public static final boolean useMegatag2 = true;

  /**
   * Let multi-tag solves correct the heading (MegaTag1 only while disabled). Off until the camera
   * mounting transform is measured: a wrong mount yaw would turn straight into a heading error. Check
   * it against the logged Roi/LimelightCameraPoseRobotSpace first.
   */
  public static final boolean useVisionHeading = false;

  /** Tags never used for localization (badly mounted, often blocked). Filtered on the Limelight. */
  public static final int[] ignoredTagIds = new int[] {};

  /** Every field tag except {@link #ignoredTagIds}, for the Limelight ID filter. */
  public static int[] allowedTagIds() {
    return java.util.stream.IntStream.rangeClosed(1, aprilTagLayout.getTags().size())
        .filter(id -> java.util.Arrays.stream(ignoredTagIds).noneMatch(ignored -> ignored == id))
        .toArray();
  }

  // Crop / downscale tuning. Robot-specific: acceleration limits should match what the
  // drivetrain can do; anything beyond that (getting hit) is caught by the disturbance logic.
  public static final VisionRoiController.Tuning roiTuning =
      new VisionRoiController.Tuning(
          0.1651, // tagSizeMeters (6.5 in)
          0.5, // tagPadFraction
          0.12, // minCropHalfSize
          0.6, // maxCropAreaFraction
          0.03, // commandLatencySec
          10.0, // maxLinearAccel m/s^2
          20.0, // maxAngularAccel rad/s^2
          0.3, // linearVelocityNoise m/s
          0.1, // angularVelocityNoise rad/s
          Math.toRadians(2.0), // tiltNoiseRad
          0.35, // missGrowthPerFrame
          0.3, // trackTimeoutSec
          1.0, // refreshPeriodSec
          20.0, // minTagPixels
          1.3, // downscaleHysteresis
          1.0, // searchDownscale
          4.0, // maxAdaptiveScale
          0.97, // adaptiveScaleDecay
          4.0, // disturbanceInnovationRatio
          1.5, // wheelSlipOmegaThreshold rad/s
          0.5); // disturbanceHoldSec

  public static final CameraConfig tsachiCamera =
      new CameraConfig(
          "limelight-tsachi", CameraModel.LIMELIGHT_3G, robotToTsachiCamera, roiTuning);

  // OV2311 PhotonVision cameras, not currently mounted. Pass to VisionIOPhoton /
  // VisionIOPhotonSim to use.
  public static final CameraConfig blCamera =
      new CameraConfig("BLcamera", CameraModel.OV2311, robotToBLcamera, roiTuning);
  public static final CameraConfig brCamera =
      new CameraConfig("BRcamera", CameraModel.OV2311, robotToBRcamera, roiTuning);
}
