package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // PhotonVision pipeline switching (VisionIOPhoton only)
  public static final short lowResPipeline = 1;
  public static final short highResPipeline = 0;

  public static final double lowResTriggerDistance = 3.0; // meters
  public static final double highResTriggerDistance = 3.5; // meters

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToBLcamera =
      new Transform3d(
          -0.285, 0.32, 0.15, new Rotation3d(0.0, -25 * Math.PI / 180, -60 * Math.PI / 180 + Math.PI));
  public static Transform3d robotToBRcamera =
      new Transform3d(-0.285, -0.32, 0.2, new Rotation3d(0.0, -25 * Math.PI / 180, 60 * Math.PI / 180 + Math.PI));

  // TODO: measure. From the note below: 0.266 m back, 0.47 m up, centered. Pitch assumed to be
  // 25 deg up (65 deg from vertical) and facing forward; must match the Limelight web UI.
  public static final Transform3d robotToTsachiCamera =
      new Transform3d(-0.266, 0.0, 0.47, new Rotation3d(0.0, Math.toRadians(-25.0), 0.0));

  public static Transform3d[] robotToCameras = {robotToBLcamera, robotToBRcamera, new Transform3d()};

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.4;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 99999; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.2, // Camera 0
        0.2 // Camera 1
      };
// limelight: 65 pitch, 0.47 m z, 0.266 m backwards in x, 0 y
      public static final double ON_EDGE_THRESHOLD = 0.92;
  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

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
}
