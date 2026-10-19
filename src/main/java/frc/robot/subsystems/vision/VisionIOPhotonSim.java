package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Simulates any camera (PhotonVision or Limelight) as a PhotonVision camera with the same optics and
 * mounting, then runs it through the real {@link VisionIOPhoton} estimation code.
 */
public class VisionIOPhotonSim extends VisionIOPhoton {

  /**
   * Imperfections of the simulated camera.
   *
   * @param fps frames per second the camera processes
   * @param avgLatencyMs average capture to result latency
   * @param latencyStdDevMs latency jitter
   * @param avgCornerErrorPx average tag corner detection error in pixels
   * @param cornerErrorStdDevPx corner detection error jitter in pixels
   * @param enableStreams publish raw / processed sim video streams (slow, debug only)
   */
  public record SimOptions(
      double fps,
      double avgLatencyMs,
      double latencyStdDevMs,
      double avgCornerErrorPx,
      double cornerErrorStdDevPx,
      boolean enableStreams) {
    public static final SimOptions DEFAULT = new SimOptions(40.0, 30.0, 5.0, 0.35, 0.1, false);
  }

  // One world shared by every simulated camera, stepped once per loop
  private static VisionSystemSim visionSim;
  private static double lastSimUpdateTime = Double.NaN;

  private final Supplier<Pose2d> simPoseSupplier;

  /**
   * @param config camera model and mounting, the same one the real camera uses
   * @param options PhotonVision-only settings
   * @param simOptions simulated camera imperfections
   * @param simPoseSupplier ground truth robot pose from the physics sim
   * @param headingSupplier robot heading as the code sees it (gyro / estimator), not ground truth
   */
  public VisionIOPhotonSim(
      CameraConfig config,
      Options options,
      SimOptions simOptions,
      Supplier<Pose2d> simPoseSupplier,
      Supplier<Rotation2d> headingSupplier) {
    super(config, options, headingSupplier);
    this.simPoseSupplier = simPoseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.aprilTagLayout);
    }

    var cameraSim =
        new PhotonCameraSim(
            camera, simProperties(config.model(), simOptions), VisionConstants.aprilTagLayout);
    cameraSim.enableRawStream(simOptions.enableStreams());
    cameraSim.enableProcessedStream(simOptions.enableStreams());
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  /** Pinhole intrinsics matching the model's horizontal and vertical FOV, no distortion. */
  static SimCameraProperties simProperties(CameraModel model, SimOptions simOptions) {
    double fx = model.widthPixels() / (2.0 * model.tanHalfHorizontalFov());
    double fy = model.heightPixels() / (2.0 * model.tanHalfVerticalFov());
    double cx = model.widthPixels() / 2.0;
    double cy = model.heightPixels() / 2.0;

    var properties = new SimCameraProperties();
    properties.setCalibration(
        model.widthPixels(),
        model.heightPixels(),
        MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0, cx, 0, fy, cy, 0, 0, 1),
        VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0));
    properties.setFPS(simOptions.fps());
    properties.setAvgLatencyMs(simOptions.avgLatencyMs());
    properties.setLatencyStdDevMs(simOptions.latencyStdDevMs());
    properties.setCalibError(simOptions.avgCornerErrorPx(), simOptions.cornerErrorStdDevPx());
    return properties;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    if (now != lastSimUpdateTime) {
      lastSimUpdateTime = now;
      visionSim.update(simPoseSupplier.get());
      Logger.recordOutput("Vision/Sim/GroundTruthPose", simPoseSupplier.get());
    }
    super.updateInputs(inputs);
  }
}
