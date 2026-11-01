package frc.robot.subsystems.vision;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * PhotonVision camera used for global localization.
 *
 * <p>Per frame, in order of preference:
 *
 * <ol>
 *   <li>MultiTag PnP solved on the coprocessor (needs "MultiTag" enabled and the field layout
 *       uploaded in the PhotonVision UI). Uses every tag corner at once, so it has no ambiguity.
 *   <li>Constrained SolvePnP on the roboRIO: fits x and y to all four corners of every visible tag,
 *       with the robot flat on the floor and its heading held to the gyro. Seeded with the trig
 *       solve below. Needs the camera calibration, which PhotonVision publishes. In sim it was 3-5x
 *       more accurate than the trig solve alone (about 1.5 cm vs 5-8 cm at 6 m).
 *   <li>"PnP distance trig solve": the best tag's distance plus its yaw / pitch in the image,
 *       combined with the gyro heading. Used when there is no calibration or the solver fails.
 *   <li>Lowest-ambiguity single-tag PnP, only when no heading is available yet.
 * </ol>
 *
 * <p>The single-tag methods never use the tag's solved orientation, which is the part that flips on
 * ambiguous single-tag solves. Filtering and standard deviations are left to {@link Vision}, so
 * PhotonVision and Limelight cameras are trusted the same way.
 */
public class VisionIOPhoton implements VisionIO {

  public enum SingleTagMethod {
    CONSTRAINED_SOLVEPNP,
    TRIG_SOLVE
  }

  /**
   * PhotonVision-only settings.
   *
   * @param pipelineIndex AprilTag pipeline to run
   * @param singleTagMethod how to localize when MultiTag has no result
   */
  public record Options(int pipelineIndex, SingleTagMethod singleTagMethod) {
    public static final Options DEFAULT = new Options(0, SingleTagMethod.CONSTRAINED_SOLVEPNP);
  }

  // Same limit the Limelight IO applies before feeding RobotState its yaw observations
  private static final double MAX_YAW_OBSERVATION_DISTANCE = 6.0;
  // A constrained solve this far from its trig-solve seed has found a wrong minimum
  private static final double MAX_CONSTRAINED_SEED_DISTANCE = 0.5;
  // Heading is held to the gyro (headingFree = false), and in sim this weight then made no
  // difference from 1 to 1e5. Letting heading vary was 2-10x less accurate with a good gyro.
  private static final double CONSTRAINED_HEADING_SCALE_FACTOR = 1.0;

  protected final PhotonCamera camera;
  private final CameraConfig config;
  private final Options options;
  private final Supplier<Rotation2d> headingSupplier;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * @param config camera model, mounting and tuning (see {@link VisionConstants})
   * @param options PhotonVision-only settings
   * @param headingSupplier robot heading, used by the single-tag solves
   */
  public VisionIOPhoton(CameraConfig config, Options options, Supplier<Rotation2d> headingSupplier) {
    this.config = config;
    this.options = options;
    this.headingSupplier = headingSupplier;
    camera = new PhotonCamera(config.name());
    camera.setPipelineIndex(options.pipelineIndex());
    poseEstimator =
        new PhotonPoseEstimator(VisionConstants.aprilTagLayout, config.robotToCamera());

    if (options.singleTagMethod() == SingleTagMethod.CONSTRAINED_SOLVEPNP) {
      // Otherwise it loads on the first single-tag frame and stalls that loop
      OpenCvLoader.forceStaticLoad();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Interpolated against each frame's capture timestamp, so it must be fed every loop
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), headingSupplier.get());

    Set<Integer> tagIds = new TreeSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    List<YawObservation> yawObservations = new ArrayList<>();
    PhotonPipelineResult latestResult = null;

    for (var result : camera.getAllUnreadResults()) {
      latestResult = result;
      if (!result.hasTargets()) {
        continue;
      }

      var estimate = estimatePose(result);
      if (estimate.isEmpty()) {
        continue;
      }

      var observation = toObservation(result, estimate.get());
      poseObservations.add(observation);
      for (var target : estimate.get().targetsUsed) {
        tagIds.add(target.getFiducialId());
      }

      if (estimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
          && observation.averageTagDistance() <= MAX_YAW_OBSERVATION_DISTANCE) {
        yawObservations.add(
            new YawObservation(
                observation.timestamp(), observation.pose().getRotation().toRotation2d()));
      }
    }

    if (!inputs.connected) {
      inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    } else if (latestResult != null) {
      inputs.latestTargetObservation =
          latestResult.hasTargets()
              ? new TargetObservation(
                  Rotation2d.fromDegrees(latestResult.getBestTarget().getYaw()),
                  Rotation2d.fromDegrees(latestResult.getBestTarget().getPitch()))
              : new TargetObservation(new Rotation2d(), new Rotation2d());
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.yawObservations = yawObservations.toArray(new YawObservation[0]);
  }

  private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
    var multiTag = poseEstimator.estimateCoprocMultiTagPose(result);
    if (multiTag.isPresent()) {
      return multiTag;
    }

    var trig = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
    if (trig.isEmpty()) {
      return poseEstimator.estimateLowestAmbiguityPose(result);
    }
    if (options.singleTagMethod() == SingleTagMethod.TRIG_SOLVE) {
      return trig;
    }

    var cameraMatrix = camera.getCameraMatrix();
    var distCoeffs = camera.getDistCoeffs();
    if (cameraMatrix.isEmpty() || distCoeffs.isEmpty()) {
      return trig;
    }
    var constrained =
        poseEstimator.estimateConstrainedSolvepnpPose(
            result,
            cameraMatrix.get(),
            distCoeffs.get(),
            trig.get().estimatedPose,
            false,
            CONSTRAINED_HEADING_SCALE_FACTOR);
    if (constrained.isEmpty()
        || constrained.get().estimatedPose.getTranslation()
                .getDistance(trig.get().estimatedPose.getTranslation())
            > MAX_CONSTRAINED_SEED_DISTANCE) {
      return trig;
    }
    return constrained;
  }

  private PoseObservation toObservation(PhotonPipelineResult result, EstimatedRobotPose estimate) {
    // Only these solves use the corners of every visible tag; the others use the best target
    List<PhotonTrackedTarget> used =
        switch (estimate.strategy) {
          case MULTI_TAG_PNP_ON_COPROCESSOR, CONSTRAINED_SOLVEPNP -> estimate.targetsUsed;
          default -> List.of(result.getBestTarget());
        };

    // Ambiguity only matters for plain single-tag PnP, whose orientation can flip
    double ambiguity =
        switch (estimate.strategy) {
          case MULTI_TAG_PNP_ON_COPROCESSOR ->
              result.getMultiTagResult().get().estimatedPose.ambiguity;
          case CONSTRAINED_SOLVEPNP, PNP_DISTANCE_TRIG_SOLVE -> 0.0;
          default -> result.getBestTarget().getPoseAmbiguity();
        };

    double totalDistance = 0.0;
    double minDistance = Double.POSITIVE_INFINITY;
    double onEdge = 0.0;
    for (var target : used) {
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      totalDistance += distance;
      minDistance = Math.min(minDistance, distance);
      onEdge = Math.max(onEdge, onEdgeFraction(target));
    }

    Logger.recordOutput("Vision/" + config.name() + "/Strategy", estimate.strategy.toString());

    return new PoseObservation(
        estimate.timestampSeconds,
        estimate.estimatedPose,
        ambiguity,
        used.size(),
        totalDistance / used.size(),
        minDistance,
        0.1,
        onEdge,
        ObservationType.PHOTONVISION);
  }

  /**
   * How close the tag's outermost corner is to the image border: 0 at the center, 1 on the edge.
   * Lens distortion is worst near the border, so {@link Vision} distrusts tags out there.
   */
  private double onEdgeFraction(PhotonTrackedTarget target) {
    double halfWidth = config.model().widthPixels() / 2.0;
    double halfHeight = config.model().heightPixels() / 2.0;
    double fraction = 0.0;
    for (TargetCorner corner : target.getDetectedCorners()) {
      fraction = Math.max(fraction, Math.abs(corner.x - halfWidth) / halfWidth);
      fraction = Math.max(fraction, Math.abs(corner.y - halfHeight) / halfHeight);
    }
    return Math.min(fraction, 1.0);
  }

  @Override
  public String getName() {
    return config.name();
  }
}
