package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.vision.VisionIO.ObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Optional;

/**
 * Decides whether a vision pose is used and how much it is trusted. Pure logic: everything it needs
 * comes in through {@link Context}, so it can be tested and replayed.
 *
 * <p>Translation std dev model:
 *
 * <pre>
 *   sigma = base * (1 + kDistance * d^2) / sqrt(tagCount)
 *           * cameraFactor * edgeFactor * motionFactor * latencyFactor * disturbanceFactor
 *   motionFactor  = 1 + kSpeed * |v| + kTurn * |omega|   (motion blur, latency error)
 *   latencyFactor = 1 + kLatency * age
 * </pre>
 *
 * All the k values are TunableNumbers. Fit base and kDistance with the stationary calibration (log a
 * still robot at measured spots, compute the spread of each spot's poses, fit sigma(d)).
 */
public final class VisionFilter {
  private VisionFilter() {}

  // Translation std dev model
  private static final TunableNumber linearBase = new TunableNumber("Vision/StdDev/LinearBase", 0.004);
  private static final TunableNumber kDistance = new TunableNumber("Vision/StdDev/kDistance", 1.0);
  private static final TunableNumber kSpeed = new TunableNumber("Vision/StdDev/kSpeed", 0.5);
  private static final TunableNumber kTurn = new TunableNumber("Vision/StdDev/kTurn", 0.5);
  private static final TunableNumber kLatency = new TunableNumber("Vision/StdDev/kLatency", 2.0);
  private static final TunableNumber disturbedFactor =
      new TunableNumber("Vision/StdDev/DisturbedFactor", 3.0);
  // MegaTag2 is more stable than a full 3D solve
  private static final TunableNumber megatag2Factor =
      new TunableNumber("Vision/StdDev/Megatag2Factor", 0.5);

  // Heading std dev, only used for good multi-tag solves (see useVisionHeading)
  private static final TunableNumber angularBase = new TunableNumber("Vision/StdDev/AngularBase", 0.05);
  private static final TunableNumber maxHeadingTagDistance =
      new TunableNumber("Vision/MaxHeadingTagDistance", 4.0);
  private static final TunableNumber maxHeadingAmbiguity =
      new TunableNumber("Vision/MaxHeadingAmbiguity", 0.15);

  // Rejection
  private static final TunableNumber maxSingleTagDistance =
      new TunableNumber("Vision/MaxSingleTagDistance", 3.5);
  // Limelight's recommendation: MegaTag2 is unreliable above this yaw rate
  private static final TunableNumber maxMegatag2YawRateDegPerSec =
      new TunableNumber("Vision/MaxMegatag2YawRateDegPerSec", 720.0);

  // Outlier gate
  private static final TunableNumber gateMinMeters = new TunableNumber("Vision/Gate/MinMeters", 0.75);
  private static final TunableNumber gateSigmas = new TunableNumber("Vision/Gate/Sigmas", 4.0);
  private static final TunableNumber gateRecoverCount = new TunableNumber("Vision/Gate/RecoverCount", 8);
  private static final TunableNumber gateAgreeMeters = new TunableNumber("Vision/Gate/AgreeMeters", 0.3);

  /**
   * What the filter knows about the robot when an observation arrives.
   *
   * @param ageSec how old the image is when it reaches the filter
   * @param disturbed robot motion is unpredictable (pushed, wheels slipping)
   */
  public record Context(
      boolean enabled,
      double linearSpeedMetersPerSec,
      double yawRateRadPerSec,
      double ageSec,
      boolean disturbed) {}

  public record StdDevs(double linear, double angular) {}

  /** Returns why the observation should not be used, or empty to use it. */
  public static Optional<String> rejectReason(PoseObservation observation, Context context) {
    if (observation.tagCount() == 0) {
      return Optional.of("NoTags");
    }
    boolean megatag2 = observation.type() == ObservationType.MEGATAG_2;
    if (observation.type() == ObservationType.MEGATAG_1 && useMegatag2 && context.enabled()) {
      return Optional.of("Megatag1WhileEnabled");
    }
    if (megatag2 && (!useMegatag2 || !context.enabled())) {
      return Optional.of("Megatag2WhileDisabled");
    }
    if (megatag2
        && Math.abs(Math.toDegrees(context.yawRateRadPerSec()))
            > maxMegatag2YawRateDegPerSec.getAsDouble()) {
      return Optional.of("SpinningTooFast");
    }
    if (Math.abs(observation.pose().getZ()) > maxZError) {
      return Optional.of("ZError");
    }
    if (observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
      return Optional.of("OutOfField");
    }
    if (observation.tagCount() == 1) {
      // MegaTag2 takes heading from the gyro, so single tags can't flip and stay usable further out
      if (!megatag2 && observation.ambiguity() > maxAmbiguity) {
        return Optional.of("Ambiguity");
      }
      if (observation.onEdgePercentage() > ON_EDGE_THRESHOLD) {
        return Optional.of("OnEdge");
      }
      if (!megatag2 && observation.averageTagDistance() > maxSingleTagDistance.getAsDouble()) {
        return Optional.of("SingleTagTooFar");
      }
    }
    return Optional.empty();
  }

  public static StdDevs stdDevs(PoseObservation observation, double cameraFactor, Context context) {
    double distance = observation.averageTagDistance();
    double tagFactor = 1.0 / Math.sqrt(Math.max(1, observation.tagCount()));
    // Distrust tags near the image edge, where lens distortion is worst
    double edgeFactor =
        Math.exp(Math.pow(observation.onEdgePercentage(), 2.5) / (2 * Math.pow(0.5, 2.0)));
    double motionFactor =
        1.0
            + kSpeed.getAsDouble() * Math.abs(context.linearSpeedMetersPerSec())
            + kTurn.getAsDouble() * Math.abs(context.yawRateRadPerSec());
    double latencyFactor = 1.0 + kLatency.getAsDouble() * Math.max(0.0, context.ageSec());
    double disturbanceFactor = context.disturbed() ? disturbedFactor.getAsDouble() : 1.0;
    double typeFactor =
        observation.type() == ObservationType.MEGATAG_2 ? megatag2Factor.getAsDouble() : 1.0;

    double distanceFactor = 1.0 + kDistance.getAsDouble() * distance * distance;
    double common =
        distanceFactor
            * tagFactor
            * cameraFactor
            * motionFactor
            * latencyFactor
            * disturbanceFactor;
    double linear = linearBase.getAsDouble() * common * edgeFactor * typeFactor;

    boolean headingUsable =
        useVisionHeading
            && observation.type() != ObservationType.MEGATAG_2
            && observation.tagCount() >= 2
            && distance < maxHeadingTagDistance.getAsDouble()
            && observation.ambiguity() < maxHeadingAmbiguity.getAsDouble()
            // A Limelight's MegaTag1 heading only seeds the gyro before the match
            && !(observation.type() == ObservationType.MEGATAG_1 && context.enabled());
    double angular = headingUsable ? angularBase.getAsDouble() * common : Double.POSITIVE_INFINITY;
    return new StdDevs(linear, angular);
  }

  /**
   * Rejects poses far from the current estimate, which are almost always bad solves. If many
   * rejected poses in a row agree with each other, the estimate is the one that's wrong (after a
   * collision or wheel slip), so they are let through to pull it back. One per camera.
   */
  public static final class InnovationGate {
    private int consecutiveRejected = 0;
    private Pose2d lastRejected = null;

    /**
     * @param estimate the pose estimate at the observation's timestamp, empty if unknown
     * @return true to use the observation
     */
    public boolean accept(
        Pose2d visionPose, Optional<Pose2d> estimate, double linearStdDev, boolean enabled) {
      // Not gated while disabled: the robot may have been placed anywhere
      if (!enabled || estimate.isEmpty()) {
        reset();
        return true;
      }
      double error = visionPose.getTranslation().getDistance(estimate.get().getTranslation());
      double gate = Math.max(gateMinMeters.getAsDouble(), gateSigmas.getAsDouble() * linearStdDev);
      if (error <= gate) {
        reset();
        return true;
      }

      boolean agreesWithLast =
          lastRejected != null
              && visionPose.getTranslation().getDistance(lastRejected.getTranslation())
                  < gateAgreeMeters.getAsDouble();
      consecutiveRejected = agreesWithLast ? consecutiveRejected + 1 : 1;
      lastRejected = visionPose;
      if (consecutiveRejected >= gateRecoverCount.getAsDouble()) {
        reset();
        return true;
      }
      return false;
    }

    private void reset() {
      consecutiveRejected = 0;
      lastRejected = null;
    }
  }
}
