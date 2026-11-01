package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Picks a crop window and AprilTag detector downscale for any vision camera every robot loop.
 * What the camera can actually do (dynamic crop, which downscale levels, how often it may switch)
 * comes from {@link Capabilities}, declared by each vendor IO.
 *
 * <p>Crop: every tag seen recently is kept as a 3D point in the camera frame (from its bearing and
 * distance). The point is moved forward in time with the robot's measured motion (gyro yaw rate,
 * wheel speeds) to the moment the next frame using the new crop will be captured, then projected
 * back into the image. The box around it covers the tag, a quiet zone, and every position the tag
 * could reach if the robot accelerates as hard as allowed. The crop is the union over all tracked
 * tags so multi-tag solves are kept.
 *
 * <p>Defense: wheel speeds lie when the robot is pushed. If the gyro and wheels disagree, or a tag
 * shows up far from where it was predicted, the controller goes full frame for a while so the
 * robot can re-localize from every tag in view, and widens its margins until predictions are good
 * again.
 *
 * <p>Downscale (Limelight detector downscale, PhotonVision decimate): chosen so the smallest
 * tracked tag still covers enough pixels after downscaling. Corners are refined at full
 * resolution, so it costs detection range, not 3D accuracy.
 *
 * <p>Image coordinates are normalized to [-1, 1] with +x right and +y up. Pure logic (no
 * NetworkTables) so it can be unit tested and reused on other robots.
 */
public class VisionRoiController {

  public enum State {
    /** No tags tracked: full frame, full resolution to find tags at range. */
    SEARCHING,
    /** Cropped around the predicted tag positions. */
    TRACKING,
    /** Periodic full frame to pick up tags that entered the view outside the crop. */
    REFRESHING,
    /** Robot motion is unpredictable (pushed, wheels slipping): full frame. */
    DISTURBED
  }

  /**
   * @param tagSizeMeters outer black square edge length of the field tags
   * @param tagPadFraction quiet zone added around a tag on each side, as a fraction of tag size
   * @param minCropHalfSize smallest allowed crop half width / height (normalized)
   * @param maxCropAreaFraction crops bigger than this fraction of the image use the full frame
   * @param commandLatencySec time from sending a crop until the camera captures with it
   * @param maxLinearAccel unpredicted robot acceleration to cover (m/s^2)
   * @param maxAngularAccel unpredicted robot angular acceleration to cover (rad/s^2)
   * @param linearVelocityNoise error in the wheel speed estimate (m/s)
   * @param angularVelocityNoise error in the gyro yaw rate (rad/s)
   * @param tiltNoiseRad chassis pitch / roll rocking not measured by anything (rad)
   * @param missGrowthPerFrame crop half size growth for each frame a tracked tag was missed
   * @param trackTimeoutSec a tag not seen for this long is dropped; with none left, full frame
   * @param refreshPeriodSec while tracking, take a full frame this often (<= 0 disables)
   * @param minTagPixels smallest tag edge, in downscaled pixels, the detector should be given
   * @param downscaleHysteresis margin required before moving to a larger downscale
   * @param searchDownscale downscale while searching, 1 for maximum range (nearest available)
   * @param maxAdaptiveScale largest margin multiplier from bad predictions
   * @param adaptiveScaleDecay per-frame decay of the margin multiplier back toward 1
   * @param disturbanceInnovationRatio prediction error / margin above which robot is disturbed
   * @param wheelSlipOmegaThreshold gyro vs wheel yaw rate difference that means slip (rad/s)
   * @param disturbanceHoldSec how long to stay full frame after a disturbance
   */
  public record Tuning(
      double tagSizeMeters,
      double tagPadFraction,
      double minCropHalfSize,
      double maxCropAreaFraction,
      double commandLatencySec,
      double maxLinearAccel,
      double maxAngularAccel,
      double linearVelocityNoise,
      double angularVelocityNoise,
      double tiltNoiseRad,
      double missGrowthPerFrame,
      double trackTimeoutSec,
      double refreshPeriodSec,
      double minTagPixels,
      double downscaleHysteresis,
      double searchDownscale,
      double maxAdaptiveScale,
      double adaptiveScaleDecay,
      double disturbanceInnovationRatio,
      double wheelSlipOmegaThreshold,
      double disturbanceHoldSec) {}

  /**
   * What a camera backend can do.
   *
   * @param supportsCrop whether the detection region can be changed every frame
   * @param downscaleLevels available downscale factors, ascending, starting at 1
   * @param minDownscaleSwitchIntervalSec minimum time between downscale changes (switching a
   *     pipeline can cost frames; 0 if switching is free)
   */
  public record Capabilities(
      boolean supportsCrop, double[] downscaleLevels, double minDownscaleSwitchIntervalSec) {}

  /**
   * One tag in one frame.
   *
   * @param yawDeg horizontal angle from the image center, positive right
   * @param pitchDeg vertical angle from the image center, positive up
   * @param areaPercent tag area as a percent of the image, 0 if unknown
   * @param distanceMeters camera to tag distance, 0 if unknown (then estimated from area)
   */
  public record TagSample(
      int id, double yawDeg, double pitchDeg, double areaPercent, double distanceMeters) {}

  /** One processed camera frame. {@code tags} is empty if nothing was detected. */
  public record Frame(double captureTimestamp, List<TagSample> tags) {}

  /**
   * Robot-relative motion. Yaw rate from the gyro is trusted for prediction; the wheel yaw rate
   * is only used to detect slip.
   */
  public record RobotMotion(
      double vxMetersPerSec,
      double vyMetersPerSec,
      double gyroOmegaRadPerSec,
      double wheelOmegaRadPerSec) {}

  /** Normalized image rectangle, +x right, +y up. */
  public record CropWindow(double xMin, double xMax, double yMin, double yMax) {
    public static final CropWindow FULL = new CropWindow(-1.0, 1.0, -1.0, 1.0);

    public double areaFraction() {
      return (xMax - xMin) * (yMax - yMin) / 4.0;
    }

    public boolean isFull() {
      return areaFraction() >= 0.999;
    }

    public boolean contains(double x, double y) {
      return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
    }

    CropWindow union(CropWindow o) {
      return new CropWindow(
          Math.min(xMin, o.xMin), Math.max(xMax, o.xMax),
          Math.min(yMin, o.yMin), Math.max(yMax, o.yMax));
    }

    boolean intersectsImage() {
      return xMax > -1.0 && xMin < 1.0 && yMax > -1.0 && yMin < 1.0;
    }

    public double[] toArray() {
      return new double[] {xMin, xMax, yMin, yMax};
    }
  }

  public record Output(
      CropWindow crop,
      double downscale,
      State state,
      double adaptiveScale,
      double framePeriodSec,
      int[] trackedTagIds) {

    public void log(String prefix) {
      Logger.recordOutput(prefix + "State", state.name());
      Logger.recordOutput(prefix + "Crop", crop.toArray());
      Logger.recordOutput(prefix + "CropAreaFraction", crop.areaFraction());
      Logger.recordOutput(prefix + "Downscale", downscale);
      Logger.recordOutput(prefix + "AdaptiveScale", adaptiveScale);
      Logger.recordOutput(prefix + "FramePeriodSec", framePeriodSec);
      Logger.recordOutput(prefix + "TrackedTagIds", trackedTagIds);
    }
  }

  private static final class Track {
    double timestamp;
    Translation3d pointInCamera;
    double sizePixels;
    int misses;
  }

  private record TrackPrediction(CropWindow box, double sizePixels) {}

  private static final double MIN_DEPTH_METERS = 0.05;
  private static final double MEASUREMENT_NOISE_NORMALIZED = 0.01;
  private static final double MAX_INNOVATION_DT_SEC = 0.2;
  private static final double MAX_PREDICTION_DT_SEC = 0.5;
  private static final double REFRESH_TIMEOUT_SEC = 0.5;

  private final CameraModel model;
  private final Capabilities capabilities;
  private final double searchDownscale;
  private final Transform3d robotToCamera;
  private final Tuning tuning;
  private final double tanHalfH;
  private final double tanHalfV;

  private final Map<Integer, Track> tracks = new HashMap<>();
  private State state = State.SEARCHING;
  private double framePeriodSec = 1.0 / 40.0;
  private double lastFrameTimestamp = Double.NaN;
  private double adaptiveScale = 1.0;
  private double disturbedUntil = Double.NEGATIVE_INFINITY;
  private double lastRefreshTime = Double.NEGATIVE_INFINITY;
  private double refreshStartTime = Double.NaN;
  private double currentDownscale;
  private double lastDownscaleChangeTime = Double.NEGATIVE_INFINITY;

  public VisionRoiController(
      CameraModel model, Transform3d robotToCamera, Tuning tuning, Capabilities capabilities) {
    this.model = model;
    this.capabilities = capabilities;
    this.robotToCamera = robotToCamera;
    this.tuning = tuning;
    this.tanHalfH = model.tanHalfHorizontalFov();
    this.tanHalfV = model.tanHalfVerticalFov();
    this.searchDownscale = nearestLevel(tuning.searchDownscale());
    this.currentDownscale = searchDownscale;
  }

  /** Downscale to use before the first update. */
  public double initialDownscale() {
    return searchDownscale;
  }

  /**
   * @param frames frames received since the last call, oldest first (may be empty)
   * @param now current FPGA time in seconds
   * @param motion current robot-relative motion
   */
  public Output update(List<Frame> frames, double now, RobotMotion motion) {
    if (Math.abs(motion.gyroOmegaRadPerSec() - motion.wheelOmegaRadPerSec())
        > tuning.wheelSlipOmegaThreshold()) {
      disturb(now);
    }

    for (Frame frame : frames) {
      ingest(frame, motion, now);
    }

    tracks.values().removeIf(t -> now - t.timestamp > tuning.trackTimeoutSec());
    if (!Double.isNaN(refreshStartTime) && now - refreshStartTime > REFRESH_TIMEOUT_SEC) {
      endRefresh(now);
    }

    return decide(now, motion);
  }

  private void ingest(Frame frame, RobotMotion motion, double now) {
    double ts = frame.captureTimestamp();
    if (!Double.isNaN(lastFrameTimestamp) && ts > lastFrameTimestamp) {
      double period = MathUtil.clamp(ts - lastFrameTimestamp, 1.0 / 200.0, 0.2);
      framePeriodSec = 0.8 * framePeriodSec + 0.2 * period;
    }
    lastFrameTimestamp = Math.max(ts, Double.isNaN(lastFrameTimestamp) ? ts : lastFrameTimestamp);

    if (!Double.isNaN(refreshStartTime) && ts >= refreshStartTime + tuning.commandLatencySec()) {
      endRefresh(now);
    }

    boolean raised = false;
    Set<Integer> seen = new HashSet<>();
    for (TagSample sample : frame.tags()) {
      double distance =
          sample.distanceMeters() > 0.0 ? sample.distanceMeters() : distanceFromArea(sample);
      if (distance <= 0.0) continue;
      seen.add(sample.id());
      Translation3d measured = measurementToCameraPoint(sample, distance);
      Track track = tracks.get(sample.id());
      if (track == null) {
        track = new Track();
        tracks.put(sample.id(), track);
      } else {
        double dt = ts - track.timestamp;
        if (dt > 0.0 && dt <= MAX_INNOVATION_DT_SEC) {
          raised |= checkInnovation(track, measured, motion, dt, now);
        }
      }
      if (ts >= track.timestamp) {
        track.timestamp = ts;
        track.pointInCamera = measured;
        track.sizePixels = measuredSizePixels(sample, distance);
        track.misses = 0;
      }
    }

    for (var entry : tracks.entrySet()) {
      if (!seen.contains(entry.getKey()) && entry.getValue().timestamp < ts) {
        entry.getValue().misses++;
      }
    }

    if (!raised) {
      adaptiveScale = Math.max(1.0, adaptiveScale * tuning.adaptiveScaleDecay());
    }
  }

  /**
   * Compares where a tag was seen with where it was predicted from its previous detection. A miss
   * larger than the motion margin means robot motion isn't what the sensors say: widen margins,
   * and if it's far off, treat the robot as disturbed. Returns whether margins were raised.
   */
  private boolean checkInnovation(
      Track track, Translation3d measured, RobotMotion motion, double dt, double now) {
    double[] predicted = project(propagate(track.pointInCamera, motion, dt, 0.0, 0.0, 0.0));
    double[] actual = project(measured);
    CropWindow motionBox = motionBounds(track.pointInCamera, motion, dt, 1.0);
    if (predicted == null || actual == null || motionBox == null) return false;

    double ux = Math.max(predicted[0] - motionBox.xMin(), motionBox.xMax() - predicted[0]);
    double uy = Math.max(predicted[1] - motionBox.yMin(), motionBox.yMax() - predicted[1]);
    double ratio =
        Math.max(
            Math.abs(actual[0] - predicted[0]) / (ux + MEASUREMENT_NOISE_NORMALIZED),
            Math.abs(actual[1] - predicted[1]) / (uy + MEASUREMENT_NOISE_NORMALIZED));

    if (ratio > tuning.disturbanceInnovationRatio()) {
      disturb(now);
    }
    if (ratio > 1.0) {
      adaptiveScale =
          Math.min(tuning.maxAdaptiveScale(), Math.max(adaptiveScale, ratio * 1.25));
      return true;
    }
    return false;
  }

  private Output decide(double now, RobotMotion motion) {
    // Capture time of the first frame that can use the crop sent now.
    double target = now + tuning.commandLatencySec() + framePeriodSec;

    CropWindow union = null;
    double minSizePixels = Double.POSITIVE_INFINITY;
    for (Track track : tracks.values()) {
      double dt = MathUtil.clamp(target - track.timestamp, 0.0, MAX_PREDICTION_DT_SEC);
      TrackPrediction prediction = predictTrack(track, motion, dt);
      if (prediction == null || !prediction.box().intersectsImage()) continue;
      union = union == null ? prediction.box() : union.union(prediction.box());
      minSizePixels = Math.min(minSizePixels, prediction.sizePixels());
    }

    if (union == null) {
      boolean disturbed = now < disturbedUntil;
      return output(
          CropWindow.FULL,
          now - lastDownscaleChangeTime < capabilities.minDownscaleSwitchIntervalSec()
              ? currentDownscale
              : searchDownscale,
          disturbed ? State.DISTURBED : State.SEARCHING,
          now);
    }

    double downscale = chooseDownscale(minSizePixels, now);

    if (now < disturbedUntil) {
      return output(CropWindow.FULL, downscale, State.DISTURBED, now);
    }

    if (state == State.SEARCHING || state == State.DISTURBED) {
      // A full frame was just taken, no need to refresh right away.
      lastRefreshTime = now;
    }
    if (tuning.refreshPeriodSec() > 0.0
        && Double.isNaN(refreshStartTime)
        && now - lastRefreshTime >= tuning.refreshPeriodSec()) {
      refreshStartTime = now;
    }
    if (!Double.isNaN(refreshStartTime)) {
      return output(CropWindow.FULL, downscale, State.REFRESHING, now);
    }

    CropWindow crop = capabilities.supportsCrop() ? finalizeCrop(union) : CropWindow.FULL;
    if (crop.areaFraction() > tuning.maxCropAreaFraction()) {
      crop = CropWindow.FULL;
    }
    return output(crop, downscale, State.TRACKING, now);
  }

  private Output output(CropWindow crop, double downscale, State newState, double now) {
    state = newState;
    if (downscale != currentDownscale) {
      currentDownscale = downscale;
      lastDownscaleChangeTime = now;
    }
    int[] ids = tracks.keySet().stream().mapToInt(Integer::intValue).sorted().toArray();
    return new Output(crop, downscale, newState, adaptiveScale, framePeriodSec, ids);
  }

  /** Box covering the tag and its quiet zone at every position the tag can reach in dt. */
  private TrackPrediction predictTrack(Track track, RobotMotion motion, double dt) {
    CropWindow motionBox = motionBounds(track.pointInCamera, motion, dt, adaptiveScale);
    if (motionBox == null) return null;

    Translation3d nominal = propagate(track.pointInCamera, motion, dt, 0.0, 0.0, 0.0);
    double distance = Math.max(nominal.getNorm(), MIN_DEPTH_METERS);
    double[] center = project(nominal);
    double cx = center != null ? center[0] : (motionBox.xMin() + motionBox.xMax()) / 2.0;
    double cy = center != null ? center[1] : (motionBox.yMin() + motionBox.yMax()) / 2.0;

    // Angular half size of the tag plus quiet zone, converted to normalized units. The
    // (1 + tan^2) factor accounts for the image stretching toward the edges.
    double tagHalfAngle =
        Math.atan(tuning.tagSizeMeters() * (0.5 + tuning.tagPadFraction()) / distance);
    double tiltAngle = tuning.tiltNoiseRad() * adaptiveScale;
    double stretchX = 1.0 + Math.pow(cx * tanHalfH, 2);
    double stretchY = 1.0 + Math.pow(cy * tanHalfV, 2);
    double halfX = (tagHalfAngle + tiltAngle) * stretchX / tanHalfH;
    double halfY = (tagHalfAngle + tiltAngle) * stretchY / tanHalfV;

    double growth = 1.0 + tuning.missGrowthPerFrame() * track.misses;
    double midX = (motionBox.xMin() + motionBox.xMax()) / 2.0;
    double midY = (motionBox.yMin() + motionBox.yMax()) / 2.0;
    double spanX = ((motionBox.xMax() - motionBox.xMin()) / 2.0 + halfX) * growth;
    double spanY = ((motionBox.yMax() - motionBox.yMin()) / 2.0 + halfY) * growth;

    double sizePixels = track.sizePixels * track.pointInCamera.getNorm() / distance;
    return new TrackPrediction(
        new CropWindow(midX - spanX, midX + spanX, midY - spanY, midY + spanY), sizePixels);
  }

  /**
   * Bounding box of the tag center's projection over a 3x3x3 grid of velocity errors. The error
   * covers sensor noise plus the given acceleration limits over dt, times {@code scale}.
   */
  private CropWindow motionBounds(
      Translation3d pointInCamera, RobotMotion motion, double dt, double scale) {
    double dv = scale * (tuning.linearVelocityNoise() + 0.5 * tuning.maxLinearAccel() * dt);
    double dw = scale * (tuning.angularVelocityNoise() + 0.5 * tuning.maxAngularAccel() * dt);
    double xMin = Double.POSITIVE_INFINITY, xMax = Double.NEGATIVE_INFINITY;
    double yMin = Double.POSITIVE_INFINITY, yMax = Double.NEGATIVE_INFINITY;
    boolean any = false;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          double[] uv = project(propagate(pointInCamera, motion, dt, i * dv, j * dv, k * dw));
          if (uv == null) continue;
          any = true;
          xMin = Math.min(xMin, uv[0]);
          xMax = Math.max(xMax, uv[0]);
          yMin = Math.min(yMin, uv[1]);
          yMax = Math.max(yMax, uv[1]);
        }
      }
    }
    return any ? new CropWindow(xMin, xMax, yMin, yMax) : null;
  }

  /** Where a field-fixed point seen in the camera frame will be after the robot moves for dt. */
  private Translation3d propagate(
      Translation3d pointInCamera,
      RobotMotion motion,
      double dt,
      double dvx,
      double dvy,
      double domega) {
    Translation3d inRobot =
        pointInCamera.rotateBy(robotToCamera.getRotation()).plus(robotToCamera.getTranslation());
    Pose2d robotDelta =
        Pose2d.kZero.exp(
            new Twist2d(
                (motion.vxMetersPerSec() + dvx) * dt,
                (motion.vyMetersPerSec() + dvy) * dt,
                (motion.gyroOmegaRadPerSec() + domega) * dt));
    Translation2d moved =
        inRobot
            .toTranslation2d()
            .minus(robotDelta.getTranslation())
            .rotateBy(robotDelta.getRotation().unaryMinus());
    Translation3d inNewRobot = new Translation3d(moved.getX(), moved.getY(), inRobot.getZ());
    return inNewRobot
        .minus(robotToCamera.getTranslation())
        .rotateBy(robotToCamera.getRotation().unaryMinus());
  }

  /** Pinhole projection to normalized image coordinates, or null if behind the camera. */
  private double[] project(Translation3d p) {
    if (p.getX() < MIN_DEPTH_METERS) return null;
    return new double[] {
      (-p.getY() / p.getX()) / tanHalfH, (p.getZ() / p.getX()) / tanHalfV
    };
  }

  private Translation3d measurementToCameraPoint(TagSample sample, double distance) {
    double tx = Math.tan(Math.toRadians(sample.yawDeg()));
    double ty = Math.tan(Math.toRadians(sample.pitchDeg()));
    double depth = distance / Math.sqrt(1.0 + tx * tx + ty * ty);
    return new Translation3d(depth, -tx * depth, ty * depth);
  }

  /** Apparent tag edge length in full-resolution pixels. */
  private double measuredSizePixels(TagSample sample, double distance) {
    if (sample.areaPercent() > 0.0) {
      return Math.sqrt(
          sample.areaPercent() / 100.0 * model.widthPixels() * model.heightPixels());
    }
    return tuning.tagSizeMeters() / distance / tanHalfH * model.widthPixels() / 2.0;
  }

  /** Rough frontal-tag distance from image area, for cameras that don't report distance. */
  private double distanceFromArea(TagSample sample) {
    if (sample.areaPercent() <= 0.0) return 0.0;
    double sizePixels =
        Math.sqrt(sample.areaPercent() / 100.0 * model.widthPixels() * model.heightPixels());
    double tanAngle = sizePixels / (model.widthPixels() / 2.0) * tanHalfH;
    return tuning.tagSizeMeters() / tanAngle;
  }

  private double chooseDownscale(double minSizePixels, double now) {
    if (now - lastDownscaleChangeTime < capabilities.minDownscaleSwitchIntervalSec()) {
      return currentDownscale;
    }
    double[] levels = capabilities.downscaleLevels();
    double best = levels[0];
    for (double level : levels) {
      double needed =
          tuning.minTagPixels() * (level > currentDownscale ? tuning.downscaleHysteresis() : 1.0);
      if (minSizePixels / level >= needed) {
        best = level;
      }
    }
    return best;
  }

  private double nearestLevel(double downscale) {
    double best = capabilities.downscaleLevels()[0];
    for (double level : capabilities.downscaleLevels()) {
      if (Math.abs(level - downscale) < Math.abs(best - downscale)) best = level;
    }
    return best;
  }

  private CropWindow finalizeCrop(CropWindow box) {
    double cx = (box.xMin() + box.xMax()) / 2.0;
    double cy = (box.yMin() + box.yMax()) / 2.0;
    double hx = Math.max((box.xMax() - box.xMin()) / 2.0, tuning.minCropHalfSize());
    double hy = Math.max((box.yMax() - box.yMin()) / 2.0, tuning.minCropHalfSize());
    return new CropWindow(
        MathUtil.clamp(cx - hx, -1.0, 1.0),
        MathUtil.clamp(cx + hx, -1.0, 1.0),
        MathUtil.clamp(cy - hy, -1.0, 1.0),
        MathUtil.clamp(cy + hy, -1.0, 1.0));
  }

  private void disturb(double now) {
    disturbedUntil = Math.max(disturbedUntil, now + tuning.disturbanceHoldSec());
  }

  private void endRefresh(double now) {
    refreshStartTime = Double.NaN;
    lastRefreshTime = now;
  }
}
