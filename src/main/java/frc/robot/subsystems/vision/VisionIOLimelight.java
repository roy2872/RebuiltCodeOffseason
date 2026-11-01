package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.LimelightHelpers;
import frc.robot.subsystems.vision.VisionRoiController.Capabilities;
import frc.robot.subsystems.vision.VisionRoiController.CropWindow;
import frc.robot.subsystems.vision.VisionRoiController.Frame;
import frc.robot.subsystems.vision.VisionRoiController.RobotMotion;
import frc.robot.subsystems.vision.VisionRoiController.TagSample;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {

  /**
   * Limelight-only settings.
   *
   * @param pipelineIndex AprilTag pipeline to run. Its web UI crop must be left fully open (all 0)
   *     for the dynamic crop to work.
   * @param invertCropY set if the crop window appears vertically mirrored on the camera stream
   */
  public record Options(int pipelineIndex, boolean invertCropY) {
    public static final Options DEFAULT = new Options(0, false);
  }

  // Crop and detector downscale are both NetworkTables overrides that apply on the next frame.
  private static final Capabilities CAPABILITIES =
      new Capabilities(true, new double[] {1.0, 1.5, 2.0, 3.0, 4.0}, 0.0);

  private static final int BOTPOSE_HEADER_LENGTH = 11;
  private static final int BOTPOSE_VALUES_PER_TAG = 7;
  private static final double CONNECTED_TIMEOUT_SEC = 0.5;
  private static final double CROP_SEND_TOLERANCE = 0.01;

  private final Options options;
  private final String name;
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<RobotMotion> motionSupplier;
  private final VisionRoiController roiController;
  private final NetworkTable table;

  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleSubscriber heartbeatSubscriber;
  private final CameraModel model;

  private double lastHeartbeat = -1.0;
  private double lastHeartbeatChangeTime = Double.NEGATIVE_INFINITY;
  private CropWindow sentCrop = null;
  private double sentDownscale = Double.NaN;

  /**
   * @param config camera model, mounting and tuning (see {@link VisionConstants})
   * @param options Limelight-only settings
   * @param rotationSupplier robot heading, sent to the Limelight for MegaTag2
   * @param motionSupplier robot-relative motion, used to predict where tags will be in the image
   */
  public VisionIOLimelight(
      CameraConfig config,
      Options options,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<RobotMotion> motionSupplier) {
    this.options = options;
    this.name = config.name();
    this.rotationSupplier = rotationSupplier;
    this.motionSupplier = motionSupplier;
    this.roiController =
        new VisionRoiController(
            config.model(), config.robotToCamera(), config.roiTuning(), CAPABILITIES);

    table = NetworkTableInstance.getDefault().getTable(name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(-1.0);
    this.model = config.model();

    LimelightHelpers.setPipelineIndex(name, options.pipelineIndex());
    if (VisionConstants.ignoredTagIds.length > 0) {
      // Filtered on the camera, so ignored tags never enter a solve
      LimelightHelpers.SetFiducialIDFiltersOverride(name, VisionConstants.allowedTagIds());
    }
    applyCameraSettings(CropWindow.FULL, roiController.initialDownscale());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    // Heading and yaw rate for MegaTag2: [yaw, yawRate, pitch, pitchRate, roll, rollRate] in deg
    orientationPublisher.accept(
        new double[] {
          rotationSupplier.get().getDegrees(),
          Math.toDegrees(motionSupplier.get().gyroOmegaRadPerSec()),
          0.0,
          0.0,
          0.0,
          0.0
        });

    double heartbeat = heartbeatSubscriber.get();
    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeTime = now;
    }
    inputs.connected = now - lastHeartbeatChangeTime < CONNECTED_TIMEOUT_SEC;

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<YawObservation> yawObservations = new ArrayList<>();
    List<Frame> frames = new ArrayList<>();

    for (var rawResult : megatag1Subscriber.readQueue()) {
      double[] values = rawResult.value;
      if (values.length < BOTPOSE_HEADER_LENGTH) {
        continue;
      }

      // Latency-compensated FPGA timestamp of the image capture (seconds)
      double captureTimestampFPGA = captureTimestamp(rawResult.timestamp, values);

      List<TagSample> tags = new ArrayList<>();
      for (int i = BOTPOSE_HEADER_LENGTH;
          i + BOTPOSE_VALUES_PER_TAG <= values.length;
          i += BOTPOSE_VALUES_PER_TAG) {
        // [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
        TagSample tag =
            new TagSample((int) values[i], values[i + 1], values[i + 2], values[i + 3], values[i + 4]);
        tags.add(tag);
        tagIds.add(tag.id());
      }
      frames.add(new Frame(captureTimestampFPGA, tags));

      PoseObservation observation =
          toObservation(values, captureTimestampFPGA, ObservationType.MEGATAG_1);
      if (observation == null) {
        continue;
      }
      poseObservations.add(observation);

      if (values[9] <= 6.0 && observation.tagCount() > 1) {
        yawObservations.add(
            new YawObservation(captureTimestampFPGA, observation.pose().getRotation().toRotation2d()));
      }
    }

    // MegaTag2 uses the heading sent above, so it needs no heading from the tags
    for (var rawResult : megatag2Subscriber.readQueue()) {
      double[] values = rawResult.value;
      if (values.length < BOTPOSE_HEADER_LENGTH) {
        continue;
      }
      PoseObservation observation =
          toObservation(
              values, captureTimestamp(rawResult.timestamp, values), ObservationType.MEGATAG_2);
      if (observation != null) {
        poseObservations.add(observation);
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.yawObservations = yawObservations.toArray(new YawObservation[0]);

    var roi = roiController.update(frames, now, motionSupplier.get());
    applyCameraSettings(roi.crop(), roi.downscale());
    inputs.disturbed = roi.state() == VisionRoiController.State.DISTURBED;

    String prefix = "Vision/" + name + "/Roi/";
    roi.log(prefix);
    // Compare against the configured robotToCamera to catch a mismatched mounting transform.
    Logger.recordOutput(
        prefix + "LimelightCameraPoseRobotSpace",
        table.getEntry("camerapose_robotspace").getDoubleArray(new double[0]));
  }

  /** Sends crop and downscale only when they change, then flushes so they apply this loop. */
  private void applyCameraSettings(CropWindow crop, double downscale) {
    boolean changed = false;
    if (sentCrop == null || !approximatelyEqual(crop, sentCrop)) {
      double yMin = options.invertCropY() ? -crop.yMax() : crop.yMin();
      double yMax = options.invertCropY() ? -crop.yMin() : crop.yMax();
      LimelightHelpers.setCropWindow(name, crop.xMin(), crop.xMax(), yMin, yMax);
      sentCrop = crop;
      changed = true;
    }
    if (downscale != sentDownscale) {
      LimelightHelpers.SetFiducialDownscalingOverride(name, (float) downscale);
      sentDownscale = downscale;
      changed = true;
    }
    if (changed) {
      NetworkTableInstance.getDefault().flush();
    }
  }

  private static boolean approximatelyEqual(CropWindow a, CropWindow b) {
    return Math.abs(a.xMin() - b.xMin()) < CROP_SEND_TOLERANCE
        && Math.abs(a.xMax() - b.xMax()) < CROP_SEND_TOLERANCE
        && Math.abs(a.yMin() - b.yMin()) < CROP_SEND_TOLERANCE
        && Math.abs(a.yMax() - b.yMax()) < CROP_SEND_TOLERANCE;
  }

  @Override
  public String getName() {
    return name;
  }

  /** Latency-compensated FPGA timestamp of the image capture, in seconds. */
  private static double captureTimestamp(long ntTimestampMicros, double[] values) {
    return ntTimestampMicros * 1.0e-6 - values[6] * 1.0e-3;
  }

  /**
   * Builds an observation from a botpose array, with the ambiguity and edge distance taken from the
   * worst of the tags used. Returns null when no tags were used.
   */
  private PoseObservation toObservation(double[] values, double timestamp, ObservationType type) {
    int tagCount = (int) values[7];
    if (tagCount == 0) {
      return null;
    }
    double maxAmbiguity = 0.0;
    double maxOnEdge = 0.0;
    double minTagDistance = Double.POSITIVE_INFINITY;
    for (int i = BOTPOSE_HEADER_LENGTH;
        i + BOTPOSE_VALUES_PER_TAG <= values.length;
        i += BOTPOSE_VALUES_PER_TAG) {
      // [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
      maxAmbiguity = Math.max(maxAmbiguity, values[i + 6]);
      maxOnEdge = Math.max(maxOnEdge, onEdgeFraction(values[i + 1], values[i + 2]));
      minTagDistance = Math.min(minTagDistance, values[i + 4]);
    }
    return new PoseObservation(
        timestamp,
        parsePose(values),
        maxAmbiguity,
        tagCount,
        values[9],
        Double.isInfinite(minTagDistance) ? values[9] : minTagDistance,
        0.1,
        maxOnEdge,
        type);
  }

  /**
   * How close a tag's center is to the image border, 0 at the center and 1 on the edge, from its
   * angle off the principal point. Lens distortion is worst near the border.
   */
  private double onEdgeFraction(double txDeg, double tyDeg) {
    double x = Math.abs(Math.tan(Math.toRadians(txDeg))) / model.tanHalfHorizontalFov();
    double y = Math.abs(Math.tan(Math.toRadians(tyDeg))) / model.tanHalfVerticalFov();
    return Math.min(1.0, Math.max(x, y));
  }

  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
