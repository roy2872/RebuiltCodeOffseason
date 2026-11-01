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
import frc.robot.RobotState;
import frc.robot.RobotState.LimelightYawObservation;
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
  private final DoubleSubscriber heartbeatSubscriber;

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
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(-1.0);

    LimelightHelpers.setPipelineIndex(name, options.pipelineIndex());
    applyCameraSettings(CropWindow.FULL, roiController.initialDownscale());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

    double heartbeat = heartbeatSubscriber.get();
    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeTime = now;
    }
    inputs.connected = now - lastHeartbeatChangeTime < CONNECTED_TIMEOUT_SEC;

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<Frame> frames = new ArrayList<>();

    for (var rawResult : megatag1Subscriber.readQueue()) {
      double[] values = rawResult.value;
      if (values.length < BOTPOSE_HEADER_LENGTH) {
        continue;
      }

      // Latency-compensated FPGA timestamp of the image capture (seconds)
      double captureTimestampFPGA = rawResult.timestamp * 1.0e-6 - values[6] * 1.0e-3;

      List<TagSample> tags = new ArrayList<>();
      double minTagDistance = Double.POSITIVE_INFINITY;
      for (int i = BOTPOSE_HEADER_LENGTH;
          i + BOTPOSE_VALUES_PER_TAG <= values.length;
          i += BOTPOSE_VALUES_PER_TAG) {
        // [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
        TagSample tag =
            new TagSample((int) values[i], values[i + 1], values[i + 2], values[i + 3], values[i + 4]);
        tags.add(tag);
        tagIds.add(tag.id());
        minTagDistance = Math.min(minTagDistance, tag.distanceMeters());
      }
      frames.add(new Frame(captureTimestampFPGA, tags));

      int tagCount = (int) values[7];
      if (tagCount == 0) {
        continue;
      }

      Pose3d pose = parsePose(values);
      poseObservations.add(
          new PoseObservation(
              captureTimestampFPGA,
              pose,
              values.length >= 18 ? values[17] : 0.0,
              tagCount,
              values[9],
              tags.isEmpty() ? values[9] : minTagDistance,
              0.1,
              0.0));

      if (values[9] <= 6.0 && tagCount > 1) {
        RobotState.mInstance.addLimelightYawObservation(
            new LimelightYawObservation(pose.getRotation().toRotation2d(), captureTimestampFPGA));
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();

    var roi = roiController.update(frames, now, motionSupplier.get());
    applyCameraSettings(roi.crop(), roi.downscale());

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
