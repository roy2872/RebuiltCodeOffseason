package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.LimelightYawObservation;
import frc.robot.subsystems.vision.VisionIO.ObservationType;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private Alert[] visionAlerts;
  private final VisionConsumer consumer;
  private final VisionFilter.InnovationGate[] gates;

  List<Pose3d> allTagPoses, allRobotPoses, allRobotPosesAccepted, allRobotPosesRejected;
  List<Pose3d> tagPoses, robotPoses, robotPosesAccepted, robotPosesRejected;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.gates = new VisionFilter.InnovationGate[io.length];
    for (int i = 0; i < io.length; i++) {
      gates[i] = new VisionFilter.InnovationGate();
    }

    this.visionAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++)
      visionAlerts[i] =
          new Alert("Vision camera " + io[i].getName() + " is disconnected!", AlertType.kWarning);
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      // Records the camera data, or in replay overwrites it from the log. Logic below must only
      // read inputs[i].
      Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
      for (var yaw : inputs[i].yawObservations) {
        RobotState.mInstance.addLimelightYawObservation(
            new LimelightYawObservation(yaw.yaw(), yaw.timestamp()));
      }
    }

    // Initialize logging values
    allTagPoses = new LinkedList<>();
    allRobotPoses = new LinkedList<>();
    allRobotPosesAccepted = new LinkedList<>();
    allRobotPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Per-camera logging values, fresh for each camera so logs don't mix cameras
      tagPoses = new LinkedList<>();
      robotPoses = new LinkedList<>();
      robotPosesAccepted = new LinkedList<>();
      robotPosesRejected = new LinkedList<>();

      visionAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }
      // Loop over pose observations
      List<String> rejectReasons = new ArrayList<>();
      List<Double> acceptedLinearStdDevs = new ArrayList<>();
      double cameraFactor =
          cameraIndex < cameraStdDevFactors.length ? cameraStdDevFactors[cameraIndex] : 1.0;
      for (var observation : inputs[cameraIndex].poseObservations) {
        VisionFilter.Context context = context(observation, inputs[cameraIndex].disturbed);
        Pose2d visionPose = observation.pose().toPose2d();

        // Check whether to reject pose
        Optional<String> rejectReason = VisionFilter.rejectReason(observation, context);
        // Taken on the bump: the camera was tilted and MegaTag2 assumes it wasn't
        if (rejectReason.isEmpty() && RobotState.mInstance.wasTiltedAt(observation.timestamp())) {
          rejectReason = Optional.of("Tilted");
        }
        VisionFilter.StdDevs stdDevs = VisionFilter.stdDevs(observation, cameraFactor, context);
        if (rejectReason.isEmpty()
            && !gates[cameraIndex].accept(
                visionPose,
                RobotState.mInstance.getEstimatedPoseAtTimestamp(observation.timestamp()),
                stdDevs.linear(),
                context.enabled())) {
          rejectReason = Optional.of("FarFromEstimate");
        }

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectReason.isPresent()) {
          robotPosesRejected.add(observation.pose());
          rejectReasons.add(observation.type() + ":" + rejectReason.get());
          continue;
        }
        robotPosesAccepted.add(observation.pose());
        acceptedLinearStdDevs.add(stdDevs.linear());
        // Plain numbers for tools/fit_vision_stddev.py (export to CSV from AdvantageScope)
        String calibration = "Vision/Camera" + cameraIndex + "/Calibration/";
        Logger.recordOutput(calibration + "Timestamp", observation.timestamp());
        Logger.recordOutput(calibration + "X", observation.pose().getX());
        Logger.recordOutput(calibration + "Y", observation.pose().getY());
        Logger.recordOutput(calibration + "Distance", observation.averageTagDistance());
        Logger.recordOutput(calibration + "TagCount", observation.tagCount());
        Logger.recordOutput(calibration + "Megatag2", observation.type() == ObservationType.MEGATAG_2);

        // send to pose estimator
        consumer.accept(
            visionPose,
            observation.timestamp(),
            VecBuilder.fill(stdDevs.linear(), stdDevs.linear(), stdDevs.angular()));
      }
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RejectReasons", rejectReasons.toArray(new String[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/AcceptedLinearStdDevs",
          acceptedLinearStdDevs.stream().mapToDouble(Double::doubleValue).toArray());

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  public boolean allCamerasConnected() {
    for (var input : inputs) {
      if (!input.connected) return false;
    }
    return true;
  }

  public int getCameraCount() {
    return io.length;
  }

  public String getCameraName(int cameraIndex) {
    return io[cameraIndex].getName();
  }

  /** Whether this camera saw at least one tag this loop. */
  public boolean cameraSeesTag(int cameraIndex) {
    return inputs[cameraIndex].tagIds.length > 0;
  }

  private static VisionFilter.Context context(PoseObservation observation, boolean disturbed) {
    ChassisSpeeds speeds = RobotState.mInstance.getRobotVelocity();
    return new VisionFilter.Context(
        DriverStation.isEnabled(),
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
        RobotState.mInstance.getGyroYawRateRadPerSec(),
        Timer.getFPGATimestamp() - observation.timestamp(),
        disturbed);
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
