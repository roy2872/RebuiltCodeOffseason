package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Runs the real PhotonVision estimation code against the PhotonVision camera sim. */
class VisionIOPhotonSimTest {
  private static final double TOLERANCE_METERS = 0.15;

  // Forward-facing, slightly pitched up, so the test does not depend on the robot's mounting
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(0.2, 0.0, 0.5, new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));
  // Narrow enough at 3 m to see only the tag it points at
  private static final CameraModel NARROW = new CameraModel("Narrow", 10.0, 6.25, 1280, 800);

  private static Pose2d truePose;
  private static VisionIOPhotonSim wideIO;
  private static VisionIOPhotonSim narrowIO;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();

    // 3 m in front of a tag, facing it
    var tagPose = VisionConstants.aprilTagLayout.getTags().get(0).pose;
    truePose = tagPose.toPose2d().transformBy(new Transform2d(3.0, 0.0, Rotation2d.k180deg));
    wideIO = createIO("wide-camera", CameraModel.LIMELIGHT_3G, ROBOT_TO_CAMERA);

    // Aimed straight at the tag center
    double pitch = -Math.atan2(tagPose.getZ() - ROBOT_TO_CAMERA.getZ(), 3.0 - ROBOT_TO_CAMERA.getX());
    narrowIO =
        createIO(
            "narrow-camera",
            NARROW,
            new Transform3d(ROBOT_TO_CAMERA.getTranslation(), new Rotation3d(0.0, pitch, 0.0)));
  }

  private static VisionIOPhotonSim createIO(
      String name, CameraModel model, Transform3d robotToCamera) {
    return new VisionIOPhotonSim(
        new CameraConfig(name, model, robotToCamera, VisionConstants.roiTuning),
        VisionIOPhoton.Options.DEFAULT,
        VisionIOPhotonSim.SimOptions.DEFAULT,
        () -> truePose,
        truePose::getRotation);
  }

  /** Several tags in view: coprocessor MultiTag. */
  @Test
  void multiTagEstimatesTruePose() {
    var observations = run(wideIO);
    for (var observation : observations) {
      assertTrue(observation.tagCount() > 1, observation.toString());
    }
  }

  /** One tag in view: trig solve with the gyro heading. */
  @Test
  void singleTagEstimatesTruePose() {
    var observations = run(narrowIO);
    for (var observation : observations) {
      assertEquals(1, observation.tagCount(), observation.toString());
      assertEquals(0.0, observation.ambiguity(), "expected trig solve: " + observation);
    }
  }

  private static List<VisionIO.PoseObservation> run(VisionIOPhotonSim io) {
    var inputs = new VisionIOInputsAutoLogged();
    List<VisionIO.PoseObservation> observations = new ArrayList<>();
    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(0.02);
      io.updateInputs(inputs);
      observations.addAll(List.of(inputs.poseObservations));
    }

    assertFalse(observations.isEmpty(), "camera never produced a pose");
    for (var observation : observations) {
      double error =
          observation.pose().toPose2d().getTranslation().getDistance(truePose.getTranslation());
      assertTrue(error < TOLERANCE_METERS, "pose error " + error + " m: " + observation);
      // Nearest tag is the one 3 m ahead (camera sits 0.2 m forward of the robot center)
      assertEquals(2.85, observation.minimumTagDistance(), 0.15, observation.toString());
    }
    return observations;
  }
}
