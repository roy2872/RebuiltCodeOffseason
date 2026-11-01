package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionRoiController.Capabilities;
import frc.robot.subsystems.vision.VisionRoiController.CropWindow;
import frc.robot.subsystems.vision.VisionRoiController.Frame;
import frc.robot.subsystems.vision.VisionRoiController.Output;
import frc.robot.subsystems.vision.VisionRoiController.RobotMotion;
import frc.robot.subsystems.vision.VisionRoiController.State;
import frc.robot.subsystems.vision.VisionRoiController.TagSample;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionRoiControllerTest {
  private static final VisionRoiController.Tuning TUNING =
      new VisionRoiController.Tuning(
          0.1651, 0.5, 0.12, 0.6, 0.03, 10.0, 20.0, 0.3, 0.1, Math.toRadians(2.0), 0.35, 0.3, 1.0,
          20.0, 1.3, 1.0, 4.0, 0.97, 4.0, 1.5, 0.5);
  private static final Capabilities LIMELIGHT =
      new Capabilities(true, new double[] {1.0, 1.5, 2.0, 3.0, 4.0}, 0.0);
  private static final RobotMotion STILL = new RobotMotion(0.0, 0.0, 0.0, 0.0);

  private static VisionRoiController controller(Capabilities capabilities) {
    // Forward-facing camera at the robot center
    return new VisionRoiController(
        CameraModel.LIMELIGHT_3G, new Transform3d(), TUNING, capabilities);
  }

  /** A 6.5 in tag seen head-on at this distance covers about this percent of a 1280x800 image. */
  private static double areaPercentAt(double distanceMeters) {
    CameraModel model = CameraModel.LIMELIGHT_3G;
    double pixels =
        0.1651 / distanceMeters / model.tanHalfHorizontalFov() * model.widthPixels() / 2.0;
    return pixels * pixels / (model.widthPixels() * model.heightPixels()) * 100.0;
  }

  private static Frame frame(double timestamp, double yawDeg, double pitchDeg, double distance) {
    return new Frame(
        timestamp,
        List.of(new TagSample(7, yawDeg, pitchDeg, areaPercentAt(distance), distance)));
  }

  private static void assertInsideImage(CropWindow crop) {
    assertTrue(crop.xMin() >= -1.0 && crop.xMax() <= 1.0, "x out of image: " + crop);
    assertTrue(crop.yMin() >= -1.0 && crop.yMax() <= 1.0, "y out of image: " + crop);
    assertTrue(crop.xMin() < crop.xMax() && crop.yMin() < crop.yMax(), "empty crop: " + crop);
  }

  @Test
  void searchesFullFrameAtFullResolutionWithNoTags() {
    Output out = controller(LIMELIGHT).update(List.of(), 0.0, STILL);
    assertEquals(State.SEARCHING, out.state());
    assertTrue(out.crop().isFull());
    assertEquals(1.0, out.downscale());
  }

  @Test
  void cropsTightlyAroundStationaryTag() {
    Output out = controller(LIMELIGHT).update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, STILL);
    assertEquals(State.TRACKING, out.state());
    assertTrue(out.crop().contains(0.0, 0.0));
    assertTrue(out.crop().areaFraction() < 0.25, "crop too big: " + out.crop());
    assertInsideImage(out.crop());
  }

  @Test
  void leadsTheCropWhenTheRobotTurns() {
    // Turning left (CCW) moves tags in front of the camera to the right of the image.
    RobotMotion turningLeft = new RobotMotion(0.0, 0.0, 3.0, 3.0);
    Output out =
        controller(LIMELIGHT).update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, turningLeft);
    double centerX = (out.crop().xMin() + out.crop().xMax()) / 2.0;
    assertTrue(centerX > 0.05, "crop should lead to the right, center x = " + centerX);
  }

  @Test
  void returnsToFullFrameWhenTagIsLostForTooLong() {
    VisionRoiController roi = controller(LIMELIGHT);
    roi.update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, STILL);
    Output coasting = roi.update(List.of(new Frame(0.08, List.of())), 0.1, STILL);
    assertEquals(State.TRACKING, coasting.state());
    Output lost = roi.update(List.of(), 0.5, STILL);
    assertEquals(State.SEARCHING, lost.state());
    assertTrue(lost.crop().isFull());
  }

  @Test
  void growsTheCropWhileCoasting() {
    VisionRoiController roi = controller(LIMELIGHT);
    Output seen = roi.update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, STILL);
    Output missed =
        roi.update(
            List.of(new Frame(0.03, List.of()), new Frame(0.06, List.of())), 0.08, STILL);
    assertTrue(missed.crop().areaFraction() > seen.crop().areaFraction());
  }

  @Test
  void goesFullFrameWhenWheelsSlip() {
    // Gyro says the robot spins, wheels say it doesn't: it's being pushed.
    RobotMotion pushed = new RobotMotion(0.0, 0.0, 3.0, 0.0);
    Output out = controller(LIMELIGHT).update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, pushed);
    assertEquals(State.DISTURBED, out.state());
    assertTrue(out.crop().isFull());
  }

  @Test
  void goesFullFrameWhenTagJumpsAwayFromPrediction() {
    VisionRoiController roi = controller(LIMELIGHT);
    roi.update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, STILL);
    // Sensors say the robot is still, but the tag moved 15 degrees in 25 ms: the robot was hit.
    Output out = roi.update(List.of(frame(0.025, 15.0, 0.0, 3.0)), 0.06, STILL);
    assertEquals(State.DISTURBED, out.state());
    assertTrue(out.crop().isFull());
    assertTrue(out.adaptiveScale() > 1.0);
  }

  @Test
  void downscalesCloseTagsButNotFarOnes() {
    Output close = controller(LIMELIGHT).update(List.of(frame(0.0, 0.0, 0.0, 0.8)), 0.04, STILL);
    assertTrue(close.downscale() > 1.0, "close tag should downscale, got " + close.downscale());

    Output far = controller(LIMELIGHT).update(List.of(frame(0.0, 0.0, 0.0, 7.0)), 0.04, STILL);
    assertEquals(1.0, far.downscale());
  }

  @Test
  void keepsCropInsideImageForTagNearTheEdge() {
    Output out = controller(LIMELIGHT).update(List.of(frame(0.0, 39.0, 25.0, 2.0)), 0.04, STILL);
    assertEquals(State.TRACKING, out.state());
    assertInsideImage(out.crop());
  }

  @Test
  void coversEveryVisibleTagForMultiTagSolves() {
    Frame twoTags =
        new Frame(
            0.0,
            List.of(
                new TagSample(1, -15.0, 0.0, areaPercentAt(3.0), 3.0),
                new TagSample(2, 15.0, 5.0, areaPercentAt(3.0), 3.0)));
    Output out = controller(LIMELIGHT).update(List.of(twoTags), 0.04, STILL);
    double x1 = -Math.tan(Math.toRadians(15.0)) / CameraModel.LIMELIGHT_3G.tanHalfHorizontalFov();
    assertTrue(out.crop().contains(x1, 0.0));
    assertTrue(out.crop().contains(-x1, 0.0));
  }

  @Test
  void periodicallyTakesAFullFrame() {
    VisionRoiController roi = controller(LIMELIGHT);
    roi.update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, STILL);
    Output out = null;
    for (double t = 0.06; t < 1.2; t += 0.02) {
      out = roi.update(List.of(frame(t - 0.03, 0.0, 0.0, 3.0)), t, STILL);
      if (out.state() == State.REFRESHING) break;
    }
    assertEquals(State.REFRESHING, out.state());
    assertTrue(out.crop().isFull());
  }

  @Test
  void neverCropsWhenCameraCannotCrop() {
    Capabilities noCrop = new Capabilities(false, new double[] {1.0, 2.0}, 0.0);
    Output out = controller(noCrop).update(List.of(frame(0.0, 0.0, 0.0, 3.0)), 0.04, STILL);
    assertEquals(State.TRACKING, out.state());
    assertTrue(out.crop().isFull());
    assertFalse(out.downscale() > 2.0);
  }
}
