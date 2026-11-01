package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

/**
 * Everything the auto framework needs from a drivetrain. Implement this once per robot; the rest of
 * {@code frc.lib.auto} has no robot or game specific code.
 *
 * <p>Poses passed to and returned from the adapter are field poses for the current alliance (already
 * flipped), except where noted.
 */
public interface DriveAdapter {
  /** Drives to {@code target}, avoiding field obstacles. Ends when it gets there. */
  Command driveTo(Pose2d target);

  /**
   * Drives to the start of a pre-drawn path, then follows it. The adapter flips the drawn
   * (blue-alliance) path for the current alliance.
   */
  Command followPath(String pathName);

  /** The poses along a pre-drawn path, blue alliance, for previews and estimates. Empty if missing. */
  List<Pose2d> pathPoses(String pathName);

  /** Tells the robot where it is (start of auto). */
  void resetPose(Pose2d pose);

  /** Stops driving. */
  Command stop();

  /** Rough time to drive between two poses, for the routine time estimate. */
  double estimateDriveSeconds(Pose2d from, Pose2d to);
}
