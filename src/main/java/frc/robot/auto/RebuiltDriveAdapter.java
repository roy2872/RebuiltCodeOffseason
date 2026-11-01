package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.DriveAdapter;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The auto framework's view of this drivetrain: PathPlanner pathfinding inside a zone, and a bump
 * crossing ({@link BumpCrossing}) whenever the target is on the other side of a hub line.
 */
public class RebuiltDriveAdapter implements DriveAdapter {
  // For the time estimate only
  private static final double AVERAGE_SPEED = 2.0; // m/s, including acceleration
  private static final double SECONDS_PER_CROSSING = 3.5; // line up, turn, cross slowly

  private final Map<String, List<Pose2d>> pathPoseCache = new HashMap<>();

  @Override
  public Command driveTo(Pose2d target) {
    // Planned when it starts, from wherever the robot actually is then
    return Commands.defer(
            () -> {
              List<Command> steps = new ArrayList<>();
              for (var crossing :
                  BumpCrossing.plan(RobotState.mInstance.getEstimatedPose(), target)) {
                steps.add(BumpCrossing.command(crossing));
              }
              steps.add(AutoBuilder.pathfindToPose(target, BumpCrossing.PATHFIND_CONSTRAINTS));
              return Commands.sequence(steps.toArray(new Command[0]));
            },
            Set.of(Drive.mInstance))
        .withName("Drive To " + target);
  }

  @Override
  public Command followPath(String pathName) {
    return Commands.defer(
            () -> {
              PathPlannerPath path = load(pathName);
              if (path == null) {
                DriverStation.reportError("Auto path '" + pathName + "' not found", false);
                return Commands.none();
              }
              Pose2d start =
                  path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
              // driveTo first so the robot crosses a bump if the path starts in another zone
              return driveTo(AllianceFlipping.apply(start)).andThen(AutoBuilder.followPath(path));
            },
            Set.of(Drive.mInstance))
        .withName("Follow " + pathName);
  }

  /**
   * Drives straight from {@code from} to {@code to} at a constant heading, field coordinates. Used for
   * sweeps through fuel with the intake leading.
   */
  public Command driveStraight(Pose2d from, Pose2d to, Rotation2d heading, double maxSpeed) {
    return Commands.defer(
            () -> {
              Rotation2d travel = to.getTranslation().minus(from.getTranslation()).getAngle();
              PathPlannerPath path =
                  new PathPlannerPath(
                      PathPlannerPath.waypointsFromPoses(
                          new Pose2d(from.getTranslation(), travel),
                          new Pose2d(to.getTranslation(), travel)),
                      new PathConstraints(
                          maxSpeed, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                      new IdealStartingState(0.0, heading),
                      new GoalEndState(0.0, heading));
              path.preventFlipping = true;
              return AutoBuilder.pathfindToPose(
                      new Pose2d(from.getTranslation(), heading), BumpCrossing.PATHFIND_CONSTRAINTS)
                  .andThen(AutoBuilder.followPath(path));
            },
            Set.of(Drive.mInstance))
        .withName("Drive Straight");
  }

  @Override
  public List<Pose2d> pathPoses(String pathName) {
    return pathPoseCache.computeIfAbsent(
        pathName,
        name -> {
          PathPlannerPath path = load(name);
          return path == null ? List.of() : path.getPathPoses();
        });
  }

  private static PathPlannerPath load(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      return null;
    }
  }

  @Override
  public void resetPose(Pose2d pose) {
    // In sim, also put the simulated robot at the start, like placing the real robot on the field.
    // Otherwise the estimate says "start pose" while the sim robot is wherever disabledInit left
    // it, and since vision doesn't correct heading the whole routine runs mirrored.
    if (Constants.currentMode == Constants.Mode.SIM) {
      Drive.mInstance.driveSimulation.setSimulationWorldPose(pose);
    }
    RobotState.mInstance.resetPose(pose);
  }

  @Override
  public Command stop() {
    return Drive.mInstance.runOnce(Drive.mInstance::stop);
  }

  @Override
  public double estimateDriveSeconds(Pose2d from, Pose2d to) {
    int crossings = BumpCrossing.linesBetween(from.getX(), to.getX()).size();
    return from.getTranslation().getDistance(to.getTranslation()) / AVERAGE_SPEED
        + crossings * SECONDS_PER_CROSSING;
  }
}
