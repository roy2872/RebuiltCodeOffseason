package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/** Turns a start position and a list of parts into one auto command, with a preview and checks. */
public final class AutoRoutineBuilder {
  /** FRC auto length; routines estimated longer than this get a warning. */
  public static final double AUTO_LENGTH_SEC = 20.0;

  private AutoRoutineBuilder() {}

  /**
   * A built routine.
   *
   * @param previewPoses start pose and each part's end pose, for the current alliance
   * @param estimatedSeconds rough driving plus action time
   * @param warnings problems to show the drive team before the match
   */
  public record Routine(
      Command command, List<Pose2d> previewPoses, double estimatedSeconds, List<String> warnings) {}

  /**
   * @param flip turns a blue-alliance pose into a pose for the current alliance
   * @param ending runs after the last part (e.g. stop all mechanisms)
   */
  public static Routine build(
      AutoStartPosition start,
      List<AutoAction> parts,
      DriveAdapter drive,
      UnaryOperator<Pose2d> flip,
      Supplier<Command> ending) {
    List<Command> commands = new ArrayList<>();
    List<Pose2d> preview = new ArrayList<>();
    List<String> warnings = new ArrayList<>();

    Pose2d startPose = flip.apply(start.bluePose());
    commands.add(Commands.runOnce(() -> drive.resetPose(startPose)).withName("Auto: reset pose"));
    preview.add(startPose);

    double seconds = 0.0;
    Pose2d bluePose = start.bluePose();
    for (int i = 0; i < parts.size(); i++) {
      AutoAction part = parts.get(i);
      commands.add(part.toCommand(drive, flip));

      // Time estimate: drive to the path start or target, along the path, then the action
      double partSeconds = 0.0;
      if (part.pathName().isPresent()) {
        List<Pose2d> pathPoses = drive.pathPoses(part.pathName().get());
        if (pathPoses.isEmpty()) {
          warnings.add("Part " + (i + 1) + " (" + part.name() + "): path '" + part.pathName().get() + "' not found");
        } else {
          partSeconds += drive.estimateDriveSeconds(bluePose, pathPoses.get(0));
          for (int p = 1; p < pathPoses.size(); p++) {
            partSeconds += drive.estimateDriveSeconds(pathPoses.get(p - 1), pathPoses.get(p));
          }
        }
      } else if (part.blueTarget().isPresent()) {
        partSeconds += drive.estimateDriveSeconds(bluePose, part.blueTarget().get());
      }
      partSeconds += part.atTargetSeconds();
      if (part.timeoutSec() > 0.0) {
        partSeconds = Math.min(partSeconds, part.timeoutSec());
      }
      seconds += partSeconds;

      var end = part.blueEndPose(drive);
      if (end.isPresent()) {
        bluePose = end.get();
        preview.add(flip.apply(bluePose));
      }
    }
    commands.add(ending.get());

    if (seconds > AUTO_LENGTH_SEC) {
      warnings.add(
          String.format("Routine is about %.1f s, longer than the %.0f s auto", seconds, AUTO_LENGTH_SEC));
    }
    Command command =
        Commands.sequence(commands.toArray(new Command[0])).withName("Auto: " + start.name());
    return new Routine(command, preview, seconds, warnings);
  }
}
