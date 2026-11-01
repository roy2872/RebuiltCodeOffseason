package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Builds the auto routine from dashboard dropdowns: a start position, then Part 1..N, each one of the
 * registered actions. "None" ends the routine. A Preset dropdown can pick a whole hand-written
 * routine instead.
 *
 * <p>The routine is rebuilt only while disabled and only when a selection or the alliance changes,
 * so nothing is built at the start of auto. Problems show as an Alert, and the planned route is
 * logged to {@code Auto/PreviewPoses} for AdvantageScope.
 *
 * <p>The choosers are logged, so replay rebuilds the same routine.
 */
public class ModularAutoChooser {
  public static final String NONE = "None";
  public static final String BUILD_FROM_PARTS = "Build From Parts";

  private final DriveAdapter drive;
  private final UnaryOperator<Pose2d> flip;
  private final Supplier<Command> ending;
  private final Map<String, AutoStartPosition> starts = new LinkedHashMap<>();
  private final Map<String, AutoAction> actions = new LinkedHashMap<>();
  private final Map<String, Supplier<Command>> presets = new LinkedHashMap<>();

  private final LoggedDashboardChooser<String> presetChooser;
  private final LoggedDashboardChooser<String> startChooser;
  private final List<LoggedDashboardChooser<String>> partChoosers = new ArrayList<>();

  private final Alert warningAlert = new Alert("", AlertType.kWarning);
  private String builtKey = null;
  private Command routine = null;

  /**
   * @param partCount number of Part dropdowns
   * @param flip turns a blue-alliance pose into a pose for the current alliance
   * @param ending runs after the last part (e.g. stop all mechanisms)
   */
  public ModularAutoChooser(
      int partCount,
      List<AutoStartPosition> startPositions,
      List<AutoAction> actionList,
      Map<String, Supplier<Command>> presetRoutines,
      DriveAdapter drive,
      UnaryOperator<Pose2d> flip,
      Supplier<Command> ending) {
    this.drive = drive;
    this.flip = flip;
    this.ending = ending;

    presetChooser = new LoggedDashboardChooser<>("Auto/Preset");
    presetChooser.addDefaultOption(BUILD_FROM_PARTS, BUILD_FROM_PARTS);
    presets.putAll(presetRoutines);
    for (String name : presets.keySet()) {
      presetChooser.addOption(name, name);
    }

    startChooser = new LoggedDashboardChooser<>("Auto/Start");
    for (int i = 0; i < startPositions.size(); i++) {
      AutoStartPosition start = startPositions.get(i);
      starts.put(start.name(), start);
      if (i == 0) {
        startChooser.addDefaultOption(start.name(), start.name());
      } else {
        startChooser.addOption(start.name(), start.name());
      }
    }

    for (AutoAction action : actionList) {
      actions.put(action.name(), action);
    }
    for (int i = 1; i <= partCount; i++) {
      var chooser = new LoggedDashboardChooser<String>("Auto/Part " + i);
      chooser.addDefaultOption(NONE, NONE);
      for (String name : actions.keySet()) {
        chooser.addOption(name, name);
      }
      partChoosers.add(chooser);
    }
  }

  /** Rebuilds the routine when the selection changes. Call every loop. */
  public void periodic() {
    if (DriverStation.isDisabled() && !selectionKey().equals(builtKey)) {
      rebuild();
    }
  }

  /** The selected routine, built ahead of time while disabled. */
  public Command getRoutine() {
    if (routine == null || !selectionKey().equals(builtKey)) {
      rebuild();
    }
    return routine;
  }

  private String selectionKey() {
    StringBuilder key = new StringBuilder();
    key.append(presetChooser.get()).append('|').append(startChooser.get());
    for (var chooser : partChoosers) {
      key.append('|').append(chooser.get());
    }
    key.append('|').append(DriverStation.getAlliance().map(Enum::name).orElse("NoAlliance"));
    return key.toString();
  }

  private void rebuild() {
    builtKey = selectionKey();
    List<String> warnings = new ArrayList<>();
    List<Pose2d> preview = new ArrayList<>();
    double estimate = 0.0;

    String preset = presetChooser.get();
    if (preset != null && presets.containsKey(preset)) {
      routine = presets.get(preset).get();
    } else {
      AutoStartPosition start = starts.get(startChooser.get());
      List<AutoAction> parts = new ArrayList<>();
      boolean ended = false;
      for (int i = 0; i < partChoosers.size(); i++) {
        String selected = partChoosers.get(i).get();
        AutoAction action = selected == null ? null : actions.get(selected);
        if (action == null) {
          ended = true;
        } else if (ended) {
          warnings.add("Part " + (i + 1) + " is ignored: an earlier part is None");
        } else {
          parts.add(action);
        }
      }
      if (start == null) {
        warnings.add("No start position selected");
        routine = drive.stop().withName("Auto: no start position");
      } else {
        var built = AutoRoutineBuilder.build(start, parts, drive, flip, ending);
        routine = built.command();
        preview = built.previewPoses();
        estimate = built.estimatedSeconds();
        warnings.addAll(built.warnings());
      }
    }

    warningAlert.setText("Auto: " + String.join("; ", warnings));
    warningAlert.set(!warnings.isEmpty());
    Logger.recordOutput("Auto/PreviewPoses", preview.toArray(new Pose2d[0]));
    Logger.recordOutput("Auto/EstimatedSeconds", estimate);
    Logger.recordOutput("Auto/Selected", routine.getName());
    Logger.recordOutput("Auto/Warnings", warnings.toArray(new String[0]));
  }
}
