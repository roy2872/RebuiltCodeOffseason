package frc.lib.logging;

import java.util.Map;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * An {@link NT4Publisher} that stops publishing chosen output keys while the robot is connected to
 * the FMS, to save field bandwidth. The keys are still saved by the WPILOGWriter, so nothing is lost
 * for post-match review.
 *
 * <p>The FMS flag is read from the logged DriverStation data every loop instead of once at boot,
 * since the FMS usually connects after the robot code has started.
 */
public class FilteredNT4Publisher implements LogDataReceiver {
  private final NT4Publisher publisher = new NT4Publisher();
  private final String[] hiddenPrefixes;

  /**
   * Creates a publisher. With no keys, nothing from AdvantageKit is published while on the field.
   *
   * @param outputKeysHiddenOnField Output keys (as passed to Logger.recordOutput) to stop publishing
   *     while the FMS is attached. Each also hides everything below it.
   */
  public FilteredNT4Publisher(String... outputKeysHiddenOnField) {
    hiddenPrefixes = new String[outputKeysHiddenOnField.length * 2];
    for (int i = 0; i < outputKeysHiddenOnField.length; i++) {
      hiddenPrefixes[i * 2] = "/RealOutputs/" + outputKeysHiddenOnField[i];
      hiddenPrefixes[i * 2 + 1] = "/ReplayOutputs/" + outputKeysHiddenOnField[i];
    }
  }

  @Override
  public void start() {
    publisher.start();
  }

  @Override
  public void end() {
    publisher.end();
  }

  @Override
  public void putTable(LogTable table) {
    if (!table.getSubtable("DriverStation").get("FMSAttached", false)) {
      publisher.putTable(table);
      return;
    }
    if (hiddenPrefixes.length == 0) {
      return;
    }

    LogTable filtered = new LogTable(table.getTimestamp());
    for (Map.Entry<String, LogValue> field : table.getAll(false).entrySet()) {
      if (!isHidden(field.getKey())) {
        filtered.put(field.getKey().substring(1), field.getValue());
      }
    }
    publisher.putTable(filtered);
  }

  private boolean isHidden(String key) {
    for (String prefix : hiddenPrefixes) {
      if (key.startsWith(prefix)) {
        return true;
      }
    }
    return false;
  }
}
