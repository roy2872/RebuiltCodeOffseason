package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Every driver/operator override as a dashboard switch, under /SmartDashboard/Overrides/.
 *
 * <p>The switches are {@link LoggedNetworkBoolean}s, so their values are logged and replay sees the
 * same values the robot used. Any switch that changes match behavior raises a warning {@link Alert}
 * while it is on, so nobody plays a match with one left on by accident.
 */
public final class DashboardOverrides {
  private static final String PREFIX = "/SmartDashboard/Overrides/";
  private static final List<OverrideSwitch> ALL = new ArrayList<>();

  /** Score even while our hub is inactive (skips the hub-shift feed lockout). */
  public static final OverrideSwitch HUB_ALWAYS_ACTIVE = new OverrideSwitch("Hub Always Active", false, true);
  /** Odometry only. Vision is still read and logged but not fused into the pose estimate. */
  public static final OverrideSwitch DISABLE_VISION = new OverrideSwitch("Disable Vision", false, true);
  /** Vision is unreliable: hub shots use the fixed close-shot preset instead of the table. */
  public static final OverrideSwitch VISION_FALLBACK_PRESETS =
      new OverrideSwitch("Vision Fallback Presets", false, true);
  /** Hub shots use a fixed flywheel speed trimmed by FlywheelBias instead of the shooting table. */
  public static final OverrideSwitch MANUAL_SHOOTING = new OverrideSwitch("Manual Shooting", true, false);
  /** The shoot button ferries instead of scoring in the hub. */
  public static final OverrideSwitch FORCE_FERRY = new OverrideSwitch("Force Ferry Mode", false, true);
  /** Driver translation is robot relative instead of field relative. */
  public static final OverrideSwitch ROBOT_RELATIVE = new OverrideSwitch("Robot Relative Drive", false, true);
  /** Shooting does not turn the robot toward the target; the driver aims and feeding doesn't wait on heading. */
  public static final OverrideSwitch DISABLE_SHOOT_AUTO_ALIGN =
      new OverrideSwitch("Disable Shoot Auto Align", false, true);

  // One-shot buttons: they turn themselves back off once their command starts
  public static final OverrideSwitch INTAKE_FORCE_HOME = new OverrideSwitch("Intake Force Home", false, false);
  public static final OverrideSwitch HOOD_FORCE_HOME = new OverrideSwitch("Hood Force Home", false, false);
  /** Runs the pit self test (robot must be enabled; Test mode runs it on its own). */
  public static final OverrideSwitch RUN_SELF_TEST = new OverrideSwitch("Run Self Test", false, false);

  private DashboardOverrides() {}

  /** Updates the override alerts. Call once per loop. */
  public static void periodic() {
    for (OverrideSwitch overrideSwitch : ALL) {
      overrideSwitch.updateAlert();
    }
  }

  /**
   * Runs {@code command} each time {@code button} is switched on, then switches it back off. A logged
   * dashboard toggle rather than a command button, which replay would never see pressed.
   */
  public static void bindOneShot(OverrideSwitch button, Command command) {
    new Trigger(button::get)
        .onTrue(Commands.runOnce(() -> button.set(false)).ignoringDisable(true).andThen(command));
  }

  public static final class OverrideSwitch {
    private final String name;
    private final LoggedNetworkBoolean entry;
    private final Alert alert; // null when this switch doesn't warn

    private OverrideSwitch(String name, boolean defaultValue, boolean warnWhenOn) {
      this.name = name;
      this.entry = new LoggedNetworkBoolean(PREFIX + name, defaultValue);
      this.alert = warnWhenOn ? new Alert("Override active: " + name, AlertType.kWarning) : null;
      ALL.add(this);
    }

    /** The value as of the start of this loop. */
    public boolean get() {
      return entry.get();
    }

    /** Sets the value. Like a dashboard edit, {@link #get()} sees it starting next loop. */
    public void set(boolean value) {
      entry.set(value);
    }

    /** Sets the value and makes {@link #get()} see it right away (for tests and one-off setup). */
    public void setNow(boolean value) {
      entry.set(value);
      entry.periodic();
    }

    public String getName() {
      return name;
    }

    private void updateAlert() {
      if (alert != null) {
        alert.set(get());
      }
    }
  }
}
