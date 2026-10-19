package frc.robot.power;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.io.MotorIO.Mode;
import frc.lib.util.TunableNumber;
import frc.robot.DashboardOverrides;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Shares the battery between mechanisms by changing current limits with what the robot is doing.
 * The shooter has priority (it must hold speed through a volley); the drive gives way, except in
 * defense mode.
 *
 * <p>Pick the mode on the dashboard ("Power/Mode": Auto, Normal or Defense), or flip Overrides/Defense
 * Mode. In Auto it follows the robot: shooting uses SCORING, a sagging battery LOW_BATTERY, otherwise
 * NORMAL. Limits are sent only when the profile changes (a profile must hold 0.5 s first, so it
 * doesn't flap). Overrides/Disable Power Manager pins NORMAL, the static constants values.
 */
public class PowerManager {
  public static final PowerManager mInstance = new PowerManager();

  /** Current limits in amps. Drive: per drive motor. Mechanisms: their main motor's limit. */
  public enum Profile {
    NORMAL(120, 60, 30, 30, 40),
    SCORING(120, 40, 40, 30, 25),
    DEFENSE(120, 80, 20, 20, 25),
    LOW_BATTERY(80, 35, 30, 20, 20);

    final TunableNumber driveStator;
    final TunableNumber driveSupply;
    final TunableNumber shooter;
    final TunableNumber feeder;
    final TunableNumber intakeRollers;

    Profile(double driveStator, double driveSupply, double shooter, double feeder, double intakeRollers) {
      String prefix = "Power/" + name() + "/";
      this.driveStator = new TunableNumber(prefix + "DriveStatorAmps", driveStator);
      this.driveSupply = new TunableNumber(prefix + "DriveSupplyAmps", driveSupply);
      this.shooter = new TunableNumber(prefix + "ShooterAmps", shooter);
      this.feeder = new TunableNumber(prefix + "FeederAmps", feeder);
      this.intakeRollers = new TunableNumber(prefix + "IntakeRollersAmps", intakeRollers);
    }
  }

  public enum ModeSelection {
    AUTO,
    NORMAL,
    DEFENSE
  }

  private static final TunableNumber lowBatteryVolts = new TunableNumber("Power/LowBatteryVolts", 9.0);
  private static final TunableNumber recoveredBatteryVolts =
      new TunableNumber("Power/RecoveredBatteryVolts", 10.0);
  private static final double MIN_PROFILE_HOLD_SEC = 0.5;

  private final LoggedDashboardChooser<ModeSelection> modeChooser =
      new LoggedDashboardChooser<>("Power/Mode");
  private final LinearFilter voltageFilter = LinearFilter.movingAverage(25); // 0.5 s
  private final Debouncer lowBatteryDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private final Alert lowBatteryAlert =
      new Alert("Battery sagging: drive and intake current limited", AlertType.kWarning);
  private final Timer sinceChange = new Timer();

  private boolean lowBattery = false;
  private Profile applied = null;

  private PowerManager() {
    modeChooser.addDefaultOption("Auto", ModeSelection.AUTO);
    modeChooser.addOption("Normal", ModeSelection.NORMAL);
    modeChooser.addOption("Defense", ModeSelection.DEFENSE);
    sinceChange.start();
  }

  /**
   * Which profile to use. Pure, so it can be tested.
   *
   * @param shooting the shooter is spinning up or shooting
   */
  public static Profile select(
      ModeSelection mode,
      boolean defenseSwitch,
      boolean managerDisabled,
      boolean lowBattery,
      boolean shooting) {
    if (managerDisabled) {
      return Profile.NORMAL;
    }
    if (defenseSwitch || mode == ModeSelection.DEFENSE) {
      return Profile.DEFENSE;
    }
    if (mode == ModeSelection.NORMAL) {
      return Profile.NORMAL;
    }
    if (lowBattery) {
      return Profile.LOW_BATTERY;
    }
    return shooting ? Profile.SCORING : Profile.NORMAL;
  }

  public void periodic() {
    double voltage = voltageFilter.calculate(RobotController.getBatteryVoltage());
    // Hysteresis: enter below lowBatteryVolts (for 0.5 s), leave above recoveredBatteryVolts
    boolean below = lowBatteryDebouncer.calculate(voltage < lowBatteryVolts.getAsDouble());
    if (below) {
      lowBattery = true;
    } else if (voltage > recoveredBatteryVolts.getAsDouble()) {
      lowBattery = false;
    }

    ModeSelection mode = modeChooser.get() == null ? ModeSelection.AUTO : modeChooser.get();
    boolean shooting = Shooter.mInstance.getSetpoint().mode == Mode.VELOCITY;
    Profile wanted =
        select(
            mode,
            DashboardOverrides.DEFENSE_MODE.get(),
            DashboardOverrides.DISABLE_POWER_MANAGER.get(),
            lowBattery,
            shooting);

    boolean tuned = applied != null && profileTuned(applied);
    if (applied == null
        || tuned
        || (wanted != applied && sinceChange.hasElapsed(MIN_PROFILE_HOLD_SEC))) {
      apply(wanted);
    }

    lowBatteryAlert.set(applied == Profile.LOW_BATTERY);
    Logger.recordOutput("Power/Profile", applied);
    Logger.recordOutput("Power/WantedProfile", wanted);
    Logger.recordOutput("Power/FilteredBatteryVolts", voltage);
    Logger.recordOutput("Power/LowBattery", lowBattery);
  }

  public Profile getProfile() {
    return applied;
  }

  private static boolean profileTuned(Profile profile) {
    // hasChanged() must run on every number so each one resets
    boolean changed = profile.driveStator.hasChanged();
    changed |= profile.driveSupply.hasChanged();
    changed |= profile.shooter.hasChanged();
    changed |= profile.feeder.hasChanged();
    changed |= profile.intakeRollers.hasChanged();
    return changed;
  }

  private void apply(Profile profile) {
    Drive.mInstance.setDriveCurrentLimits(
        profile.driveStator.getAsDouble(), profile.driveSupply.getAsDouble());
    Shooter.mInstance.setCurrentLimit(profile.shooter.getAsDouble());
    Feeder.mInstance.setCurrentLimit(profile.feeder.getAsDouble());
    IntakeRollers.mInstance.setCurrentLimit(profile.intakeRollers.getAsDouble());
    applied = profile;
    sinceChange.restart();
  }
}
