package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.TunableNumber;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks which hub is active during a REBUILT match.
 *
 * <p>Teleop (140 s) is a 10 s transition with both hubs active, four 25 s alliance shifts where only
 * one hub is active, and a 30 s end game with both active. After auto the FMS game data is the
 * alliance ('R' or 'B') whose hub is inactive in shift 1; shifts then alternate.
 *
 * <p>Fallbacks: without FMS match time (-1) a local timer started at teleop init is used, and with
 * missing or invalid game data our hub is assumed active and a warning is raised.
 */
public class HubShiftTracker {
  public static final HubShiftTracker mInstance = new HubShiftTracker();

  public static final double TELEOP_LENGTH = 140.0;
  public static final double TRANSITION_END = 130.0; // teleop time remaining
  public static final double SHIFT_LENGTH = 25.0;
  public static final double END_GAME_START = 30.0;

  /**
   * Feeding is allowed when our hub will be active this long from now, about the fuel's flight time,
   * so fuel lands while the hub counts it. Also stops feeding this long before our hub turns off.
   */
  private static final TunableNumber scoreLeadSeconds = new TunableNumber("HubShift/ScoreLeadSeconds", 0.5);

  public enum Phase {
    DISABLED,
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME
  }

  /**
   * @param ourHubActive whether our hub counts fuel right now
   * @param secondsUntilChange time until our hub's active state next changes (infinite if it won't)
   * @param gameDataValid false when the FMS hasn't said which hub goes inactive first
   */
  public record State(
      Phase phase, boolean ourHubActive, double secondsUntilChange, boolean gameDataValid) {}

  private final Alert gameDataAlert =
      new Alert("Hub shift: no FMS game data, assuming our hub is always active", AlertType.kWarning);
  private final Timer teleopTimer = new Timer();
  private State state = new State(Phase.DISABLED, true, Double.POSITIVE_INFINITY, false);
  private boolean canScore = true;

  private HubShiftTracker() {}

  public void onTeleopInit() {
    teleopTimer.restart();
  }

  public void periodic() {
    boolean teleop = DriverStation.isTeleopEnabled();
    double matchTime = DriverStation.getMatchTime();
    if (teleop && matchTime < 0) {
      matchTime = Math.max(0.0, TELEOP_LENGTH - teleopTimer.get());
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();

    state =
        compute(
            DriverStation.isAutonomousEnabled(),
            teleop,
            matchTime,
            gameData,
            alliance.orElse(Alliance.Blue));
    double lead = scoreLeadSeconds.getAsDouble();
    canScore =
        DashboardOverrides.HUB_ALWAYS_ACTIVE.get()
            || (teleop
                ? compute(false, true, matchTime - lead, gameData, alliance.orElse(Alliance.Blue))
                    .ourHubActive()
                : state.ourHubActive());

    gameDataAlert.set(teleop && !state.gameDataValid());

    Logger.recordOutput("HubShift/Phase", state.phase());
    Logger.recordOutput("HubShift/OurHubActive", state.ourHubActive());
    Logger.recordOutput("HubShift/SecondsUntilChange", state.secondsUntilChange());
    Logger.recordOutput("HubShift/GameDataValid", state.gameDataValid());
    Logger.recordOutput("HubShift/CanScore", canScore);
    SmartDashboard.putBoolean("HubShift/Our Hub Active", state.ourHubActive());
    SmartDashboard.putNumber(
        "HubShift/Seconds Until Change",
        Double.isInfinite(state.secondsUntilChange()) ? -1.0 : state.secondsUntilChange());
    SmartDashboard.putString("HubShift/Phase", state.phase().name());
  }

  public State getState() {
    return state;
  }

  public boolean isOurHubActive() {
    return state.ourHubActive();
  }

  /** Whether fuel fed now would score: our hub is active when it lands, or Hub Always Active is on. */
  public boolean canScore() {
    return canScore;
  }

  /**
   * The hub state for a moment in the match. Pure, so it can be tested without a Driver Station.
   *
   * @param teleopTimeRemaining teleop seconds remaining (the DS match time during teleop)
   * @param gameData FMS game data: the alliance whose hub is inactive in shift 1
   */
  public static State compute(
      boolean auto, boolean teleop, double teleopTimeRemaining, String gameData, Alliance ourAlliance) {
    if (auto) {
      return new State(Phase.AUTO, true, Double.POSITIVE_INFINITY, true);
    }
    if (!teleop) {
      return new State(Phase.DISABLED, true, Double.POSITIVE_INFINITY, true);
    }

    Optional<Alliance> inactiveFirst = parseGameData(gameData);
    double t = Math.max(0.0, teleopTimeRemaining);
    Phase phase = phaseAt(t);
    if (inactiveFirst.isEmpty()) {
      return new State(phase, true, Double.POSITIVE_INFINITY, false);
    }
    boolean weStartInactive = inactiveFirst.get() == ourAlliance;

    boolean active = activeAt(phase, weStartInactive);
    // Walk forward through the remaining phase boundaries until our state flips
    double secondsUntilChange = Double.POSITIVE_INFINITY;
    for (double boundary : phaseBoundaries()) {
      if (boundary < t && activeAt(phaseAt(boundary - 1e-6), weStartInactive) != active) {
        secondsUntilChange = t - boundary;
        break;
      }
    }
    return new State(phase, active, secondsUntilChange, true);
  }

  private static double[] phaseBoundaries() {
    return new double[] {
      TRANSITION_END,
      TRANSITION_END - SHIFT_LENGTH,
      TRANSITION_END - 2 * SHIFT_LENGTH,
      TRANSITION_END - 3 * SHIFT_LENGTH,
      END_GAME_START
    };
  }

  private static Phase phaseAt(double t) {
    if (t > TRANSITION_END) return Phase.TRANSITION;
    if (t > TRANSITION_END - SHIFT_LENGTH) return Phase.SHIFT_1;
    if (t > TRANSITION_END - 2 * SHIFT_LENGTH) return Phase.SHIFT_2;
    if (t > TRANSITION_END - 3 * SHIFT_LENGTH) return Phase.SHIFT_3;
    if (t > END_GAME_START) return Phase.SHIFT_4;
    return Phase.END_GAME;
  }

  private static boolean activeAt(Phase phase, boolean weStartInactive) {
    return switch (phase) {
      case SHIFT_1, SHIFT_3 -> !weStartInactive;
      case SHIFT_2, SHIFT_4 -> weStartInactive;
      default -> true;
    };
  }

  private static Optional<Alliance> parseGameData(String gameData) {
    if (gameData == null || gameData.isEmpty()) {
      return Optional.empty();
    }
    return switch (Character.toUpperCase(gameData.charAt(0))) {
      case 'R' -> Optional.of(Alliance.Red);
      case 'B' -> Optional.of(Alliance.Blue);
      default -> Optional.empty();
    };
  }
}
