package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.TunableNumber;
import frc.robot.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Pit self test: moves every mechanism and checks that it gets where it should, in time. Run it in
 * Test mode (it starts on its own) or from the dashboard (SelfTest/Run) while enabled.
 *
 * <p>Each step reports PASSED, FAILED (with what it measured) or SKIPPED, as an Alert and in
 * SelfTest/Results; SelfTest/Passed is true when nothing failed. Disabling the robot stops it.
 *
 * <p>The swerve step spins the wheels, so it only runs with SelfTest/Wheels Off Ground switched on.
 *
 * <p>The vision steps check one camera at a time, so a person can hold a tag in front of each in
 * turn (SelfTest/VisionSecondsPerCamera each). They only run with SelfTest/Check Vision.
 *
 * <p>The fuel path step (SelfTest/Fuel Path, on by default) intakes one fuel and shoots it slowly.
 * Start with an empty hopper and set one fuel in front of the intake. Without a fuel sensor the robot
 * can't see the fuel, so on the real robot those steps report DONE and someone watches the fuel go
 * through; in simulation they check the fuel sim's count.
 */
public class SelfTest {
  // A stalled, unplugged or disconnected roller reads about 0
  private static final TunableNumber minRollerRps = new TunableNumber("SelfTest/MinRollerRps", 0.25);
  private static final TunableNumber shooterToleranceRps =
      new TunableNumber("SelfTest/ShooterToleranceRps", 2.0);
  private static final TunableNumber moduleAngleToleranceDeg =
      new TunableNumber("SelfTest/ModuleAngleToleranceDeg", 5.0);

  private static final TunableNumber visionSecondsPerCamera =
      new TunableNumber("SelfTest/VisionSecondsPerCamera", 5.0);
  private static final TunableNumber fuelIntakeSeconds = new TunableNumber("SelfTest/FuelIntakeSeconds", 4.0);
  private static final TunableNumber fuelShotSeconds = new TunableNumber("SelfTest/FuelShotSeconds", 3.0);

  private final Vision vision;
  private final Optional<IntSupplier> fuelHeld;
  private final LoggedNetworkBoolean wheelsOffGround =
      new LoggedNetworkBoolean("/SmartDashboard/SelfTest/Wheels Off Ground", false);
  private final LoggedNetworkBoolean checkVision =
      new LoggedNetworkBoolean("/SmartDashboard/SelfTest/Check Vision", false);
  private final LoggedNetworkBoolean fuelPath =
      new LoggedNetworkBoolean("/SmartDashboard/SelfTest/Fuel Path", true);
  // Pressed once the fuel is at the intake, so nobody's hand is there when the rollers start
  private final LoggedNetworkBoolean fuelPlaced =
      new LoggedNetworkBoolean("/SmartDashboard/SelfTest/Fuel Placed", false);
  private static final TunableNumber fuelPlaceTimeoutSeconds =
      new TunableNumber("SelfTest/FuelPlaceTimeoutSeconds", 30.0);
  private boolean waitingForFuel = false;
  private final Map<String, String> results = new LinkedHashMap<>();
  private final Map<String, Alert[]> alerts = new LinkedHashMap<>();

  /**
   * @param fuelHeld how much fuel the robot holds, if something can measure it (the fuel sim in
   *     simulation); empty on the real robot, which has no fuel sensor
   */
  public SelfTest(Vision vision, Optional<IntSupplier> fuelHeld) {
    this.vision = vision;
    this.fuelHeld = fuelHeld;
  }

  /** Same as the SelfTest/Fuel Path switch, applied right away (for tests). */
  public void setFuelPath(boolean value) {
    fuelPath.set(value);
    fuelPath.periodic();
  }

  /** Same as pressing SelfTest/Fuel Placed, applied right away (for tests). */
  public void setFuelPlaced(boolean value) {
    fuelPlaced.set(value);
    fuelPlaced.periodic();
  }

  /** Whether the fuel path is waiting for someone to set a fuel at the intake. */
  public boolean isWaitingForFuel() {
    return waitingForFuel;
  }

  /** Same as the SelfTest/Check Vision switch, applied right away (for tests). */
  public void setCheckVision(boolean value) {
    checkVision.set(value);
    checkVision.periodic();
  }

  /** Same as the SelfTest/Wheels Off Ground switch, applied right away (for tests). */
  public void setWheelsOffGround(boolean value) {
    wheelsOffGround.set(value);
    wheelsOffGround.periodic();
  }

  /** The whole self test, step by step. */
  public Command command() {
    List<Command> steps = new ArrayList<>();
    steps.add(Commands.runOnce(this::clearResults));

    steps.add(
        step(
            "Connections",
            Commands.none(),
            () -> Drive.mInstance.isGyroConnected()
                && Drive.mInstance.areModulesConnected()
                && vision.allCamerasConnected(),
            () -> "gyro " + Drive.mInstance.isGyroConnected()
                + ", swerve modules " + Drive.mInstance.areModulesConnected()
                + ", cameras " + vision.allCamerasConnected(),
            0.5));

    steps.add(
        step(
            "Hood Home",
            SuperStructure.mInstance.hoodForceHomeCommand(),
            () -> Hood.mInstance.nearPosition(HoodConstants.HOOD_MIN_ANGLE),
            () -> "at " + degrees(Hood.mInstance.getPosition()),
            3.0));
    Angle hoodMid = HoodConstants.HOOD_MIN_ANGLE.plus(HoodConstants.HOOD_MAX_ANGLE).div(2.0);
    for (Angle target : new Angle[] {hoodMid, HoodConstants.HOOD_MAX_ANGLE, HoodConstants.HOOD_MIN_ANGLE}) {
      steps.add(
          step(
              "Hood To " + degrees(target),
              Hood.mInstance.setpointCommand(Setpoint.withMotionMagicSetpoint(target)),
              () -> Hood.mInstance.nearPosition(target),
              () -> "reached " + degrees(Hood.mInstance.getPosition()) + ", wanted " + degrees(target),
              2.0));
    }

    steps.add(
        step(
            "Intake Deploy",
            IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED),
            () -> IntakeDeploy.mInstance.nearPosition(IntakeDeployConstants.INTAKE_DEPLOYED_ANGLE),
            () -> "at " + IntakeDeploy.mInstance.getPosition(),
            3.0));
    steps.add(
        roller(
            "Intake Rollers",
            IntakeRollers.mInstance.runEnd(
                () -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.INTAKE),
                () -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.IDLE)),
            () -> IntakeRollers.mInstance.getVelocity().in(RotationsPerSecond)));
    steps.add(
        step(
            "Intake Stow",
            IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED),
            () -> IntakeDeploy.mInstance.nearPosition(IntakeDeployConstants.INTAKE_STOWED_ANGLE),
            () -> "at " + IntakeDeploy.mInstance.getPosition(),
            3.0));

    steps.add(
        roller(
            "Feeder",
            Feeder.mInstance.runEnd(
                () -> Feeder.mInstance.applySetpoint(Feeder.FEED_VOLTAGE),
                () -> Feeder.mInstance.applySetpoint(Feeder.IDLE)),
            () -> Feeder.mInstance.getVelocity().in(RotationsPerSecond)));

    steps.add(
        step(
            "Shooter Spin Up",
            Shooter.mInstance.runEnd(
                () -> Shooter.mInstance.applySetpoint(Shooter.TESTSHOT),
                () -> Shooter.mInstance.applySetpoint(Shooter.IDLE)),
            () -> Shooter.mInstance.nearVelocity(
                ShooterConstants.kTestShot, RotationsPerSecond.of(shooterToleranceRps.getAsDouble())),
            () -> "at " + String.format("%.1f rps", Shooter.mInstance.getVelocity().in(RotationsPerSecond)),
            3.0));

    for (double degrees : new double[] {0, 90, 180, 270}) {
      Rotation2d angle = Rotation2d.fromDegrees(degrees);
      steps.add(
          gated(
              wheelsOffGround,
              step(
                  "Swerve Steer " + (int) degrees,
                  Drive.mInstance.run(() -> Drive.mInstance.runModuleTest(angle, 0.0)),
                  () -> Drive.mInstance.modulesAlong(angle, moduleAngleToleranceDeg.getAsDouble()),
                  () -> "a module is more than " + moduleAngleToleranceDeg.getAsDouble() + " deg off",
                  1.5),
              "Swerve Steer " + (int) degrees));
    }
    steps.add(
        gated(
            wheelsOffGround,
            step(
                "Swerve Drive",
                Drive.mInstance.runEnd(
                    () -> Drive.mInstance.runModuleTest(Rotation2d.kZero, 0.5), Drive.mInstance::stop),
                () -> Drive.mInstance.slowestModuleSpeed() > 0.3,
                () -> String.format("slowest wheel %.2f m/s", Drive.mInstance.slowestModuleSpeed()),
                2.0),
            "Swerve Drive"));

    // One camera at a time, so a person can hold a tag in front of each in turn
    for (int i = 0; i < vision.getCameraCount(); i++) {
      int camera = i;
      String name = "Vision " + vision.getCameraName(camera);
      steps.add(
          gated(
              checkVision,
              step(
                  name,
                  Commands.runOnce(() -> results.put(name, "WAITING (show it a tag)")),
                  () -> vision.cameraSeesTag(camera),
                  () -> "saw no tag",
                  visionSecondsPerCamera.getAsDouble()),
              name));
    }

    steps.add(gated(fuelPath, fuelPathSteps(), "Fuel Path"));

    steps.add(Commands.runOnce(this::publishSummary));
    return Commands.sequence(steps.toArray(new Command[0]))
        .finallyDo(
            () -> {
              Drive.mInstance.stop();
              publishSummary();
            })
        .andThen(SuperStructure.mInstance.stopCommand())
        .withName("Self Test");
  }

  /**
   * Intakes one fuel, then shoots it slowly (slow shooter speed, slow feed). Counted with the fuel sim
   * when there is one; otherwise timed, for someone to watch.
   */
  private Command fuelPathSteps() {
    int[] heldAtStart = new int[1];
    Runnable markHeld = () -> heldAtStart[0] = fuelHeld.map(IntSupplier::getAsInt).orElse(0);

    Command intake =
        Commands.parallel(
            IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED),
            IntakeRollers.mInstance.runEnd(
                () -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.INTAKE),
                () -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.IDLE)));
    Command intakeOne =
        fuelHeld.isPresent()
            // Stops as soon as one fuel is in, so only one goes through
            ? step(
                "Fuel Intake One",
                intake,
                () -> fuelHeld.get().getAsInt() > heldAtStart[0],
                () -> "no fuel came in",
                fuelIntakeSeconds.getAsDouble())
            : watchedStep("Fuel Intake One", intake, fuelIntakeSeconds.getAsDouble(), "one fuel went in");

    Command spinUp =
        step(
            "Fuel Slow Spin Up",
            Commands.none(),
            () -> Shooter.mInstance.nearVelocity(
                ShooterConstants.kSlow, RotationsPerSecond.of(shooterToleranceRps.getAsDouble())),
            () -> String.format("at %.1f rps", Shooter.mInstance.getVelocity().in(RotationsPerSecond)),
            3.0);
    Command slowFeed =
        Feeder.mInstance.runEnd(
            () -> Feeder.mInstance.applySetpoint(Feeder.SLOW_FEED_VOLTAGE),
            () -> Feeder.mInstance.applySetpoint(Feeder.IDLE));
    Command shootOne =
        fuelHeld.isPresent()
            ? step(
                "Fuel Slow Shot",
                slowFeed,
                () -> fuelHeld.get().getAsInt() < heldAtStart[0],
                () -> "the fuel never left",
                fuelShotSeconds.getAsDouble())
            : watchedStep("Fuel Slow Shot", slowFeed, fuelShotSeconds.getAsDouble(), "the fuel left the shooter");

    // The shooter holds the slow speed, hood flat, while the fuel is fed through
    Command shot =
        Commands.deadline(
            Commands.sequence(spinUp, Commands.runOnce(markHeld), shootOne),
            Shooter.mInstance.runEnd(
                () -> Shooter.mInstance.applySetpoint(Shooter.SLOW),
                () -> Shooter.mInstance.applySetpoint(Shooter.IDLE)),
            Hood.mInstance.setpointCommand(Hood.STOWED));

    String waitName = "Fuel Placed";
    Command waitForFuel =
        Commands.runOnce(
                () -> {
                  fuelPlaced.set(false);
                  waitingForFuel = true;
                  results.put(waitName, "WAITING (set one fuel at the intake, then press SelfTest/Fuel Placed)");
                  Logger.recordOutput("SelfTest/" + waitName, results.get(waitName));
                })
            .andThen(Commands.waitUntil(fuelPlaced::get).withTimeout(fuelPlaceTimeoutSeconds.getAsDouble()))
            .finallyDo(() -> waitingForFuel = false);

    Command runPath =
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  fuelPlaced.set(false);
                  results.put(waitName, "DONE");
                }),
            Commands.runOnce(markHeld),
            intakeOne,
            IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED),
            shot);

    return waitForFuel
        .andThen(
            Commands.either(
                runPath,
                Commands.runOnce(() -> results.put(waitName, "SKIPPED (no one pressed SelfTest/Fuel Placed)")),
                fuelPlaced::get))
        .withName("Self Test: Fuel Path");
  }

  /** Runs {@code action} until {@code passed} or the timeout, then records the result. */
  private Command step(
      String name, Command action, BooleanSupplier passed, Supplier<String> detail, double timeoutSec) {
    Alert[] stepAlerts = alertsFor(name);
    return Commands.deadline(Commands.waitUntil(passed).withTimeout(timeoutSec), action)
        .andThen(
            Commands.runOnce(
                () -> {
                  boolean ok = passed.getAsBoolean();
                  record(name, ok ? "PASSED" : "FAILED (" + detail.get() + ")", ok, stepAlerts);
                }))
        .withName("Self Test: " + name);
  }

  /**
   * A step the robot can't check itself: runs {@code action} for a fixed time and reports DONE, for
   * someone to confirm {@code whatToWatch} by eye. Never counts as a failure.
   */
  private Command watchedStep(String name, Command action, double seconds, String whatToWatch) {
    Alert[] stepAlerts = alertsFor(name);
    return Commands.runOnce(() -> results.put(name, "RUNNING (watch: " + whatToWatch + ")"))
        .andThen(action.withTimeout(seconds))
        .andThen(
            Commands.runOnce(
                () -> record(name, "DONE (check by eye: " + whatToWatch + ")", true, stepAlerts)))
        .withName("Self Test: " + name);
  }

  private Alert[] alertsFor(String name) {
    return alerts.computeIfAbsent(
        name,
        n -> new Alert[] {new Alert("", AlertType.kInfo), new Alert("", AlertType.kError)});
  }

  private void record(String name, String result, boolean ok, Alert[] stepAlerts) {
    results.put(name, result);
    stepAlerts[0].setText("SelfTest: " + name + " " + result);
    stepAlerts[0].set(ok);
    stepAlerts[1].setText("SelfTest: " + name + " " + result);
    stepAlerts[1].set(!ok);
    Logger.recordOutput("SelfTest/" + name, result);
  }

  /** A roller passes when it turns at least SelfTest/MinRollerRps (stalled, unplugged, broken belt). */
  private Command roller(String name, Command run, Supplier<Double> rps) {
    return step(
        name,
        run,
        () -> Math.abs(rps.get()) > minRollerRps.getAsDouble(),
        () -> String.format("only %.1f rps", rps.get()),
        1.5);
  }

  private Command gated(LoggedNetworkBoolean enabled, Command step, String name) {
    return Commands.either(
        step,
        Commands.runOnce(() -> results.put(name, "SKIPPED")),
        enabled::get);
  }

  private void clearResults() {
    results.clear();
    for (Alert[] stepAlerts : alerts.values()) {
      stepAlerts[0].set(false);
      stepAlerts[1].set(false);
    }
  }

  private void publishSummary() {
    boolean passed = results.values().stream().noneMatch(r -> r.startsWith("FAILED"));
    String[] lines =
        results.entrySet().stream().map(e -> e.getKey() + ": " + e.getValue()).toArray(String[]::new);
    SmartDashboard.putBoolean("SelfTest/Passed", passed && !results.isEmpty());
    SmartDashboard.putStringArray("SelfTest/Results", lines);
    Logger.recordOutput("SelfTest/Passed", passed && !results.isEmpty());
    Logger.recordOutput("SelfTest/Results", lines);
  }

  private static String degrees(Angle angle) {
    return String.format("%.1f deg", angle.in(Degrees));
  }
}
