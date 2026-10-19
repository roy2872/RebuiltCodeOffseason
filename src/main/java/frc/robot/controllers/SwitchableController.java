package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Forwards to whichever controller is picked in the "Controller" dashboard chooser, so the mapping
 * can be changed without redeploying, including at a match. Buttons can be bound once at startup:
 * each one follows the current pick. The pick is logged, so replay uses the same mapping.
 */
public class SwitchableController implements ControllerInterface {
  private final LoggedDashboardChooser<ControllerInterface> chooser =
      new LoggedDashboardChooser<>("Controller");
  private final Map<String, ControllerInterface> options = new LinkedHashMap<>();
  private final ControllerInterface defaultController;

  public SwitchableController() {
    options.put("Xbox", new SingleXboxController(0));
    options.put("Simulation", new SimulationController(0));
    options.put("QX", new QXController(0));
    options.put("Dummy", new DummyController(0));
    options.put(
        "Two Xbox (driver 0, operator 1)",
        new TwoControllers(new SingleXboxController(0), new SingleXboxController(1)));

    // Replay gets the robot's default, for logs that don't hold the pick
    String defaultName = Constants.currentMode == Constants.Mode.SIM ? "Simulation" : "Xbox";
    defaultController = options.get(defaultName);
    chooser.addDefaultOption(defaultName, defaultController);
    options.forEach((name, controller) -> {
      if (controller != defaultController) chooser.addOption(name, controller);
    });
  }

  private ControllerInterface selected() {
    ControllerInterface controller = chooser.get();
    return controller != null ? controller : defaultController;
  }

  /** A trigger that reads the given button on whichever controller is currently picked. */
  private Trigger switching(Function<ControllerInterface, Trigger> button) {
    // Each controller's trigger is made once here, not every loop
    Map<ControllerInterface, Trigger> triggers = new HashMap<>();
    options.values().forEach(controller -> triggers.put(controller, button.apply(controller)));
    return new Trigger(() -> triggers.get(selected()).getAsBoolean());
  }

  @Override
  public Trigger resetGyroButton() {
    return switching(ControllerInterface::resetGyroButton);
  }

  @Override
  public double xVelocityAnalog() {
    return selected().xVelocityAnalog();
  }

  @Override
  public double yVelocityAnalog() {
    return selected().yVelocityAnalog();
  }

  @Override
  public double rotationVelocityAnalog() {
    return selected().rotationVelocityAnalog();
  }

  @Override
  public Trigger alignToBumpButton() {
    return switching(ControllerInterface::alignToBumpButton);
  }

  @Override
  public Trigger intakeButton() {
    return switching(ControllerInterface::intakeButton);
  }

  @Override
  public Trigger closeIntakeButton() {
    return switching(ControllerInterface::closeIntakeButton);
  }

  @Override
  public Trigger intakeForceHomeButton() {
    return switching(ControllerInterface::intakeForceHomeButton);
  }

  @Override
  public Trigger shootButton() {
    return switching(ControllerInterface::shootButton);
  }

  @Override
  public Trigger shootCloseButton() {
    return switching(ControllerInterface::shootCloseButton);
  }

  @Override
  public Trigger purgeIntakeButton() {
    return switching(ControllerInterface::purgeIntakeButton);
  }

  @Override
  public Trigger fetchButton() {
    return switching(ControllerInterface::fetchButton);
  }

  @Override
  public Trigger fetchManualButton() {
    return switching(ControllerInterface::fetchManualButton);
  }

  @Override
  public Trigger xLockOverride() {
    return switching(ControllerInterface::xLockOverride);
  }
}
