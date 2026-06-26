package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleXboxController implements ControllerInterface {

  private final XboxController controller;

  public SingleXboxController() {
    controller = new XboxController(0);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8) &&true);
  }

  @Override
  public double xVelocityAnalog() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(0);
  }

  @Override
  public double rotationVelocityAnalog() {
    return -controller.getRightX();
  }

  @Override
  public Trigger intakeButton() {
    return new Trigger(() -> (controller.getRightTriggerAxis() > 0.25));
  }

  @Override
  public Trigger closeIntakeButton() {
    return new Trigger(() -> controller.getLeftBumper());
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> controller.getRightBumperButton());
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> controller.getRawButton(7));
  }

  @Override
  public Trigger purgeIntakeButton() {
    // return new Trigger(() -> controller.getPOV() == 270);
    return new Trigger(() -> controller.getRawButton(1));
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> controller.getPOV() == 180);
  }

  @Override
  public Trigger xLockOverride() {
    return new Trigger(() -> false);
  }
}
