package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleXboxController implements ControllerInterface {

  private final XboxController controller;

  public SingleXboxController(int port) {
    controller = new XboxController(port);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8));
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
    return new Trigger(() -> (controller.getRightTriggerAxis() > 0.7));
  }

  @Override
  public Trigger closeIntakeButton() {
    return new Trigger(() -> controller.getRightBumperButton());
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> controller.getYButton());
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> controller.getXButton());
  }

  @Override
  public Trigger purgeIntakeButton() {
    // return new Trigger(() -> controller.getPOV() == 270);
    return new Trigger(() -> controller.getRightBumperButton());
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> controller.getBButton());
  }

  @Override
  public Trigger fetchManualButton() {
    return new Trigger(() -> controller.getAButton());
  }

  @Override
  public Trigger xLockOverride() {
    return new Trigger(() -> controller.getRightTriggerAxis() > 0.7);
  }
}
