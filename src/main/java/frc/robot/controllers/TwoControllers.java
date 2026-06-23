package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoControllers implements ControllerInterface {

  private final GenericHID mainController;
  private final GenericHID operatormainController;

  public TwoControllers() {
    mainController = new GenericHID(0);
    operatormainController = new GenericHID(1);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> mainController.getRawButton(3));
  }

  @Override
  public double xVelocityAnalog() {
    return mainController.getRawAxis(3);
  }

  @Override
  public double yVelocityAnalog() {
    return -mainController.getRawAxis(2);
  }

  @Override
  public double rotationVelocityAnalog() {
    return -mainController.getRawAxis(0);
  }

  @Override
  public Trigger intakeButton() {
    return new Trigger(() -> !mainController.getRawButton(1));
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> mainController.getRawAxis(7) == -1);
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> mainController.getRawAxis(7) == 0);
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> mainController.getRawButton(4));
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> mainController.getRawAxis(7) == 1);
  }

  @Override
  public Trigger xLockOverride() {
    return new Trigger(() -> false);
  }
}
