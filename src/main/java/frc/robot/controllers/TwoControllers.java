package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoControllers implements ControllerInterface {

  private final GenericHID mainController;
  private final GenericHID operatorController;

  public TwoControllers() {
    mainController = new GenericHID(0);
    operatorController = new GenericHID(1);
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
    return new Trigger(() -> operatorController.getRawButton(7));
  }

  @Override
  public Trigger closeIntakeButton() {
    return new Trigger(() -> operatorController.getRawButton(5));
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> operatorController.getRawButton(8));
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> operatorController.getRawButton(6));
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> operatorController.getRawButton(3));
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> operatorController.getRawButton(2));
  }

  @Override
  public Trigger xLockOverride() {
    return new Trigger(() -> operatorController.getRawButton(4));
  }
}
