package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoControllers implements ControllerInterface {

  private final ControllerInterface mainController;
  private final ControllerInterface operatorController;

  public TwoControllers(ControllerInterface mainController, ControllerInterface operatorController) {
    this.mainController = mainController;
    this.operatorController = operatorController;
  }

  @Override
  public Trigger resetGyroButton() {
    return mainController.resetGyroButton();
  }

  @Override
  public double xVelocityAnalog() {
    return mainController.xVelocityAnalog();
  }

  @Override
  public double yVelocityAnalog() {
    return mainController.yVelocityAnalog();
  }

  @Override
  public double rotationVelocityAnalog() {
    return mainController.rotationVelocityAnalog();
  }

  @Override
  public Trigger alignToBumpButton() {
    return mainController.alignToBumpButton();
  }

  @Override
  public Trigger intakeButton() {
    return operatorController.intakeButton();
  }

  @Override
  public Trigger closeIntakeButton() {
    return operatorController.closeIntakeButton();
  }

  @Override
  public Trigger shootButton() {
    return operatorController.shootButton();
  }

  @Override
  public Trigger shootCloseButton() {
    return operatorController.shootCloseButton();
  }

  @Override
  public Trigger purgeIntakeButton() {
    return operatorController.purgeIntakeButton();
  }

  @Override
  public Trigger fetchButton() {
    return operatorController.fetchButton();
  }

  @Override
  public Trigger fetchManualButton() {
    return operatorController.fetchManualButton();
  }

  @Override
  public Trigger xLockOverride() {
    return operatorController.xLockOverride();
  }
}
