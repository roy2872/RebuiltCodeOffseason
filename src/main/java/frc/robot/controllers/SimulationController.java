package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimulationController implements ControllerInterface {
  private final GenericHID controller;

  public SimulationController() {
    controller = new GenericHID(0);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8));
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getRawAxis(0);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double rotationVelocityAnalog() {
    return controller.getRawAxis(2);
  }

  @Override
  public Trigger intakeButton() {
    return new Trigger(() -> controller.getRawButton(1));
  }

  @Override
  public Trigger closeIntakeButton() {
    return new Trigger(() -> controller.getRawButton(6));
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> controller.getRawButton(7));
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> controller.getRawButton(2));
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> controller.getRawButton(3));
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> controller.getRawButton(4));
  }

  @Override
  public Trigger xLockOverride() {
    return new Trigger(() -> false);
  }
}
