package frc.robot.controllers;

import javax.naming.ldap.Control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DummyController implements ControllerInterface {
    
    
  private final GenericHID controller;

  public DummyController() {
    controller = new GenericHID(0);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(10));
  }

  @Override
  public double xVelocityAnalog() {
    return -controller.getRawAxis(0);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double rotationVelocityAnalog() {
    return -controller.getRawAxis(2);
  }

  @Override
  public Trigger intakeButton() {
    return new Trigger(() -> controller.getRawButton(3));
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> controller.getRawButton(2));
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> controller.getRawButton(1));
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> false);
  }
}
