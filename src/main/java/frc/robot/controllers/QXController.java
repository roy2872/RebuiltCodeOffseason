package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class QXController implements ControllerInterface {
    
    private final GenericHID controller;

    public QXController(int port) {
        controller = new GenericHID(port);
    }
  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(3));
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getRawAxis(3);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(2);
  }

  @Override
  public double rotationVelocityAnalog() {
    return -controller.getRawAxis(0);
  }

  @Override
  public Trigger intakeButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger closeIntakeButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger shootButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger shootCloseButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger fetchButton() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger xLockOverride() {
    return new Trigger(() -> false);
  }
}
