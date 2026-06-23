package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerInterface {

  public Trigger resetGyroButton();

  public double xVelocityAnalog();

  public double yVelocityAnalog();

  public double rotationVelocityAnalog();

  public Trigger intakeButton();

  public Trigger openClimbButton();

  public Trigger closeClimbButton();

  public Trigger shootButton();

  public Trigger shootCloseButton();

  public Trigger purgeIntakeButton();

  public Trigger fetchButton();

  public Trigger xLockOverride();
}
