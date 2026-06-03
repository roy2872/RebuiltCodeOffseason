package frc.robot.subsystems.hopper;

import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Hopper extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public enum HopperStates {
    IDLE,
    ACTIVE,
    PURGE
  }

  @AutoLogOutput(key = "Hopper/currentState")
  private HopperStates currentState = HopperStates.IDLE;

  private double jamTime = -1.0;
  private Trigger reverseHopperButton;

  public Hopper(MotorSubsystemConfig config, MotorIO io, Trigger reverseHopperButton) {
    super(new MotorInputsAutoLogged(), io, config);
    this.reverseHopperButton = reverseHopperButton;
  }

  @Override
  public void periodic() {
    super.periodic();
    // super.setVoltageOutput(10.0);
    if(!reverseHopperButton.getAsBoolean()) stateMachine();
    else runUnjamVoltage();
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> super.setVoltageOutput(0);
      case ACTIVE -> runVoltageDetectJams();
      case PURGE -> super.setVoltageOutput(-8.0);
    }
  }

  private void runVoltageDetectJams() {
    if(RobotController.getFPGATime() * 1e-6 - jamTime > HopperConstants.UNJAM_REVERSE_TIME) {
      super.setVoltageOutput(HopperConstants.ACTIVE_VOLTAGE);
      if(super.inputs.currentStatorAmps > HopperConstants.DETECT_JAM_CURRENT || RobotController.getFPGATime() * 1e-6 - jamTime > 5.0) jamTime = RobotController.getFPGATime() * 1e-6;
    } else {
      super.setVoltageOutput(HopperConstants.UNJAM_REVERSE_VOLTAGE);
    }
  }
  
  private void runUnjamVoltage() {
    super.setVoltageOutput(HopperConstants.UNJAM_REVERSE_VOLTAGE);
  }

  public void setState(HopperStates wantedState) {
    if (currentState != wantedState) currentState = wantedState;
  }
}
