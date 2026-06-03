package frc.robot.subsystems.beltDrive;

import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowers;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;

import org.littletonrobotics.junction.AutoLogOutput;

public class BeltDrive extends MotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {

  public enum BeltDriveStates {
    IDLE,
    ACTIVE,
    PURGE
  }

  @AutoLogOutput(key = "BeltDrive/currentState")
  private BeltDriveStates currentState = BeltDriveStates.IDLE;

  public BeltDrive(MotorSubsystemWithFollowersConfig config, MotorIO io, MotorIO followerIO) {
    super(config, new MotorInputsAutoLogged(), io, new MotorInputsAutoLogged[] {new MotorInputsAutoLogged()}, new MotorIO[] {followerIO});
  }

  @Override
  public void periodic() {
    super.periodic();
    stateMachine();
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> super.setVoltageOutput(0);
      case ACTIVE -> super.setVoltageOutput(BeltDriveConstants.ACTIVE_VOLTAGE);
      case PURGE -> super.setVoltageOutput(BeltDriveConstants.PURGE_VOLTAGE);
    }
  }

  public void setState(BeltDriveStates wantedState) {
    if (currentState != wantedState) currentState = wantedState;
  }
}
