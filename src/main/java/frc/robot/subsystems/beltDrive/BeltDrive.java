package frc.robot.subsystems.beltDrive;

import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowers;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeltDrive extends MotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {

  public enum BeltDriveStates {
    IDLE,
    ACTIVE,
    PURGE,
    MANUAL_VOLTAGE
  }
  SendableChooser<BeltDriveStates> chooser = new SendableChooser<>();
  LoggedDashboardChooser<BeltDriveStates> stateChooser;

  @AutoLogOutput(key = "BeltDrive/currentState")
  private BeltDriveStates currentState = BeltDriveStates.IDLE;

  public BeltDrive(MotorSubsystemWithFollowersConfig config, MotorIO io, MotorIO followerIO) {
    super(config, new MotorInputsAutoLogged(), io, new MotorInputsAutoLogged[] {new MotorInputsAutoLogged()}, new MotorIO[] {followerIO});
    SmartDashboard.putBoolean("BeltDrive/ManualControl", false);
    for(var state : BeltDriveStates.values()) 
      chooser.addOption(state.name(), state);
    stateChooser = new LoggedDashboardChooser<>("BeltDrive/StateChooser", chooser);
    SmartDashboard.putNumber("BeltDrive/RequestedManualVoltage", 0.0);
  }

  @Override
  public void periodic() {
    super.periodic();
    if(SmartDashboard.getBoolean("BeltDrive/ManualControl", false) && stateChooser.get() != null)
      currentState = stateChooser.get();
    stateMachine();
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> super.setVoltageOutput(0);
      case ACTIVE -> super.setVoltageOutput(BeltDriveConstants.ACTIVE_VOLTAGE);
      case PURGE -> super.setVoltageOutput(BeltDriveConstants.PURGE_VOLTAGE);
      case MANUAL_VOLTAGE -> 
        super.setVoltageOutput(SmartDashboard.getNumber("BeltDrive/RequestedManualVoltage", 0.0));
    }
  }

  public void setState(BeltDriveStates wantedState) {
    if (currentState != wantedState) currentState = wantedState;
  }
}
