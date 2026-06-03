package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.IntakeDeployConstants.INTAKE_CLOSED_ANGLE;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeDeployConstants.INTAKE_OPEN_ANGLE;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public enum IntakeDeployStates {
    OPEN,
    CLOSED,
    IDLE
  }

  @AutoLogOutput(key = "Intake/IntakeDeploy/currentState")
  private IntakeDeployStates currentState = IntakeDeployStates.IDLE;

  private final ArmFeedforward ffController;

  public IntakeDeploy(ArmFeedforward ffController, MotorSubsystemConfig config, MotorIO io) {
    super(new MotorInputsAutoLogged(), io, config);
    this.ffController = ffController;
  }

  public void realPeriodic() {
    super.periodic();
    // super.setVoltageOutput(2.0);
    // important- positiion should be at 0.410
    stateMachine(); // TODO: dont forget to uncomment when intake works
    Logger.recordOutput("Intake/IntakeDeploy/requestedAngle", positionSetpoint);
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> super.setVoltageOutput(0);
      case OPEN -> super.setPositionSetpoint(
          INTAKE_OPEN_ANGLE,
          ffController.calculate(super.inputs.unitPosition, super.inputs.velocityUnitsPerSecond));
      case CLOSED -> super.setPositionSetpoint(
          INTAKE_CLOSED_ANGLE,
          ffController.calculate(super.inputs.unitPosition, super.inputs.velocityUnitsPerSecond));
    }
  }

  public void runAngle(double positionSetpoint) {
    this.positionSetpoint = positionSetpoint; // optional
  }

  public void setOverridePosition(double positionSetpoint) {
    super.setPositionSetpoint(
        positionSetpoint,
        ffController.calculate(super.inputs.unitPosition, super.inputs.velocityUnitsPerSecond));
  }

  public void setState(IntakeDeployStates state) {
    this.currentState = state;
  }
}
