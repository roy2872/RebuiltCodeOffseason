package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants.ROLLER_VOLTAGE_INTAKE;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeRollers extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public enum IntakeRollerStates {
    INTAKE,
    PURGE,
    IDLE
  }

  @AutoLogOutput(key = "Intake/IntakeRollers/currentState")
  private IntakeRollerStates currentState = IntakeRollerStates.IDLE;

  private final SimpleMotorFeedforward ffController;

  public IntakeRollers(
      SimpleMotorFeedforward ffController, MotorSubsystemConfig config, MotorIO io) {
    super(new MotorInputsAutoLogged(), io, config);
    this.ffController = ffController;
  }

  public void realPeriodic() {
    super.periodic();
    stateMachine();
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> super.setVoltageOutput(0.0);
      case INTAKE -> super.setVoltageOutput(
          ROLLER_VOLTAGE_INTAKE + ffController.calculate(super.inputs.velocityUnitsPerSecond));

      case PURGE -> super.setVoltageOutput(
          -12 + ffController.calculate(super.inputs.velocityUnitsPerSecond));
    }
  }

  public void setState(IntakeRollerStates state) {
    currentState = state;
  }
}
