package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private enum ClimberOverrideMode {
    DISABLED,
    OPEN_LOOP
  }

  private static final String DASHBOARD_PREFIX = "Climber/Override/";
  private static final String MODE_CHOOSER_KEY = DASHBOARD_PREFIX + "Mode";
  private static final String VOLTAGE_KEY = DASHBOARD_PREFIX + "Voltage";
  private static final String RATCHET_ANGLE_KEY = DASHBOARD_PREFIX + "RatchetAngle";

  public enum ClimberStates {
    IDLE(-1.0, 0.0),
    OPEN_NO_CLIMB(CLIMBER_OPEN_POSITION_UNITS, OPEN_CLIMB_VOLTAGE),
    CLOSE_NO_CLIMB(CLIMBER_OPEN_POSITION_UNITS, CLOSE_NO_CLIMB_VOLTAGE),
    OPEN_UNCLIMB(CLIMBER_OPEN_POSITION_UNITS, UNCLIMBING_VOLTAGE),
    CLOSE_CLIMB(CLIMBER_CLOSED_POSITION_UNITS, CLOSE_CLIMB_VOLTAGE);

    public final double targetPositionUnits;
    public final double targetVoltage;

    ClimberStates(double targetPositionUnits, double targetVoltage) {
      this.targetPositionUnits = targetPositionUnits;
      this.targetVoltage = targetVoltage;
    }
  }

  @AutoLogOutput(key = "Climber/currentState")
  private ClimberStates currentState = ClimberStates.IDLE;

  private final Servo rachetServo;
  private final SendableChooser<ClimberOverrideMode> overrideModeChooser = new SendableChooser<>();

  public Climber(MotorSubsystemConfig config, MotorIO motorIO) {
    super(new MotorInputsAutoLogged(), motorIO, config);
    this.rachetServo = new Servo(0);
    overrideModeChooser.setDefaultOption("Disabled", ClimberOverrideMode.DISABLED);
    overrideModeChooser.addOption("Open Loop", ClimberOverrideMode.OPEN_LOOP);
    SmartDashboard.putData(MODE_CHOOSER_KEY, overrideModeChooser);
    SmartDashboard.putNumber(VOLTAGE_KEY, 0.0);
    SmartDashboard.putNumber(RATCHET_ANGLE_KEY, 0.0);
    super.setCurrentPosition(CLIMBER_MAX_POSITION_UNITS);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (!runDashboardOverride()) {
      stateMachine();
    }
    telemtrize();
    // Additional periodic code for Climber can be added here
  }

  private boolean runDashboardOverride() {
    ClimberOverrideMode mode = overrideModeChooser.getSelected();
    if (mode == null) {
      mode = ClimberOverrideMode.DISABLED;
    }

    Logger.recordOutput("Climber/overrideMode", mode.toString());

    switch (mode) {
      case DISABLED -> {
        return false;
      }
      case OPEN_LOOP -> {
        super.setVoltageOutput(SmartDashboard.getNumber(VOLTAGE_KEY, 0.0));
        rachetServo.setAngle(SmartDashboard.getNumber(RATCHET_ANGLE_KEY, 0.0));
        return true;
      }
    }

    return false;
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> {
        super.setVoltageOutput(0.0);
        rachetServo.setAngle(0);
      }
      case OPEN_NO_CLIMB -> {
        if (super.inputs.unitPosition < ClimberStates.OPEN_NO_CLIMB.targetPositionUnits)
          super.setVoltageOutput(ClimberStates.OPEN_NO_CLIMB.targetVoltage);
        else super.setVoltageOutput(0);
        rachetServo.setAngle(RACHET_OPEN_SERVO_POSE);
        // System.out.println("opennoclimb");
      }
      case CLOSE_NO_CLIMB -> {
        if (super.inputs.unitPosition < ClimberStates.CLOSE_NO_CLIMB.targetPositionUnits)
          super.setVoltageOutput(-ClimberStates.CLOSE_NO_CLIMB.targetVoltage);
        rachetServo.setAngle(RACHET_OPEN_SERVO_POSE); // Rachet open
        // System.out.println("closenoclimb");
      }

      case OPEN_UNCLIMB -> {
        if (super.inputs.unitPosition < ClimberStates.OPEN_UNCLIMB.targetPositionUnits)
          // super.setVoltageOutput(ClimberStates.OPEN_UNCLIMB.targetVoltage);
          // System.out.println("openUnclimb");
        // else super.setVoltageOutput(0);
        rachetServo.setAngle(RACHET_OPEN_SERVO_POSE);
      } 
      case CLOSE_CLIMB -> {
        rachetServo.setAngle(RACHET_CLOSED_SERVO_POSE);
        // System.out.println(ClimberStates.CLOSE_CLIMB.targetPositionUnits + " " + (super.inputs.unitPosition > ClimberStates.CLOSE_CLIMB.targetPositionUnits));
        if (super.inputs.unitPosition > ClimberStates.CLOSE_CLIMB.targetPositionUnits)
          super.setVoltageOutput(-ClimberStates.CLOSE_CLIMB.targetVoltage);
        else super.setVoltageOutput(0);
      }

      default -> System.out.println("Climber is really broken!!!");
    }
  }

  private void telemtrize() {}

  public void setState(ClimberStates wantedState) {
    if (wantedState != currentState) currentState = wantedState;
  }

  public ClimberStates getCurrentState() {
    return currentState;
  }

  public boolean isAtGoal() {
    return Math.abs(currentState.targetPositionUnits - super.inputs.unitPosition) <= TOLERANCE;
  }
}
