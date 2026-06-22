package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.hood.HoodConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.util.ControlGains.PidGains;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private enum HoodOverrideMode {
    DISABLED,
    VOLTAGE,
    POSITION
  }

  private static final String DASHBOARD_PREFIX = "Hood/Override/";
  private static final String MODE_CHOOSER_KEY = DASHBOARD_PREFIX + "Mode";
  private static final String VOLTAGE_KEY = DASHBOARD_PREFIX + "Voltage";
  private static final String POSITION_KEY = DASHBOARD_PREFIX + "Position";

  // private static final String PID_PREFIX = "Hood/PID/";
  // private static final String KP_KEY = PID_PREFIX + "Kp";
  // private static final String KI_KEY = PID_PREFIX + "Ki";
  // private static final String KD_KEY = PID_PREFIX + "Kd";
  // private static final String PID_CONFIRM_KEY = PID_PREFIX + "Confirm";

  public enum HoodStates {
    TRACKING,
    IDLE,
    CLOSED,
    BOOT_SEQUENCE
  }

  @AutoLogOutput(key = "Hood/currentState")
  private HoodStates currentState = HoodStates.IDLE;

  @AutoLogOutput(key = "Hood/targetAngle")
  private DoubleSupplier targetAngle = ()->0.0;

  private final SimpleMotorFeedforward MOTOR_FF;
  private final SendableChooser<HoodOverrideMode> overrideModeChooser = new SendableChooser<>();
  private final Timer bootSequenceTimer;

  @AutoLogOutput(key = "Hood/hasBeenReset")
  private boolean hasBeenReset = false;

  private SysIdRoutine hoodRoutine;

  public Hood(MotorSubsystemConfig config, MotorIO io) {
    super(new MotorInputsAutoLogged(), io, config);
    SmartDashboard.putBoolean("Hood/ResetButton", false);
    MOTOR_FF =
        new SimpleMotorFeedforward( // kg is negligable
            HOOD_FF.kS(), HOOD_FF.kV(), HOOD_FF.kA());

    hoodRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds),
        Volts.of(4),
        Seconds.of(3),
        (state) -> Logger.recordOutput("Hood/hoodSysidState", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> this.setVoltageOutput(volts.in(Volts)),
        null,
        this
      )
    );
    overrideModeChooser.setDefaultOption("Disabled", HoodOverrideMode.DISABLED);
    overrideModeChooser.addOption("Voltage", HoodOverrideMode.VOLTAGE);
    overrideModeChooser.addOption("Position", HoodOverrideMode.POSITION);
    SmartDashboard.putData(MODE_CHOOSER_KEY, overrideModeChooser);
    SmartDashboard.putNumber(VOLTAGE_KEY, 0.0);
    SmartDashboard.putNumber(POSITION_KEY, HOOD_STARTING_ANGLE);

    // SmartDashboard.putNumber(KP_KEY, HOOD_PID.kP);
    // SmartDashboard.putNumber(KI_KEY, HOOD_PID.kI);
    // SmartDashboard.putNumber(KD_KEY, HOOD_PID.kD);
    // SmartDashboard.putBoolean(PID_CONFIRM_KEY, false);
    super.setCurrentPosition(HoodConstants.HOOD_STARTING_ANGLE);
    bootSequenceTimer = new Timer();
  }

  @Override
  public void periodic() {
    super.periodic();
    if(SmartDashboard.getBoolean("Hood/ResetButton", false))
      setState(HoodStates.BOOT_SEQUENCE);
      
    // if (SmartDashboard.getBoolean(PID_CONFIRM_KEY, false)) {
    //   io.setPID(
    //     SmartDashboard.getNumber(KP_KEY, HOOD_PID.kP),
    //     SmartDashboard.getNumber(KI_KEY, HOOD_PID.kI),
    //     SmartDashboard.getNumber(KD_KEY, HOOD_PID.kD)
    //   );
    // }
    if (!runDashboardOverride()) {
      stateMachine();
    }
  }

  private boolean runDashboardOverride() {
    HoodOverrideMode mode = overrideModeChooser.getSelected();
    if (mode == null) {
      mode = HoodOverrideMode.DISABLED;
    }

    Logger.recordOutput("Hood/overrideMode", mode.toString());

    switch (mode) {
      case DISABLED -> {
        return false;
      }
      case VOLTAGE -> {
        super.setVoltageOutput(SmartDashboard.getNumber(VOLTAGE_KEY, 0.0));
        return true;
      }
      case POSITION -> {
        double positionSetpoint = SmartDashboard.getNumber(POSITION_KEY, HOOD_STARTING_ANGLE);
        super.setMaxMotionSetpointPosition(
            positionSetpoint, MOTOR_FF.calculate(super.inputs.velocityUnitsPerSecond));
        return true;
      }
    }

    return false;
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> super.setVoltageOutput(0);
      case TRACKING -> super.setMaxMotionSetpointPosition(
          targetAngle.getAsDouble(), MOTOR_FF.calculate(super.inputs.velocityUnitsPerSecond));
      case CLOSED -> super.setMaxMotionSetpointPosition(HOOD_MAX_ANGLE, HOOD_ANGLE_TOLERANCE);
      case BOOT_SEQUENCE -> {
        if(!bootSequenceTimer.hasElapsed(BOOT_SEQUENCE_TIME)) {
          hasBeenReset = true;
          resetPosition(HOOD_STARTING_ANGLE);
          bootSequenceTimer.reset();
          bootSequenceTimer.stop();
        }
        else super.setVoltageOutput(BOOT_SEQUENCE_VOLTAGE);
      }
      default -> super.setVoltageOutput(0);
    }
  }

  public void setState(HoodStates wantedState) {
    if(wantedState == HoodStates.BOOT_SEQUENCE && currentState != HoodStates.BOOT_SEQUENCE) 
      startBootSequence();
    else if (currentState == HoodStates.BOOT_SEQUENCE && wantedState != HoodStates.BOOT_SEQUENCE) 
      return;
    if (currentState != wantedState) currentState = wantedState;
  }

  public void setTargetAngle(DoubleSupplier angle) { // shooting command sets this according to distance
    setState(HoodStates.TRACKING);
    this.targetAngle = angle;
  }

  public void resetPosition(double newpos) {
    super.setCurrentPosition(newpos);
  }

  public void startBootSequence() {
    bootSequenceTimer.reset();
    bootSequenceTimer.start();
    // setState(HoodStates.BOOT_SEQUENCE);
  }

  public void stopBootSequence() {
    bootSequenceTimer.stop();
    setState(HoodStates.IDLE);
  }
  
  public Command hoodSysidRoutine(boolean quasistatic, SysIdRoutine.Direction direction) {
    return Commands.print(
      "Running sysid on this mechanism is dangerous, " +
      "please uncomment the following code if you are sure of what you're doing."
    );
  }

  public boolean wasHoodReset() {
    return hasBeenReset;
  }

  public boolean atSetpoint() {
    if(currentState == HoodStates.BOOT_SEQUENCE) 
      return false;
    return Math.abs(super.inputs.unitPosition - targetAngle.getAsDouble()) <= 0.5;
  }
}
