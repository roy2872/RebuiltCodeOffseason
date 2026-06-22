package frc.lib.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ControlGains.PidGains;

import org.littletonrobotics.junction.Logger;

public class MotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
    extends SubsystemBase {
  protected final T inputs;
  protected final U io;
  protected final boolean usingAbsoluteEncoder;

  protected double positionSetpoint = 0.0;

  protected MotorSubsystemConfig config;

  private String PID_PREFIX;
  private String KP_KEY;
  private String KI_KEY;
  private String KD_KEY;
  private String PID_CONFIRM_KEY;

  private PidGains pidGains;

  public MotorSubsystem(T inputs, U io, MotorSubsystemConfig config) {
    this.inputs = inputs;
    this.io = io;
    this.config = config;

    this.usingAbsoluteEncoder = config.usingAbsoluteEncoder;
    setupPID();
  }

  @Override
  public void periodic() {
    double timestamp = RobotController.getFPGATime() * 1e-6;
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    Logger.recordOutput(
        getName() + "/latencyPeriodicSeconds", RobotController.getFPGATime() * 1e-6 - timestamp);

    if (SmartDashboard.getBoolean(PID_CONFIRM_KEY, false)) {
      double kp, ki, kd;
      kp = SmartDashboard.getNumber(KP_KEY, pidGains.kP);
      ki = SmartDashboard.getNumber(KI_KEY, pidGains.kI);
      kd = SmartDashboard.getNumber(KD_KEY, pidGains.kD);
      io.setPID(
        kp, ki, kd
      );
      this.pidGains = new PidGains(kp, ki, kd);
      SmartDashboard.putBoolean(PID_CONFIRM_KEY, false);
    }
    Logger.recordOutput(PID_PREFIX + "Active/kP", pidGains.kP);
    Logger.recordOutput(PID_PREFIX + "Active/kI", pidGains.kI);
    Logger.recordOutput(PID_PREFIX + "Active/kD", pidGains.kD);
  }

  /**
   * @param positionUnits in the subsystem units
   * @param ffVolts
   */
  public void setPositionSetpoint(double positionUnits, double ffVolts) {
    this.positionSetpoint = positionUnits;
    Logger.recordOutput(getName() + "/Requested/SetpointPositionUnits", positionUnits);
    io.setPositionSetpoint(positionUnits, ffVolts);
  }

  public void setVelocitySetpoint(double velocityUnitsPerSecond, double ffVolts) {
    Logger.recordOutput(
        getName() + "/Requested/SetpointVelocityUnitsPerSecond", velocityUnitsPerSecond);
    io.setVelocitySetpoint(velocityUnitsPerSecond, ffVolts);
  }

  public void setMaxMotionSetpointPosition(double positionUnits, double ffVolts) {
    positionSetpoint = positionUnits;
    Logger.recordOutput(getName() + "/Requested/MaxMotionSetpointPositionUnits", positionUnits);
    io.setMaxMotionSetpointPosition(positionUnits, ffVolts);
  }

  public void setMaxMotionSetpointVelocity(double velocityUnitsPerSecond, double ffVolts) {
    Logger.recordOutput(
        getName() + "/Requested/MaxMotionSetpointVelocityUnitsPerSecond", velocityUnitsPerSecond);
    io.setMaxMotionSetpointVelocity(velocityUnitsPerSecond, ffVolts);
  }

  public void setVoltageOutput(double voltage) {
    Logger.recordOutput(getName() + "/Requested/VoltageOutputVolts", voltage);
    io.setVoltageOutput(voltage);
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  public void setCurrentPositionAsZero() {
    io.setCurrentPositionAsZero();
  }

  public void setCurrentPosition(double positionUnits) {
    io.setCurrentPosition(positionUnits);
  }

  public void setNeutralMode(boolean isBrake) {
    io.setNeutralMode(isBrake);
  }

  public void setEnableSoftLimit(boolean fwd, boolean rev) {
    io.setEnableSoftLimit(fwd, rev);
  }

  public void setEnableHardLimit(boolean fwd, boolean rev) {
    io.setEnableHardLimit(fwd, rev);
  }

  public void setPID(double kp, double ki, double kd) {
    io.setPID(kp, ki, kd);
  }

  private void setupPID() {
    PID_PREFIX = getName() + "/PID/";
    KP_KEY = PID_PREFIX + "Kp";
    KI_KEY = PID_PREFIX + "Ki";
    KD_KEY = PID_PREFIX + "Kd";
    PID_CONFIRM_KEY = PID_PREFIX + "Confirm";

    pidGains = io.getPID();
    SmartDashboard.putNumber(KP_KEY, pidGains.kP);
    SmartDashboard.putNumber(KI_KEY, pidGains.kI);
    SmartDashboard.putNumber(KD_KEY, pidGains.kD);
    SmartDashboard.putBoolean(PID_CONFIRM_KEY, false);
  }
}
