package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowers;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig.FollowerConfig;
import frc.robot.Constants;

import static frc.robot.subsystems.drive.DriveConstants.MAX_ACCELERATION;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

public class ShooterWheel extends MotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {

  public enum WheelStates {
    IDLE,
    ACTIVE,
    HOLD
  };

  private static final String DASHBOARD_PREFIX = "Shooter/Override/";
  private static final String MODE_CHOOSER_KEY = DASHBOARD_PREFIX + "Mode";
  private static final String VOLTAGE_KEY = DASHBOARD_PREFIX + "Voltage";
  private static final String VELOCITY_KEY = DASHBOARD_PREFIX + "Velocity";



  private WheelStates currentState = WheelStates.IDLE;
  private final SimpleMotorFeedforward ffController;
  private DoubleSupplier velocitySetpointSupplier;

  // private final CircularBuffer<Double> kfBuffer = new CircularBuffer<>(10);

  private DCMotor gearbox;

  private double profiledSetpointRadPerSec = 0.0;

  private Debouncer atGoalTimer;

  private double ffVolts = 0.0; //fuck it we ball
  private double filteredAccel = 0.0;
  private boolean nonZeroAccel = false;

  public ShooterWheel(
      SimpleMotorFeedforward ffController, 
      String name, 
      MotorSubsystemWithFollowersConfig mainConfig, 
      MotorIO mainIO,
      MotorIO followerIO) {
        //config input io input io
    super(
      mainConfig,
      new MotorInputsAutoLogged(), 
      mainIO,
      new MotorInputsAutoLogged[]{new MotorInputsAutoLogged()}, new MotorIO[]{followerIO}
    );
    super.setName(name);
    this.ffController = ffController;
    velocitySetpointSupplier = () -> 0;
    super.setupPID();

    gearbox = DCMotor.getNEO(2);

    atGoalTimer = new Debouncer(1.0);
  }

  public void realPeriodic() {
    super.periodic();
    stateMachine();

    Logger.recordOutput(getName() + "/currentState", currentState);
    Logger.recordOutput(getName() + "/requestedVelocity", velocitySetpointSupplier.getAsDouble());
  }

  private void stateMachine() {

    double targetVelocity = Units.radiansPerSecondToRotationsPerMinute(profiledSetpointRadPerSec) / 60.0; // [RPS]
    double currentVelocity = inputs.velocityUnitsPerSecond;
    double voltage = inputs.appliedVolts;

    Logger.recordOutput(getName() + "/targetVelocity", targetVelocity);
    switch (currentState) {

      case IDLE -> {
        super.setVoltageOutput(0);
        ffVolts = 0; 
        profiledSetpointRadPerSec = 0;
        atGoalTimer.calculate(false);
      }

      case ACTIVE -> {
          // if(currentVelocity < targetVelocity && 
  // targetVelocity - currentVelocity > (1-ShooterConstants.ACTIVATE_MAX_POWER_PERCENTAGE)*targetVelocity + 0.3 * targetVelocity) {
            // super.setVoltageOutput(10.0);
          // }
          // else 
          super.setMaxMotionSetpointVelocity(
            targetVelocity,
            ffVolts
            // ffController.calculate(targetVelocity)
            );
      }

      case HOLD -> {
          // if(currentVelocity < targetVelocity && 
  // targetVelocity - currentVelocity > (1-ShooterConstants.ACTIVATE_MAX_POWER_PERCENTAGE)*targetVelocity + 0.3 * targetVelocity) {
            // super.setVoltageOutput(10.0);
          // }
          // else 
          super.setMaxMotionSetpointVelocity(
            targetVelocity,
            ffController.calculate(currentVelocity));
        // super.setVoltageOutput(8);
        // double error = Math.abs(currentVelocity - targetVelocity); // 480 500
        // // boolean onTarget = error < Math.max(0.05 * targetVelocity, 0.05); 
        // boolean onTarget = error < Math.max((0.05*60) * targetVelocity, 0.05*60);

        // if (onTarget && currentVelocity > 1e-4) {
        //   double kf = voltage / currentVelocity;
        //   kfBuffer.addFirst(kf);
        // }

        // double avgKf = 0;
        // int samples = kfBuffer.size();

        // for (int i = 0; i < samples; i++) {
        //   avgKf += (double) kfBuffer.get(i);
        // }

        // if (samples > 0) {
        //   avgKf /= samples;
        // }

        // double outputVoltage = avgKf * targetVelocity;

        // if (currentVelocity <= 1e-4) {
        //   super.setVoltageOutput(12.0);
        // }
        // else super.setVoltageOutput(outputVoltage);

        // Logger.recordOutput(getName() + "/kfEstimate", avgKf);
      }
    }
  }

  public void setStateIdle() {
    currentState = WheelStates.IDLE;
    velocitySetpointSupplier = () -> 0;
  }

  public void runVelocity(DoubleSupplier velocitySetpointSupplier) {
      currentState = WheelStates.ACTIVE;
      
      this.velocitySetpointSupplier = velocitySetpointSupplier;
      
      double targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
          this.velocitySetpointSupplier.getAsDouble() * 60.0
      );

      double currentVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
          inputs.velocityUnitsPerSecond * 60.0
      );

      double vbus = RobotController.getBatteryVoltage();
      if (Constants.currentMode == Constants.Mode.SIM) {
          vbus = 12.0;
      }
      double supplyLimit = 120.0; 
      double budget = supplyLimit * vbus * 0.82; 
      
      double backEmf = currentVelocityRadPerSec / gearbox.KvRadPerSecPerVolt;

      // 2. Physics current bounds
      double maxStatorCurrent = (-backEmf + Math.sqrt(backEmf * backEmf + 4.0 * budget * gearbox.rOhms)) / (2.0 * gearbox.rOhms);
      double voltageLimitedCurrent = Math.max(0.0, (vbus - backEmf) / gearbox.rOhms);
      maxStatorCurrent = Math.min(maxStatorCurrent, voltageLimitedCurrent);

      // 3. Accelerations bounds
      double moi = getName() == "MainWheel" ? 0.015 : 0.001; // [kg*m^2] moment of inertia
      double maxAcceleration = getName() == "MainWheel" ? 2000.0 : 2000.0;

      double maxAccelFromCurrent = (gearbox.getTorque(maxStatorCurrent) - gearbox.getTorque(ffController.getKs() / gearbox.rOhms)) / moi;
      maxAccelFromCurrent = MathUtil.clamp(maxAccelFromCurrent, 0.0, Units.rotationsPerMinuteToRadiansPerSecond(maxAcceleration));

      // 4. Smooth profiling step
      double maxStep = maxAccelFromCurrent * Constants.CYCLE_TIME;
      double profiledError = targetVelocityRadPerSec - profiledSetpointRadPerSec;
      double rawAccel;
      if (Math.abs(profiledError) <= maxStep) {
          profiledSetpointRadPerSec = targetVelocityRadPerSec;
          rawAccel = profiledError / Constants.CYCLE_TIME;
      } else {
          profiledSetpointRadPerSec += Math.copySign(maxStep, profiledError);
          rawAccel = Math.copySign(maxAccelFromCurrent, profiledError);
      }

      double maxError = Units.rotationsPerMinuteToRadiansPerSecond(150);
      profiledSetpointRadPerSec = MathUtil.clamp(
        profiledSetpointRadPerSec, 
        currentVelocityRadPerSec - maxError,
        currentVelocityRadPerSec + maxError
      );

      if (!nonZeroAccel) {
        filteredAccel = rawAccel;
      } else {
        filteredAccel +=
          (rawAccel - filteredAccel) * Constants.CYCLE_TIME * 0.05;
      }
      nonZeroAccel = true;

      double nativeVelocity = Units.radiansPerSecondToRotationsPerMinute(profiledSetpointRadPerSec) / 60.0;
      double nativeAccel = Units.radiansPerSecondToRotationsPerMinute(filteredAccel) / 60.0;
      ffVolts = 
        Math.signum(profiledSetpointRadPerSec) * ffController.getKs() + 
        nativeVelocity * ffController.getKv() +
        nativeAccel * ffController.getKa();
        

      Logger.recordOutput(getName() + "/error", Units.radiansPerSecondToRotationsPerMinute(profiledError)/60.0);
      Logger.recordOutput(getName() + "/ffVolts", ffVolts);
      Logger.recordOutput(getName() + "/SetpointAccel", filteredAccel);
      Logger.recordOutput(getName() + "/maxAccel", maxAccelFromCurrent);
  }


  public void holdVelocity(DoubleSupplier velocitySetpointSupplier) {
    currentState = WheelStates.HOLD;
    this.velocitySetpointSupplier = velocitySetpointSupplier;
  }

  public boolean atVelocity() {
    double targetVelocity = velocitySetpointSupplier.getAsDouble();
    double currentVelocity = inputs.velocityUnitsPerSecond;
    double tolerance = Math.min(0.15 * targetVelocity, 1); // 0.05
    if (targetVelocity < 1e-4) {
      tolerance = 1.0; 
    }
    boolean atGoal = Math.abs(currentVelocity - targetVelocity) <= tolerance;
    return atGoalTimer.calculate(atGoal);
  }

  // private void resetHold() {
  //   kfBuffer.clear();
  // }
}