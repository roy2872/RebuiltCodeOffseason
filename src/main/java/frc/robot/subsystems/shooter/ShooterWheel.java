package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.CircularBuffer;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowers;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig.FollowerConfig;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterWheel extends MotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {

  public enum WheelStates {
    IDLE,
    ACTIVE,
    HOLD
  };

  private WheelStates currentState = WheelStates.IDLE;
  private final SimpleMotorFeedforward ffController;
  private DoubleSupplier velocitySetpointSupplier;

  // private final CircularBuffer<Double> kfBuffer = new CircularBuffer<>(10);

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
  }

  public void realPeriodic() {
    super.periodic();
    stateMachine();

    Logger.recordOutput(getName() + "/currentState", currentState);
    Logger.recordOutput(getName() + "/requestedVelocity", velocitySetpointSupplier.getAsDouble());
  }

  private void stateMachine() {

    double targetVelocity = velocitySetpointSupplier.getAsDouble();
    double currentVelocity = inputs.velocityUnitsPerSecond;
    double voltage = inputs.appliedVolts;

    switch (currentState) {

      case IDLE -> super.setVoltageOutput(0);

      case ACTIVE -> {
        super.setMaxMotionSetpointVelocity(
            targetVelocity,
            ffController.calculate(currentVelocity));
        // super.setVoltageOutput(8);
      }

      case HOLD -> {
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
  }

  public void holdVelocity(DoubleSupplier velocitySetpointSupplier) {
    currentState = WheelStates.HOLD;
    this.velocitySetpointSupplier = velocitySetpointSupplier;
  }

  public boolean atVelocity() {
    double targetVelocity = velocitySetpointSupplier.getAsDouble();
    double currentVelocity = inputs.velocityUnitsPerSecond;
    double tolerance = Math.min(0.05 * targetVelocity, 0.05);
    return Math.abs(currentVelocity - targetVelocity) <= tolerance;
  }

  // private void resetHold() {
  //   kfBuffer.clear();
  // }
}