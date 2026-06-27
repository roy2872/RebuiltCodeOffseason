package frc.robot.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.util.ControlGains.FFConstants;
import frc.lib.util.ControlGains.PidGains;
import frc.lib.util.ControlGains.SimpleFFConstants;
import frc.robot.Constants;

public class IntakeConstants {

  public static final class IntakeRollerConstants {

    public static final double ROLLER_VOLTAGE_INTAKE = -8.0;

    public static final SimpleFFConstants INTAKE_ROLLER_FF =
        switch (Constants.currentMode) {
          case REAL -> new SimpleFFConstants(0.0, 0.0, 0.0);
          case SIM -> new SimpleFFConstants(0.0, 0.0, 0.0);
          default -> new SimpleFFConstants(0.0, 0.0, 0.0);
        };

    public static final MotorSubsystemConfig INTAKE_ROLLER_CONFIG = new MotorSubsystemConfig();

    static {
      INTAKE_ROLLER_CONFIG.name = "IntakeRollers";
      INTAKE_ROLLER_CONFIG.id = 51; 

      INTAKE_ROLLER_CONFIG
          .sparkConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60)
          .inverted(false)
          .secondaryCurrentLimit(80);

      INTAKE_ROLLER_CONFIG.unitToRotorRatio = 1.0; // TODO: set ratio

      INTAKE_ROLLER_CONFIG.usingAbsoluteEncoder = false;

      INTAKE_ROLLER_CONFIG.momentOfInertia = 0.1; // TODO: set MOI
    } 
  }

  public static final class IntakeDeployConstants {

    // public static final double INTAKE_OPEN_ANGLE = Units.degreesToRadians(30); // TODO: tune angles
    // public static final double INTAKE_CLOSED_ANGLE = Units.degreesToRadians(80);
    public static final double INTAKE_OPEN_ANGLE = 0.2;
    public static final double INTAKE_CLOSED_ANGLE = 0.3; 

    // public static final double INTAKE_MIN_ANGLE = Units.degreesToRadians(0);
    // public static final double INTAKE_MAX_ANGLE = Units.degreesToRadians(90);

    public static final FFConstants INTAKE_DEPLOY_FF =
        switch (Constants.currentMode) {
          case REAL -> new FFConstants(0.0, 0.0, 0.0, 0.0);
          case SIM -> new FFConstants(0.0, 0.0, 0.0, 0.0);
          default -> new FFConstants(0.0, 0.0, 0.0, 0.0);
        };

    private static final PidGains DEPLOY_PID =
        switch (Constants.currentMode) {
          case REAL -> new PidGains(5, 0.0, 0.0);
          case SIM -> new PidGains(0.1, 0.0, 0.0);
          default -> new PidGains(0.1, 0.0, 0.0);
        };

    public static final MotorSubsystemConfig INTAKE_DEPLOY_CONFIG = new MotorSubsystemConfig();

    static {
      INTAKE_DEPLOY_CONFIG.name = "IntakeDeploy";
      INTAKE_DEPLOY_CONFIG.id = 52; 

      INTAKE_DEPLOY_CONFIG
          .sparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(60)
          .inverted(true)
          .secondaryCurrentLimit(80)
          .closedLoop
          .pidf(
              DEPLOY_PID.kP,
              DEPLOY_PID.kI,
              DEPLOY_PID.kD,
              0,
              ClosedLoopSlot.kSlot0)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .maxMotion
            .maxVelocity(4.0); // keep velocity ff 0
      INTAKE_DEPLOY_CONFIG
          .sparkConfig
          .softLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(0.3)
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(0.2); // TODO: may be the other way around

      INTAKE_DEPLOY_CONFIG.unitToRotorRatio = 1.0; // TODO: set ratio

      INTAKE_DEPLOY_CONFIG.usingAbsoluteEncoder = true;
      INTAKE_DEPLOY_CONFIG.absoluteEncoderToRotorRatio = 1; // TODO set ratio

      INTAKE_DEPLOY_CONFIG.momentOfInertia = 0.1; // TODO: set MOI
    }
  }
}
