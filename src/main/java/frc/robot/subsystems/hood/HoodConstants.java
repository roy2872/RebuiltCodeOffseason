package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_STARTING_ANGLE;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.robot.Constants;

public class HoodConstants {

  public record SimpleFFConstants(double kS, double kV, double kA) {}

  public record PidGains(double kP, double kI, double kD) {}

  public static final double HOOD_STARTING_ANGLE = 90; // [deg]
  public static final double HOOD_MIN_ANGLE = HOOD_STARTING_ANGLE-26; // [raw motor angle]
  public static final double HOOD_MAX_ANGLE = HOOD_STARTING_ANGLE-2; // [raw motor angle]
  
  public static final double HOOD_ANGLE_TOLERANCE = 0.5; // [deg]

  public static final SimpleFFConstants HOOD_FF =
      switch (Constants.currentMode) {
        case REAL -> new SimpleFFConstants(0.0, 0.0, 0.0);
        case SIM -> new SimpleFFConstants(0.0, 0.0, 0.0);
        default -> new SimpleFFConstants(0.0, 0.0, 0.0);
      };

  private static final PidGains HOOD_PID =
      switch (Constants.currentMode) {
        case REAL -> new PidGains(2.5, 0.0, 0.0); // tuned fine
        case SIM -> new PidGains(0.1, 0.0, 0.0);
        default -> new PidGains(0.1, 0.0, 0.0);
      };

  public static final MotorSubsystemConfig HOOD_CONFIG = new MotorSubsystemConfig();

  static {
    HOOD_CONFIG.name = "Hood";
    HOOD_CONFIG.id = 58;

    HOOD_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20) 
        .inverted(false)
        .secondaryCurrentLimit(30)
        .closedLoop
        .pidf(HOOD_PID.kP, HOOD_PID.kI, HOOD_PID.kD, 0, ClosedLoopSlot.kSlot0)
        .maxOutput(1)
        .minOutput(-1)
        .maxMotion
        .cruiseVelocity(10000)
        .maxAcceleration(12000)
        .allowedProfileError(20);
    HOOD_CONFIG
        .sparkConfig
        .softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(HOOD_MAX_ANGLE)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(HOOD_MIN_ANGLE); // TODO: if hood doesnt move change min for max and max for min
    // HOO+D_CONFIG.kMaxPositionUnits = HOOD_MAX_ANGLE; //TODO: uncomment if u want
    // HOOD_CONFIG.kMinPositionUnits = HOOD_MIN_ANGLE;
    HOOD_CONFIG.unitToRotorRatio = 1/0.74955908298; // this * rotor = 1 hood degree

    HOOD_CONFIG.usingAbsoluteEncoder = 
        false; // TODO: may not be neccesary, built-in encoder sufficient
    // HOOD_CONFIG.absoluteEncoderToRotorRatio = 0.05929411764; // this amount of encoder rotations for 1 shaft rotation
    // 1/that amount = motor rotation count per 1 encoder rotation
    // 0.74955908298 motor rotations is 1 hood degree
    // 1 motor rotations is 1.3341176469 hood degrees

    HOOD_CONFIG.momentOfInertia = 0.1; // TODO: set MOI
  }
}
