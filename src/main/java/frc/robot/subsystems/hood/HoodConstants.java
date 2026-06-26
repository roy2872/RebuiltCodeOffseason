package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_STARTING_ANGLE;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.util.ControlGains.PidGains;
import frc.lib.util.ControlGains.SimpleFFConstants;
import frc.robot.Constants;

public class HoodConstants {

  public static final double HOOD_STARTING_ANGLE = 88; // [deg]
  public static final double HOOD_MIN_ANGLE = HOOD_STARTING_ANGLE-26; 
  public static final double HOOD_MAX_ANGLE = HOOD_STARTING_ANGLE-2; 
  public static final double BOOT_SEQUENCE_TIME = 2.0;
  public static final double BOOT_SEQUENCE_VOLTAGE = 0.5; // [V]
  public static final double HOOD_CLOSE_ANGLE = 85.0; // [deg]
  
  public static final double HOOD_ANGLE_TOLERANCE = 0.3; // [deg]

  public static final ArmFeedforward HOOD_FF =
      switch (Constants.currentMode) {
        case REAL -> new ArmFeedforward(0.0, 0.0, 0.0);
        case SIM -> new ArmFeedforward(0.0, 0.0, 0.0);
        default -> new ArmFeedforward(0.0, 0.0, 0.0);
      };

  protected static final PidGains HOOD_PID =
      switch (Constants.currentMode) {
        case REAL -> new PidGains(1.5, 0.0, 0.2);
        case SIM -> new PidGains(0.1, 0.0, 0.0);
        default -> new PidGains(0.1, 0.0, 0.0);
      };

  public static final MotorSubsystemConfig HOOD_CONFIG = new MotorSubsystemConfig();

  static {
    HOOD_CONFIG.name = "Hood";
    HOOD_CONFIG.id = 60;

    HOOD_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10) 
        .inverted(true)
        .secondaryCurrentLimit(20)
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
        .reverseSoftLimit(HOOD_MIN_ANGLE); 
    // HOOD_CONFIG.kMaxPositionUnits = HOOD_MAX_ANGLE; //TODO: uncomment if u want
    // HOOD_CONFIG.kMinPositionUnits = HOOD_MIN_ANGLE;
    HOOD_CONFIG.unitToRotorRatio = 1/0.74955908298; // this * rotor = 1 hood degree

    HOOD_CONFIG.usingAbsoluteEncoder = 
        false;

    HOOD_CONFIG.momentOfInertia = 0.1; // TODO: set MOI
  }
}
