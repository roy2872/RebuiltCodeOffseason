package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig.FollowerConfig;
import frc.lib.util.ControlGains.PidGains;
import frc.lib.util.ControlGains.SimpleFFConstants;
import frc.robot.Constants;

public class ShooterConstants {

  public static final SimpleFFConstants MAIN_WHEEL_FF_CONSTANTS =
      switch (Constants.currentMode) {
        case REAL -> new SimpleFFConstants(0.0, 0.1535*28.0/28.0, 0.53556);
        case SIM -> new SimpleFFConstants(0.0, 0.0, 0.0);
        default -> new SimpleFFConstants(0.0, 0.0, 0.0);
      };

  public static final SimpleFFConstants HOOD_WHEEL_FF_CONSTANTS =
      switch (Constants.currentMode) {
        case REAL -> new SimpleFFConstants(0.13343, 0.13258, 0.038279);
        case SIM -> new SimpleFFConstants(0.0, 0.0, 0.0);
        default -> new SimpleFFConstants(0.0, 0.0, 0.0);
      };

  private static final PidGains MAIN_PID =
      switch (Constants.currentMode) {
        case REAL -> new PidGains(0.002005, 0.0, 0.1); 
        case SIM -> new PidGains(0.1, 0.0, 0.0);
        default -> new PidGains(0.1, 0.0, 0.0);
      };

  private static final PidGains HOOD_PID =
      switch (Constants.currentMode) {
        case REAL -> new PidGains(0.001, 0.0, 0.05);
        case SIM -> new PidGains(0.1, 0.0, 0.0);
        default -> new PidGains(0.1, 0.0, 0.0);
      };

  public static final MotorSubsystemWithFollowersConfig MAIN_WHEEL_MOTOR_CONFIG = new MotorSubsystemWithFollowersConfig();
  public static final MotorSubsystemWithFollowersConfig HOOD_WHEEL_MOTOR_CONFIG = new MotorSubsystemWithFollowersConfig();

  public static final FollowerConfig MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG = new FollowerConfig();
  public static final FollowerConfig HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG = new FollowerConfig();

  public static final double MAIN_WHEEL_DIAMETER = Units.inchesToMeters(3); // meter
  public static final double HOOD_WHEEL_DIAMETER = Units.inchesToMeters(2); // meter

  public static final double COMPENSATION_FACTOR = 1.0;
  public static final double BACKSPIN_FACTOR = 1.0; 

  static {
    MAIN_WHEEL_MOTOR_CONFIG.name = "ShooterMainWheel";
    MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG.config.name = "ShooterMainWheelFollower";
    HOOD_WHEEL_MOTOR_CONFIG.name = "ShooterHoodWheel";
    HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG.config.name = "ShooterHoodWheelFollower";
    MAIN_WHEEL_MOTOR_CONFIG.id = 56;
    MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG.config.id = 57;
    HOOD_WHEEL_MOTOR_CONFIG.id = 58;
    HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG.config.id = 59;

    MAIN_WHEEL_MOTOR_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .inverted(true)
        .secondaryCurrentLimit(80)
        .closedLoop
        .pidf(
            MAIN_PID.kP, MAIN_PID.kI, MAIN_PID.kD, 0, ClosedLoopSlot.kSlot0)
        .maxMotion
        .cruiseVelocity(5000)
        .maxAcceleration(10000); // keep velocity ff 0

    HOOD_WHEEL_MOTOR_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .inverted(true)
        .secondaryCurrentLimit(80)
        .closedLoop
        .pidf(
            HOOD_PID.kP, HOOD_PID.kI, HOOD_PID.kD, 0, ClosedLoopSlot.kSlot0)
        .maxMotion
        .cruiseVelocity(5000)
        .maxAcceleration(10000); // keep velocity ff 0

    MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG.config.sparkConfig.apply(MAIN_WHEEL_MOTOR_CONFIG.sparkConfig).follow(56);
    HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG.config.sparkConfig.apply(HOOD_WHEEL_MOTOR_CONFIG.sparkConfig).follow(58);

    MAIN_WHEEL_MOTOR_CONFIG.unitToRotorRatio = 1.0 / 60.0; // TODO: set ratio
    HOOD_WHEEL_MOTOR_CONFIG.unitToRotorRatio = 1.0 / 60.0;

    MAIN_WHEEL_MOTOR_CONFIG.usingAbsoluteEncoder = false;
    HOOD_WHEEL_MOTOR_CONFIG.usingAbsoluteEncoder = false;

    MAIN_WHEEL_MOTOR_CONFIG.momentOfInertia = 0.1; // TODO: set MOI
    HOOD_WHEEL_MOTOR_CONFIG.momentOfInertia = 0.1;
    // HOOD_WHEEL_MOTOR_CONFIG.inverted = true;
    MAIN_WHEEL_MOTOR_CONFIG.followerConfigs = new FollowerConfig[]{MAIN_WHEEL_MOTOR_FOLLOWER_CONFIG};
    HOOD_WHEEL_MOTOR_CONFIG.followerConfigs = new FollowerConfig[]{HOOD_WHEEL_MOTOR_FOLLOWER_CONFIG};
  }
}
