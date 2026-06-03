package frc.robot.subsystems.hopper;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.subsystems.MotorSubsystemConfig;

public class HopperConstants {

  public static final double ACTIVE_VOLTAGE = 8.5;
  // public static final double ACTIVE_VOLTAGE = 9.5;
  public static final double UNJAM_REVERSE_TIME = 0.5;
  public static final double UNJAM_REVERSE_VOLTAGE = 8.0;
  public static final double DETECT_JAM_CURRENT = 80.5;
  public static final MotorSubsystemConfig HOPPER_CONFIG = new MotorSubsystemConfig();

  static {
    HOPPER_CONFIG.id = 53; 
    HOPPER_CONFIG.name = "Hopper";
    HOPPER_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .secondaryCurrentLimit(60)
        .inverted(true);

    HOPPER_CONFIG.usingAbsoluteEncoder = false;
    HOPPER_CONFIG.unitToRotorRatio =  1;
    HOPPER_CONFIG.momentOfInertia = 0.5;
  }
}
