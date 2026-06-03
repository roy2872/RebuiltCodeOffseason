package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.subsystems.MotorSubsystemConfig;

public class ClimberConstants {
  // climb has to start at highest point
  public static final double CLIMBER_MAX_POSITION_UNITS = 0.0; // Max extension in units
  public static final double CLIMBER_MIN_POSITION_UNITS = -65.0*1.7; // Min retraction in units TODO: also needs to be more

  // public static final double CLIMBER_OPEN_POSITION_UNITS = 49.0; // Top position for climbing
  public static final double CLIMBER_OPEN_POSITION_UNITS = -1.0;
  // public static final double CLIMBER_IDLE_POSITION_UNITS = 49.0;
  public static final double CLIMBER_IDLE_POSITION_UNITS = -1.0;
  // public static final double CLIMBER_CLOSED_POSITION_UNITS = 5.0;
  public static final double CLIMBER_CLOSED_POSITION_UNITS = -63.0*1.7; //TODO: needs to be more

  public static final double CLOSE_CLIMB_VOLTAGE = 3.0; // Voltage applied during climbing
  public static final double UNCLIMBING_VOLTAGE = 3.0; // Voltage to go down from climb
  public static final double OPEN_CLIMB_VOLTAGE = 3.0; // Voltage to open climb
  public static final double CLOSE_NO_CLIMB_VOLTAGE = 3.0; // Voltage to close climb

  public static final double RACHET_OPEN_SERVO_POSE = 10+45; // good angle
  public static final double RACHET_CLOSED_SERVO_POSE = 80+30; // should reduce to lower delay

  public static final double TOLERANCE = 0.03;

  public static final MotorSubsystemConfig CLIMBER_CONFIG = new MotorSubsystemConfig();

  static {
    CLIMBER_CONFIG.id = 59;
    CLIMBER_CONFIG.name = "Climber Motor";
    CLIMBER_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .inverted(true)
        .secondaryCurrentLimit(80);

    CLIMBER_CONFIG.unitToRotorRatio = 1.0;
    CLIMBER_CONFIG.usingAbsoluteEncoder = false;
    CLIMBER_CONFIG.momentOfInertia = 1.0;
  }
}
