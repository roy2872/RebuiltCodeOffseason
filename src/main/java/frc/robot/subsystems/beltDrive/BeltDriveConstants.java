package frc.robot.subsystems.beltDrive;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig.FollowerConfig;

public class BeltDriveConstants {

  public static final double ACTIVE_VOLTAGE = 12.0;
  public static final double PURGE_VOLTAGE = -6.0;
  public static final MotorSubsystemWithFollowersConfig BELT_DRIVE_CONFIG = new MotorSubsystemWithFollowersConfig();
  public static final FollowerConfig BELT_FOLLOWER_CONFIG = new FollowerConfig();
  public static final FollowerConfig BELT_FOLLOWER_CONFIG_LEFT = new FollowerConfig();

  static {
    BELT_DRIVE_CONFIG.id = 55;
  
    BELT_DRIVE_CONFIG.name = "Belt Drive Top";
    BELT_DRIVE_CONFIG
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .secondaryCurrentLimit(40)
        .inverted(false);

    BELT_DRIVE_CONFIG.usingAbsoluteEncoder = false;
    BELT_DRIVE_CONFIG.unitToRotorRatio = 1.0;
    BELT_DRIVE_CONFIG.momentOfInertia = 0.5;

    BELT_FOLLOWER_CONFIG.config.id = 54;
    BELT_FOLLOWER_CONFIG.config.name = "Belt Drive Bottom";
    BELT_FOLLOWER_CONFIG.config
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .secondaryCurrentLimit(40)
        .inverted(false)
        .follow(55);
  
    BELT_FOLLOWER_CONFIG.config.usingAbsoluteEncoder = false;
    BELT_FOLLOWER_CONFIG.config.unitToRotorRatio = 1.0;
    BELT_FOLLOWER_CONFIG.config.momentOfInertia = 0.5;


    BELT_FOLLOWER_CONFIG.config.id = 53;
    BELT_FOLLOWER_CONFIG.config.name = "Belt Drive Left";
    BELT_FOLLOWER_CONFIG.config
        .sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .secondaryCurrentLimit(40)
        .inverted(false)
        .follow(55);
  
    BELT_FOLLOWER_CONFIG.config.usingAbsoluteEncoder = false;
    BELT_FOLLOWER_CONFIG.config.unitToRotorRatio = 1.0;
    BELT_FOLLOWER_CONFIG.config.momentOfInertia = 0.5;
    
    
    BELT_DRIVE_CONFIG.followerConfigs = new FollowerConfig[]{BELT_FOLLOWER_CONFIG, BELT_FOLLOWER_CONFIG_LEFT};
  }
}
