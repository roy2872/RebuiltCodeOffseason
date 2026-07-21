package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.util.ControlGains.PidGains;
import frc.robot.Constants;

public class HoodConstants {

  // Strong-typed angle, voltage, and time units
  public static final Angle HOOD_STARTING_ANGLE = Degrees.of(84.0);
  public static final Angle HOOD_MIN_ANGLE = HOOD_STARTING_ANGLE;
  public static final Angle HOOD_MAX_ANGLE = HOOD_STARTING_ANGLE.minus(Degrees.of(30.0));
  public static final Angle HOOD_CLOSE_ANGLE = Degrees.of(82.0);
  public static final Angle HOOD_ANGLE_TOLERANCE = Degrees.of(0.3);

  public static final Time BOOT_SEQUENCE_TIME = Seconds.of(2.0);
  public static final Voltage BOOT_SEQUENCE_VOLTAGE = Volts.of(0.5);

  public static final int HOOD_MOTOR_ID = 60;

  // Conversion ratio (mechanism rotations / motor rotations)
  public static final double GEAR_RATIO = (360.0 / 269.84127) / 360.0;

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

  /**
   * Generates the configuration for the Spark Max hardware controller.
   */
  public static SparkMaxConfig getSparkConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10)
        .secondaryCurrentLimit(20)
        .inverted(true);

    config.encoder
        .positionConversionFactor(GEAR_RATIO) // Encodes directly to rotations at the mechanism
        .velocityConversionFactor(GEAR_RATIO);

    config.closedLoop
        .pidf(HOOD_PID.kP, HOOD_PID.kI, HOOD_PID.kD, 0.0, ClosedLoopSlot.kSlot0);

    config.closedLoop.maxMotion
        .cruiseVelocity(10000)
        .maxAcceleration(12000)
        .allowedProfileError(20);

    config.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(HOOD_MAX_ANGLE.in(Rotations))
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(HOOD_MIN_ANGLE.in(Rotations));

    return config;
  }

  /**
   * Builds the MotorIOSparkMaxConfig structure for hardware IO construction.
   */
  public static MotorIOSparkMaxConfig getIOConfig() {
    MotorIOSparkMaxConfig ioConfig = new MotorIOSparkMaxConfig();
    ioConfig.mainID = HOOD_MOTOR_ID;
    ioConfig.mainConfig = getSparkConfig();
    ioConfig.unit = Degrees;
    return ioConfig;
  }

  /**
   * Factory method to instantiate the motor IO instance used directly by Hood.java.
   */
  public static MotorIOSparkMax getMotorIO() {
    return new MotorIOSparkMax(getIOConfig());
  }
}