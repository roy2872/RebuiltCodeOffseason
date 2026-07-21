package frc.robot.subsystems.intakedeploy;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.util.ControlGains.PidGains;
import frc.robot.Constants;
import frc.robot.Ports;

public class IntakeDeployConstants {

  // Absolute encoder zero offset (in mechanism rotations or degrees depending on factor)
  public static final double ABSOLUTE_ENCODER_OFFSET = 0.0; // TODO: Measure with deploy at hard stop/stowed
  public static final boolean ABSOLUTE_ENCODER_INVERTED = false;

  // Strong-typed angle, voltage, and time units
  public static final Angle INTAKE_DEPLOYED_ANGLE = Degrees.of(84.0);
  public static final Angle INTAKE_PARTIAL_IN_ANGLE = INTAKE_DEPLOYED_ANGLE.minus(Degrees.of(30.0));
  public static final Angle INTAKE_STOWED_ANGLE = Degrees.of(82.0);
  public static final Angle INTAKE_ANGLE_TOLERANCE = Degrees.of(0.3);

  public static final int INTAKE_MOTOR_ID = 60;

  // Conversion ratio (mechanism rotations / motor rotations)
  public static final double GEAR_RATIO = (360.0 / 269.84127) / 360.0;

  public static final ArmFeedforward INTAKE_FF =
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
        .smartCurrentLimit(30)
        .secondaryCurrentLimit(40)
        .inverted(true);

    // 1. Configure Absolute Encoder Settings
    config.absoluteEncoder
        .positionConversionFactor(360.0) // Convert native rotations (0.0 to 1.0) directly to degrees
        .velocityConversionFactor(360.0 / 60.0) // RPM to Degrees per second
        .zeroOffset(ABSOLUTE_ENCODER_OFFSET)
        .inverted(ABSOLUTE_ENCODER_INVERTED);

    // 2. Direct Closed-Loop Control to use the Absolute Encoder
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Overrides default internal motor encoder
        .pid(HOOD_PID.kP, HOOD_PID.kI, HOOD_PID.kD, ClosedLoopSlot.kSlot0);

    config.closedLoop.maxMotion
        .cruiseVelocity(40)
        .maxAcceleration(100)
        .allowedProfileError(20);

    config.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(INTAKE_DEPLOYED_ANGLE.in(Degrees))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(INTAKE_STOWED_ANGLE.in(Degrees));

    return config;
  }

  public static MotorIOSparkMaxConfig getIOConfig() {
    MotorIOSparkMaxConfig ioConfig = new MotorIOSparkMaxConfig();
    ioConfig.mainID = Ports.INTAKE_DEPLOY.id;
    ioConfig.mainConfig = getSparkConfig();
    ioConfig.unit = Degrees;
    return ioConfig;
  }

  public static MotorIOSparkMax getMotorIO() {
    return new MotorIOSparkMax(getIOConfig());
  }
}