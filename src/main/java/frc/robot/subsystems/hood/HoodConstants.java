package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMaxSim;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.sim.ServoMechanismSim;
import frc.lib.sim.ServoMechanismSim.ServoMechanismSimConstants;
import frc.lib.visualization.VisualizedJoint;
import frc.lib.util.ControlGains.PidGains;
import frc.robot.Constants;
import frc.robot.Ports;

public class HoodConstants {

  // Hood angles are measured from vertical: the ball leaves at 90 degrees minus the hood angle.
  // Stowed is the steepest shot (84 degree launch); the hood travels 30 degrees from there.
  public static final Angle HOOD_STARTING_ANGLE = Degrees.of(6.0);
  public static final Angle HOOD_MIN_ANGLE = HOOD_STARTING_ANGLE;
  public static final Angle HOOD_MAX_ANGLE = HOOD_STARTING_ANGLE.plus(Degrees.of(30.0));
  public static final Angle HOOD_CLOSE_ANGLE = Degrees.of(8.0);
  public static final Angle HOOD_ANGLE_TOLERANCE = Degrees.of(0.3);

  // Motion profile. A NEO 550 (~11000 RPM free) through the gearing tops out near 245 deg/s at the
  // hood, so cruise at ~70% of that and reach it in 0.1 s. A full 6-36 deg move takes ~0.28 s.
  public static final AngularVelocity HOOD_CRUISE_VELOCITY = DegreesPerSecond.of(170.0);
  public static final AngularAcceleration HOOD_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(1700.0);

  // 3D visualization. Points are in the robot frame (meters, origin on the floor at robot center)
  // with the hood at HOOD_STARTING_ANGLE, which must be the pose the hood is exported in from CAD.
  public static final Translation3d VISUALIZATION_PIVOT = new Translation3d(-0.235, 0.0, 0.533); // Hood axle, at the back-bottom of the hood
  public static final Translation3d VISUALIZATION_TIP = new Translation3d(-0.04, 0.0, 0.55); // TODO: measure from CAD (sim side view only)
  // Positive rotation about +Y tips the front down. TODO: flip to Y_AXIS.unaryMinus() if the model moves the wrong way
  public static final Translation3d VISUALIZATION_AXIS = VisualizedJoint.Y_AXIS;
  // The CAD hood pose is this far above the real stowed pose, so the model is turned down by it
  public static final Angle VISUALIZATION_ZERO_OFFSET = Degrees.of(30.0);

  public static final Time BOOT_SEQUENCE_TIME = Seconds.of(2.0);
  public static final Voltage BOOT_SEQUENCE_VOLTAGE = Volts.of(0.5);

  public static final int HOOD_MOTOR_ID = Ports.HOOD.id;

  // Conversion ratio (mechanism rotations / motor rotations)
  public static final double GEAR_RATIO = (360.0 / 269.84127) / 360.0;

  public static final ArmFeedforward HOOD_FF =
      switch (Constants.currentMode) {
        case REAL, REPLAY -> new ArmFeedforward(0.0, 0.0, 0.0);
        case SIM -> new ArmFeedforward(0.0, 0.0, 0.0);
      };

  protected static final PidGains HOOD_PID =
      switch (Constants.currentMode) {
        case REAL, REPLAY -> new PidGains(1.5, 0.0, 0.2);
        case SIM -> new PidGains(30.0, 0.0, 0.0); // Error is in hood rotations, so this needs to be large
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
        .pid(HOOD_PID.kP, HOOD_PID.kI, HOOD_PID.kD, ClosedLoopSlot.kSlot0) // MAXMotion (e.g. STOWED)
        .pid(HOOD_PID.kP, HOOD_PID.kI, HOOD_PID.kD, ClosedLoopSlot.kSlot2); // Plain position setpoints

    config.closedLoop.maxMotion
        // MAXMotion units follow the velocity conversion factor: hood RPM and hood RPM per second
        .cruiseVelocity(HOOD_CRUISE_VELOCITY.in(RPM))
        .maxAcceleration(HOOD_MAX_ACCELERATION.in(RotationsPerSecondPerSecond) * 60.0)
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
    // Replay uses the real IO too: its inputs are overwritten from the log
    if (Constants.currentMode != Constants.Mode.SIM)
      return new MotorIOSparkMax(getIOConfig());
    else
      return new MotorIOSparkMaxSim(getIOConfig(), new ServoMechanismSim(getSimConstants()));
  }

  public static ServoMechanismSimConstants getSimConstants() {
    ServoMechanismSimConstants config = new ServoMechanismSimConstants();
    config.gearing = 1.0 / GEAR_RATIO; // The sim wants motor rotations per mechanism rotation
    config.maxAngle = HOOD_MAX_ANGLE;
    config.minAngle = HOOD_MIN_ANGLE;
    config.momentOfInertia = 0.05;
    config.motor = DCMotor.getNeo550(1);
    config.simulateGravity = false;
    config.startingAngle = HOOD_MIN_ANGLE;
    return config;
  }

  	public static ServoHomingConfig getServoConfig() {
		  ServoHomingConfig servoConfig = new ServoHomingConfig();
		  servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		  servoConfig.kHomingVoltage = Units.Volts.of(-1.5); // includes direction
		  servoConfig.kSetHomedVelocity = Units.DegreesPerSecond.of(5.0);
		  servoConfig.kHomePosition = HOOD_MIN_ANGLE;
		return servoConfig;
	}
}