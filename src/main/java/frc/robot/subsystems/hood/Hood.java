package frc.robot.subsystems.hood;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;

public class Hood extends ServoMotorSubsystem<SparkMaxConfig, MotorIOSparkMax> {

  // Preset Setpoints
  public static final Setpoint<Voltage> IDLE = Setpoint.withNeutralSetpoint();
  public static final Setpoint<Angle> MAX_ANGLE = Setpoint.withMotionMagicSetpoint(HoodConstants.HOOD_MAX_ANGLE);
  public static final Setpoint<Angle> STOWED_ANGLE = Setpoint.withMotionMagicSetpoint(HoodConstants.HOOD_MIN_ANGLE);
  public static final Setpoint<Voltage> BOOT_SEQUENCE = Setpoint.withVoltageSetpoint(HoodConstants.BOOT_SEQUENCE_VOLTAGE);

  public static final Hood mInstance = new Hood();

  public Hood() {
    super(
        HoodConstants.getMotorIO(),
        "Hood",
        HoodConstants.HOOD_ANGLE_TOLERANCE,
        false
    );
    setCurrentPosition(HoodConstants.HOOD_STARTING_ANGLE);
  }
}