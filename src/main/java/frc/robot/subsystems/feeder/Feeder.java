package frc.robot.subsystems.feeder;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;

public class Feeder extends MotorSubsystem<SparkMaxConfig, MotorIOSparkMax> {

	public static final Setpoint<Voltage> FEED_VOLTAGE = Setpoint.withVoltageSetpoint(FeederConstants.FEED_VOLTAGE);
	public static final Setpoint<Voltage> SLOW_FEED_VOLTAGE =
			Setpoint.withVoltageSetpoint(FeederConstants.SLOW_FEED_VOLTAGE);
	public static final Setpoint<Voltage> IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint<AngularVelocity> REVERSE = Setpoint.withVelocitySetpoint(FeederConstants.REVERSE_VELOCITY);
	public static final Setpoint<Voltage> SLOW_REVERSE = Setpoint.withVoltageSetpoint(FeederConstants.SLOW_REVERSE_VOLTAGE);

	public static final Feeder mInstance = new Feeder();

	private Feeder() {
		super(FeederConstants.getMotorIO(), "Belt Drive");
	}
}