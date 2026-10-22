package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.util.TunableNumber;
import frc.lib.util.power.PowerPriority;

public class Feeder extends MotorSubsystem<SparkMaxConfig, MotorIOSparkMax> {

	public static final Setpoint<Voltage> FEED_VOLTAGE = Setpoint.withVoltageSetpoint(FeederConstants.FEED_VOLTAGE);
	public static final Setpoint<Voltage> SLOW_FEED_VOLTAGE =
			Setpoint.withVoltageSetpoint(FeederConstants.SLOW_FEED_VOLTAGE);
	public static final Setpoint<Voltage> IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint<AngularVelocity> REVERSE = Setpoint.withVelocitySetpoint(FeederConstants.REVERSE_VELOCITY);
	public static final Setpoint<Voltage> SLOW_REVERSE = Setpoint.withVoltageSetpoint(FeederConstants.SLOW_REVERSE_VOLTAGE);

	public static final Feeder mInstance = new Feeder();

	// Feed voltage slider on the dashboard (TunableNumbers/Feeder/Feed Voltage)
	private final TunableNumber feedVoltage =
			new TunableNumber("Feeder/Feed Voltage", FeederConstants.FEED_VOLTAGE.in(Volts));

	private Feeder() {
		super(FeederConstants.getMotorIO(), "Feeder");
		// 3 motors x 30A smart limit
		enablePowerManagement(PowerPriority.HIGH, 30.0, 90.0);
	}

	/** Feed setpoint using the dashboard slider value, clamped to the allowed range. */
	public Setpoint<Voltage> getFeedVoltageSetpoint() {
		double volts = MathUtil.clamp(
				feedVoltage.getAsDouble(),
				FeederConstants.MIN_FEED_VOLTAGE.in(Volts),
				FeederConstants.MAX_FEED_VOLTAGE.in(Volts));
		return Setpoint.withVoltageSetpoint(Volts.of(volts));
	}
}