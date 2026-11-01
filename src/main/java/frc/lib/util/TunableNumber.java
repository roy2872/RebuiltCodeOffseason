package frc.lib.util;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class creating a mutable object which can be mutated dynamically through SmartDashboard.
 *
 * <p>The dashboard value is logged, so replay sees the same values the robot used. To try a new
 * value in replay, change the default in code and set {@link #TUNING_ENABLED} to false: every
 * TunableNumber then returns its default and ignores the log.
 */
public class TunableNumber implements DoubleSupplier {
	/** When false, values are never read from the dashboard (or the log) and the defaults are used. */
	public static final boolean TUNING_ENABLED = true;

	private static final String TABLE_KEY = "/SmartDashboard/TunableNumbers";
	private final String key; // key associated with the object on SmartDashboard
	private double defaultValue; // default value
	private LoggedNetworkNumber dashboardNumber; // null when tuning is disabled
	private double lastCurrentValue; // last checked current value

	/**
	 * Constructs a new Tunable number
	 * @param smartDashboardKey key used in smartDashboard
	 * @param defaultValue
	 */
	public TunableNumber(String smartDashboardKey, double defaultValue) {
		key = TABLE_KEY + "/" + smartDashboardKey;
		publish(defaultValue);
		this.lastCurrentValue = defaultValue;
	}
	/**
	 * Gets the default value
	 * @return
	 */
	public double getDefault() {
		return defaultValue;
	}
	/**
	 * Sets the default value, which is published to SmartDashboard if it has no value yet
	 * @param valueToSet
	 */
	public void publish(double valueToSet) {
		defaultValue = valueToSet;
		if (!TUNING_ENABLED) {
			return;
		}
		if (dashboardNumber == null) {
			dashboardNumber = new LoggedNetworkNumber(key, valueToSet);
		} else {
			dashboardNumber.setDefault(valueToSet);
		}
	}
	/**
	 * Gets the current value on smartdashboard, as of the start of this loop
	 */
	@Override
	public double getAsDouble() {
		return TUNING_ENABLED ? dashboardNumber.get() : defaultValue;
	}
	/**
	 * Checks if the current value in smartdashboard has changed
	 * Returns true is the current value is different from the last current value
	 */
	public boolean hasChanged() {
		double currentValue = getAsDouble();
		if (currentValue != lastCurrentValue) {
			lastCurrentValue = currentValue;
			return true;
		}
		return false;
	}
}
