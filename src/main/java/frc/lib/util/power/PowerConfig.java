package frc.lib.util.power;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.TunableNumber;
import java.util.function.BooleanSupplier;

/**
 * Power settings for one consumer. Min and max are live-tunable from the dashboard
 * under TunableNumbers/Power/&lt;name&gt;/.
 */
public class PowerConfig {
	private final String name;
	private final PowerPriority priority;
	private final TunableNumber minAmps;
	private final TunableNumber maxAmps;
	private final TunableNumber autoMaxAmps;
	private BooleanSupplier overrideActive = () -> false;
	private PowerPriority overridePriority;
	private TunableNumber overrideMaxAmps;

	/**
	 * @param name Consumer name, used for the tunable keys.
	 * @param priority Shedding priority.
	 * @param minAmps Total current the consumer always gets, even when the battery is struggling.
	 * @param maxAmps Total current the consumer gets when there is plenty of power. Should match the
	 *     static current limits in the motor configs (per motor limit * number of motors).
	 */
	public PowerConfig(String name, PowerPriority priority, double minAmps, double maxAmps) {
		this(name, priority, minAmps, maxAmps, maxAmps);
	}

	/**
	 * Same as {@link #PowerConfig(String, PowerPriority, double, double)}, with a separate max for
	 * autonomous (auto is short and has nothing else pulling current, so it can afford more).
	 */
	public PowerConfig(String name, PowerPriority priority, double minAmps, double maxAmps, double autoMaxAmps) {
		this.name = name;
		this.priority = priority;
		this.minAmps = new TunableNumber("Power/" + name + "/MinAmps", minAmps);
		this.maxAmps = new TunableNumber("Power/" + name + "/MaxAmps", maxAmps);
		this.autoMaxAmps = new TunableNumber("Power/" + name + "/AutoMaxAmps", autoMaxAmps);
	}

	/**
	 * While {@code active} is true in teleop, the consumer uses {@code priority} and {@code maxAmps}
	 * instead of its normal ones (e.g. defense mode for the drive). Auto always keeps the normal ones.
	 */
	public PowerConfig withOverride(BooleanSupplier active, PowerPriority priority, double maxAmps) {
		overrideActive = active;
		overridePriority = priority;
		overrideMaxAmps = new TunableNumber("Power/" + name + "/OverrideMaxAmps", maxAmps);
		return this;
	}

	public boolean isOverridden() {
		return overridePriority != null && !DriverStation.isAutonomous() && overrideActive.getAsBoolean();
	}

	public PowerPriority getPriority() {
		return isOverridden() ? overridePriority : priority;
	}

	public double getMinAmps() {
		return Math.max(0.0, minAmps.getAsDouble());
	}

	public double getMaxAmps() {
		double max;
		if (isOverridden()) {
			max = overrideMaxAmps.getAsDouble();
		} else {
			max = DriverStation.isAutonomous() ? autoMaxAmps.getAsDouble() : maxAmps.getAsDouble();
		}
		return Math.max(getMinAmps(), max);
	}
}
