package frc.lib.util.power;

import frc.lib.util.TunableNumber;

/**
 * Power settings for one consumer. Min and max are live-tunable from the dashboard
 * under TunableNumbers/Power/&lt;name&gt;/.
 */
public class PowerConfig {
	public final PowerPriority priority;
	private final TunableNumber minAmps;
	private final TunableNumber maxAmps;

	/**
	 * @param name Consumer name, used for the tunable keys.
	 * @param priority Shedding priority.
	 * @param minAmps Total current the consumer always gets, even when the battery is struggling.
	 * @param maxAmps Total current the consumer gets when there is plenty of power. Should match the
	 *     static current limits in the motor configs (per motor limit * number of motors).
	 */
	public PowerConfig(String name, PowerPriority priority, double minAmps, double maxAmps) {
		this.priority = priority;
		this.minAmps = new TunableNumber("Power/" + name + "/MinAmps", minAmps);
		this.maxAmps = new TunableNumber("Power/" + name + "/MaxAmps", maxAmps);
	}

	public double getMinAmps() {
		return Math.max(0.0, minAmps.getAsDouble());
	}

	public double getMaxAmps() {
		return Math.max(getMinAmps(), maxAmps.getAsDouble());
	}
}
