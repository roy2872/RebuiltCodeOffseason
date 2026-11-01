package frc.lib.util.power;

/**
 * Anything the PowerManager can measure and current-limit, usually a subsystem.
 * MotorSubsystem already implements this, so most subsystems never touch it directly.
 */
public interface PowerConsumer {
	/** Name used for dashboard and log keys. */
	String getPowerName();

	/** Current this consumer is drawing from the battery right now, summed over all of its motors. */
	double getSupplyCurrentAmps();

	/**
	 * Sets the total current this consumer may draw, summed over all of its motors.
	 * Implementations split it between their motors. Only the current ceiling changes,
	 * the closed loop controllers keep running as configured.
	 */
	void setSupplyCurrentLimitAmps(double totalAmps);
}
