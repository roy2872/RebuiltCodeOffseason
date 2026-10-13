package frc.lib.util.power;

/**
 * How important a power consumer is when the battery can't feed everything.
 * The PowerManager fills the budget in declaration order, so CRITICAL consumers
 * are the last to lose current and LOW consumers are the first.
 */
public enum PowerPriority {
	/** Must keep working no matter what (e.g. the drivetrain). */
	CRITICAL,
	/** Scoring mechanisms that should stay fast (e.g. the shooter). */
	HIGH,
	/** Supporting mechanisms (e.g. feeder, hood). */
	MEDIUM,
	/** Can be slowed down first (e.g. intake). */
	LOW
}
