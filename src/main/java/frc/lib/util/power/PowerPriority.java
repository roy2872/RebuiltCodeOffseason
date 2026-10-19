package frc.lib.util.power;

/**
 * How important a power consumer is when the battery can't feed everything.
 * The PowerManager fills the budget in declaration order, so CRITICAL consumers
 * are the last to lose current and LOW consumers are the first.
 *
 * <p>Like 6328's FinanceDepartment, mechanisms are protected and the drivetrain takes what is left:
 * mechanisms only run in short bursts and need their full current while they do, while the drive
 * is the biggest load and still works fine on less.
 */
public enum PowerPriority {
	/** Must keep working no matter what (e.g. the shooter). */
	CRITICAL,
	/** Scoring path that should stay fast (e.g. feeder, hood). */
	HIGH,
	/** Supporting mechanisms (e.g. intake). */
	MEDIUM,
	/** Takes whatever is left, first to be limited (e.g. the drivetrain). */
	LOW
}
