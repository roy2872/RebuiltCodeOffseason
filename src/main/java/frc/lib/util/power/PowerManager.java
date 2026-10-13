package frc.lib.util.power;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Shares the battery's current between subsystems to keep the robot out of brownout.
 *
 * <p>Every loop it estimates how much total current the battery can supply before its voltage
 * drops below {@code TargetVoltage}, using a simple battery model {@code V = V_rest - I * R}:
 * <pre>budget = (V_rest - V_target) / R = I_measured + (V_measured - V_target) / R</pre>
 * While the consumers draw less than the budget everyone keeps their max limit (limits are
 * ceilings, mechanisms rarely all peak together). Once the measured draw goes over the budget,
 * which is the same as the voltage dropping under the target, the budget is handed out by
 * {@link PowerPriority}: every consumer gets its minimum, and what is left fills the CRITICAL
 * consumers to their maximum first, then HIGH, MEDIUM and LOW. Inside one priority the remainder
 * is shared in proportion to each consumer's range.
 *
 * <p>Limits drop instantly when power is short and recover slowly ({@code RecoveryAmpsPerSec}),
 * so the robot doesn't oscillate in and out of limiting. Limits are only sent to the motor
 * controllers when they change meaningfully, to keep CAN traffic low.
 *
 * <p>Only current limits change. Closed loop controllers (PID, motion magic) keep running.
 *
 * <p>Usage: register consumers (MotorSubsystem does it through {@code enablePowerManagement}),
 * optionally pass the PDH with {@link #usePowerDistribution}, and call {@link #periodic()} once
 * per loop after the CommandScheduler.
 */
public class PowerManager {
	public static final PowerManager mInstance = new PowerManager();

	/** Coarse state for drivers. Not used for control. */
	public enum PowerState {
		/** Nothing is limited. */
		NORMAL,
		/** Some MEDIUM/LOW consumers are limited. */
		LIMITING,
		/** CRITICAL/HIGH consumers are limited, or the voltage is under target. */
		CRITICAL
	}

	private static final String TABLE = "Power/";
	/** Limits closer than this to the last sent limit are not resent. */
	private static final double LIMIT_DEADBAND_AMPS = 3.0;
	/** Minimum time between two limit updates to the same consumer. */
	private static final double MIN_UPDATE_PERIOD_SECONDS = 0.1;
	/** Voltage the dashboard treats as a full battery when computing headroom. */
	private static final double NOMINAL_VOLTAGE = 12.0;

	private final TunableNumber enabled = new TunableNumber("Power/Enabled", 1.0);
	private final TunableNumber targetVoltage = new TunableNumber("Power/TargetVoltage", 8.0);
	private final TunableNumber batteryResistanceOhms = new TunableNumber("Power/BatteryResistanceOhms", 0.020);
	private final TunableNumber maxBudgetAmps = new TunableNumber("Power/MaxBudgetAmps", 400.0);
	private final TunableNumber recoveryAmpsPerSec = new TunableNumber("Power/RecoveryAmpsPerSec", 40.0);
	private final TunableNumber lowBatteryVolts = new TunableNumber("Power/LowBatteryVolts", 12.3);

	private final List<Entry> entries = new ArrayList<>();
	private final LinearFilter voltageFilter = LinearFilter.movingAverage(5);
	private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
	private PowerDistribution powerDistribution = null;
	private boolean filtersPrimed = false;
	private boolean shedding = false;

	private final Alert lowBatteryAlert = new Alert("", AlertType.kWarning);
	private final Alert limitingAlert = new Alert("", AlertType.kWarning);
	private final Alert brownoutAlert = new Alert("", AlertType.kError);

	private double lastTimestamp = Timer.getFPGATimestamp();
	private double lastVoltage = Double.NaN;
	private double lastCurrent = Double.NaN;
	private double estimatedResistance = Double.NaN;
	private double minVoltageSinceEnable = Double.POSITIVE_INFINITY;
	private boolean wasEnabled = false;
	private boolean wasBrownedOut = false;
	private int brownoutCount = 0;
	private PowerState state = PowerState.NORMAL;

	private static class Entry {
		final PowerConsumer consumer;
		final PowerConfig config;
		double target;
		double allocated;
		double applied = Double.NaN;
		double lastApplyTimestamp = Double.NEGATIVE_INFINITY;

		Entry(PowerConsumer consumer, PowerConfig config) {
			this.consumer = consumer;
			this.config = config;
			this.target = config.getMaxAmps();
			this.allocated = target;
		}

		boolean isLimited() {
			return allocated < config.getMaxAmps() - LIMIT_DEADBAND_AMPS;
		}
	}

	private PowerManager() {}

	/** Adds a consumer to be measured and current-limited. */
	public void register(PowerConsumer consumer, PowerConfig config) {
		entries.add(new Entry(consumer, config));
	}

	/**
	 * Uses a PDH/PDP to measure total robot current. Without one, the manager sums the
	 * consumers' own measurements, which misses unmanaged loads (roboRIO, radio, steer motors...).
	 */
	public void usePowerDistribution(PowerDistribution powerDistribution) {
		this.powerDistribution = powerDistribution;
	}

	public PowerState getState() {
		return state;
	}

	public void periodic() {
		double now = Timer.getFPGATimestamp();
		double dt = MathUtil.clamp(now - lastTimestamp, 0.0, 0.1);
		lastTimestamp = now;

		// Measure
		double consumersAmps = 0.0;
		for (Entry entry : entries) {
			consumersAmps += Math.abs(entry.consumer.getSupplyCurrentAmps());
		}
		double rawCurrent = powerDistribution != null ? powerDistribution.getTotalCurrent() : consumersAmps;
		if (RobotBase.isSimulation()) {
			RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(rawCurrent));
		}
		double rawVoltage = RobotController.getBatteryVoltage();
		if (!filtersPrimed) {
			// The filters start full of zeros, which would read as a dead battery for the first loops
			voltageFilter.reset(new double[] {rawVoltage, rawVoltage, rawVoltage, rawVoltage, rawVoltage}, new double[0]);
			currentFilter.reset(new double[] {rawCurrent, rawCurrent, rawCurrent, rawCurrent, rawCurrent}, new double[0]);
			filtersPrimed = true;
		}
		double voltage = voltageFilter.calculate(rawVoltage);
		double current = currentFilter.calculate(rawCurrent);
		updateResistanceEstimate(voltage, current);

		// Budget from the battery model
		double resistance = Math.max(1e-3, batteryResistanceOhms.getAsDouble());
		double vTarget = targetVoltage.getAsDouble();
		double budget = current + (voltage - vTarget) / resistance;
		double unmanagedAmps = Math.max(0.0, current - consumersAmps);
		double minTotal = 0.0;
		for (Entry entry : entries) {
			minTotal += entry.config.getMinAmps();
		}
		budget = MathUtil.clamp(budget - unmanagedAmps, minTotal, Math.max(minTotal, maxBudgetAmps.getAsDouble()));

		// Allocate and apply. Limits are ceilings, so their sum may exceed the budget as long as
		// the actual draw doesn't, shedding only starts once the measured draw goes over budget.
		boolean managing = enabled.getAsDouble() > 0.5;
		shedding = managing && consumersAmps > budget;
		if (shedding) {
			allocate(budget);
		} else {
			for (Entry entry : entries) {
				entry.target = entry.config.getMaxAmps();
			}
		}
		for (Entry entry : entries) {
			slewAndApply(entry, now, dt);
		}

		updateState(voltage, vTarget);
		updateEvents(rawVoltage);
		publish(voltage, current, budget, vTarget, managing);
	}

	/** Water-fill: everyone gets their min, then priorities are filled in order up to their max. */
	private void allocate(double budget) {
		double available = budget;
		for (Entry entry : entries) {
			entry.target = entry.config.getMinAmps();
			available -= entry.target;
		}
		for (PowerPriority priority : PowerPriority.values()) {
			double tierRange = 0.0;
			for (Entry entry : entries) {
				if (entry.config.priority == priority) {
					tierRange += entry.config.getMaxAmps() - entry.config.getMinAmps();
				}
			}
			if (tierRange <= 0.0) {
				continue;
			}
			double share = MathUtil.clamp(available / tierRange, 0.0, 1.0);
			for (Entry entry : entries) {
				if (entry.config.priority == priority) {
					entry.target += (entry.config.getMaxAmps() - entry.config.getMinAmps()) * share;
				}
			}
			available -= tierRange * share;
		}
	}

	/** Drops instantly, recovers at RecoveryAmpsPerSec, and only resends meaningful changes. */
	private void slewAndApply(Entry entry, double now, double dt) {
		if (entry.target < entry.allocated) {
			entry.allocated = entry.target;
		} else {
			entry.allocated = Math.min(entry.target, entry.allocated + recoveryAmpsPerSec.getAsDouble() * dt);
		}

		// Cuts are sent right away, increases are rate limited to keep CAN traffic down
		boolean firstApply = Double.isNaN(entry.applied);
		boolean bigCut = entry.applied - entry.allocated >= LIMIT_DEADBAND_AMPS;
		boolean bigRaise = entry.allocated - entry.applied >= LIMIT_DEADBAND_AMPS;
		boolean reachedMax = entry.allocated >= entry.config.getMaxAmps() && entry.applied != entry.allocated;
		boolean rateOk = now - entry.lastApplyTimestamp >= MIN_UPDATE_PERIOD_SECONDS;
		if (firstApply || bigCut || ((bigRaise || reachedMax) && rateOk)) {
			entry.consumer.setSupplyCurrentLimitAmps(entry.allocated);
			entry.applied = entry.allocated;
			entry.lastApplyTimestamp = now;
		}
	}

	/** Logs R = -dV/dI from large current steps, so BatteryResistanceOhms can be calibrated from match logs. */
	private void updateResistanceEstimate(double voltage, double current) {
		if (!Double.isNaN(lastVoltage)) {
			double deltaCurrent = current - lastCurrent;
			if (Math.abs(deltaCurrent) > 20.0) {
				double sample = MathUtil.clamp(-(voltage - lastVoltage) / deltaCurrent, 0.0, 0.1);
				estimatedResistance = Double.isNaN(estimatedResistance) ? sample : 0.9 * estimatedResistance + 0.1 * sample;
			}
		}
		lastVoltage = voltage;
		lastCurrent = current;
	}

	private void updateState(double voltage, double vTarget) {
		boolean anyLimited = false;
		boolean importantLimited = false;
		for (Entry entry : entries) {
			if (entry.isLimited()) {
				anyLimited = true;
				importantLimited |= entry.config.priority.compareTo(PowerPriority.HIGH) <= 0;
			}
		}
		if (importantLimited || voltage < vTarget) {
			state = PowerState.CRITICAL;
		} else if (anyLimited) {
			state = PowerState.LIMITING;
		} else {
			state = PowerState.NORMAL;
		}
	}

	private void updateEvents(double rawVoltage) {
		boolean robotEnabled = DriverStation.isEnabled();
		if (robotEnabled && !wasEnabled) {
			minVoltageSinceEnable = Double.POSITIVE_INFINITY;
		}
		wasEnabled = robotEnabled;
		if (robotEnabled) {
			minVoltageSinceEnable = Math.min(minVoltageSinceEnable, rawVoltage);
		} else {
			// Only judge the battery while resting, voltage always sags while driving
			double lowVolts = lowBatteryVolts.getAsDouble();
			lowBatteryAlert.setText(String.format("Low battery: %.2fV (swap below %.1fV)", rawVoltage, lowVolts));
			lowBatteryAlert.set(rawVoltage < lowVolts);
		}

		boolean brownedOut = RobotController.isBrownedOut();
		if (brownedOut && !wasBrownedOut) {
			brownoutCount++;
		}
		wasBrownedOut = brownedOut;
		brownoutAlert.setText("Brownout occurred (" + brownoutCount + " since boot)");
		brownoutAlert.set(brownoutCount > 0);

		StringBuilder limited = new StringBuilder();
		for (Entry entry : entries) {
			if (entry.isLimited()) {
				limited.append(limited.length() == 0 ? "" : ", ").append(entry.consumer.getPowerName());
			}
		}
		limitingAlert.setText("Power limiting active: " + limited);
		limitingAlert.set(limited.length() > 0);
	}

	private void publish(double voltage, double current, double budget, double vTarget, boolean managing) {
		double headroom = MathUtil.clamp((voltage - vTarget) / (NOMINAL_VOLTAGE - vTarget), 0.0, 1.0) * 100.0;

		// Small driver-facing set
		SmartDashboard.putNumber(TABLE + "Battery V", voltage);
		SmartDashboard.putNumber(TABLE + "Total A", current);
		SmartDashboard.putNumber(TABLE + "Budget A", budget);
		SmartDashboard.putNumber(TABLE + "Headroom %", headroom);
		SmartDashboard.putString(TABLE + "State", state.name());
		SmartDashboard.putBoolean(TABLE + "Power OK", state == PowerState.NORMAL);
		SmartDashboard.putNumber(TABLE + "Brownouts", brownoutCount);

		// Full detail for logs
		Logger.recordOutput(TABLE + "Enabled", managing);
		Logger.recordOutput(TABLE + "Shedding", shedding);
		Logger.recordOutput(TABLE + "State", state);
		Logger.recordOutput(TABLE + "BatteryVoltage", voltage);
		Logger.recordOutput(TABLE + "TotalCurrentAmps", current);
		Logger.recordOutput(TABLE + "BudgetAmps", budget);
		Logger.recordOutput(TABLE + "HeadroomPercent", headroom);
		Logger.recordOutput(TABLE + "EstimatedResistanceOhms", Double.isNaN(estimatedResistance) ? 0.0 : estimatedResistance);
		Logger.recordOutput(TABLE + "MinVoltageSinceEnable",
				Double.isInfinite(minVoltageSinceEnable) ? voltage : minVoltageSinceEnable);
		Logger.recordOutput(TABLE + "BrownoutCount", brownoutCount);
		for (Entry entry : entries) {
			String key = TABLE + "Consumers/" + entry.consumer.getPowerName() + "/";
			Logger.recordOutput(key + "DrawAmps", Math.abs(entry.consumer.getSupplyCurrentAmps()));
			Logger.recordOutput(key + "TargetAmps", entry.target);
			Logger.recordOutput(key + "LimitAmps", entry.allocated);
			Logger.recordOutput(key + "Limited", entry.isLimited());
		}
	}
}
