package frc.lib.bases;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.TunableNumber;
import frc.lib.util.power.PowerConfig;
import frc.lib.util.power.PowerConsumer;
import frc.lib.util.power.PowerManager;
import frc.lib.util.power.PowerPriority;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Base subsystem for any subsystem that uses motors.
 * * @param <C> The configuration class specific to the hardware library (e.g., SparkMaxConfig or TalonFXConfiguration).
 * @param <IO> The generic MotorIO interface managing that configuration.
 */
public class MotorSubsystem<C, IO extends MotorIO<C>> extends SubsystemBase implements PowerConsumer {
    protected final IO io;
    protected final String name;
    protected boolean tuningMode = false;
    
    // Tunable gains, one row per PID slot: kP, kI, kD, kV, kS, kG
    private static final int TUNABLE_SLOTS = 3;
    private static final String[] GAIN_NAMES = {"kP", "kI", "kD", "kV", "kS", "kG"};
    protected TunableNumber[][] tunableGains = null;

    /**
     * Creates a MotorSubsystem with a MotorIO and name for telemetry.
     * @param io MotorIO for the subsystem.
     * @param name Name for telemetry.
     * @param tuningMode Boolean determining whether the user wants to enable tuning on the fly
     */
    public MotorSubsystem(IO io, String name, boolean tuningMode) {
        super(name);
        this.io = io;
        this.name = name;
        this.tuningMode = tuningMode; // This should never be on for subsystems without PID
        tuning(name);
    }

    public MotorSubsystem(IO io, String name) {
        this(io, name, false);
    }

    /**
     * Publishes the gains of every slot as TunableNumbers under "TunableNumbers/<name>/Slot<n>/",
     * starting from the gains configured on the motor.
     */
    public void tuning(String name) {
        if (!tuningMode) {
            tunableGains = null;
            return;
        }
        tunableGains = new TunableNumber[TUNABLE_SLOTS][];
        for (int slot = 0; slot < TUNABLE_SLOTS; slot++) {
            MotorIO.Gains gains = io.getGains(slot);
            double[] values = {gains.kP(), gains.kI(), gains.kD(), gains.kV(), gains.kS(), gains.kG()};
            tunableGains[slot] = new TunableNumber[GAIN_NAMES.length];
            for (int i = 0; i < GAIN_NAMES.length; i++) {
                tunableGains[slot][i] = new TunableNumber(name + "/Slot" + slot + "/" + GAIN_NAMES[i], values[i]);
            }
        }
    }

    /** Sends the tuned gains of every slot that changed on the dashboard to the motor. */
    public void useTunableNumbers() {
        for (int slot = 0; slot < TUNABLE_SLOTS; slot++) {
            TunableNumber[] g = tunableGains[slot];
            boolean changed = false;
            for (TunableNumber gain : g) {
                // No short circuit, every gain has to update its last value
                changed |= gain.hasChanged();
            }
            if (changed) {
                io.setGains(slot, new MotorIO.Gains(
                        g[0].getAsDouble(), g[1].getAsDouble(), g[2].getAsDouble(),
                        g[3].getAsDouble(), g[4].getAsDouble(), g[5].getAsDouble()));
            }
        }
    }

    @Override
    public void periodic() {
        io.updateInputs();
        io.logInputs(name + "/Inputs");
        Logger.recordOutput(name + "/Enabled", io.getEnabled());
        Logger.recordOutput(name + "/Setpoint/Mode", io.getSetpoint().mode);
        // Logged in the same base units as the Inputs (rad, rad/s, V), so it lines up with them on a graph
        Unit setpointUnit = io.getSetpointBaseUnit();
        if (setpointUnit != null) {
            Logger.recordOutput(name + "/Setpoint/Value", io.getSetpoint().baseUnits, setpointUnit);
        } else {
            Logger.recordOutput(name + "/Setpoint/Value", io.getSetpoint().baseUnits);
        }
        outputTelemetry();
        if (tuningMode) {
            useTunableNumbers();
        }
    }

    public void outputTelemetry() {
        LoggedTracer.record(name);
    }

    public Angle getPosition() {
        return io.getPosition();
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    public Current getStatorCurrent() {
        return io.getStatorCurrent();
    }

    public Current getSupplyCurrent() {
        return io.getSupplyCurrent();
    }

    public Voltage getMotorVoltage() {
        return io.getMotorVoltage();
    }

    @SuppressWarnings("rawtypes")
    public Setpoint getSetpoint() {
        return io.getSetpoint();
    }

    @SuppressWarnings({"unchecked", "rawtypes"})
    public void applySetpoint(Setpoint setpoint) {
        io.applySetpoint(setpoint);
    }

    @SuppressWarnings("rawtypes")
    public Command setpointCommand(Setpoint setpoint) {
        return runOnce(() -> applySetpoint(setpoint));
    }

    @SuppressWarnings("rawtypes")
    public Command followSetpointCommand(Supplier<Setpoint> supplier) {
        return run(() -> applySetpoint(supplier.get()));
    }

    public void disable() {
        io.disable();
    }

    public void enable() {
        io.enable();
    }

    public Command disableCommand() {
        return Commands.runOnce(() -> io.disable());
    }

    public Command enableCommand() {
        return Commands.runOnce(() -> io.enable());
    }

    public IO getIO() {
        return io;
    }

    /**
     * Lets the PowerManager share battery current with this subsystem. Call once from the constructor.
     *
     * @param priority Who gets cut last when power is short.
     * @param minAmps Total current (all motors) this subsystem always keeps.
     * @param maxAmps Total current (all motors) at full power, should match the configured current limits.
     */
    protected void enablePowerManagement(PowerPriority priority, double minAmps, double maxAmps) {
        PowerManager.mInstance.register(this, new PowerConfig(name, priority, minAmps, maxAmps));
    }

    @Override
    public String getPowerName() {
        return name;
    }

    @Override
    public double getSupplyCurrentAmps() {
        return io.getTotalSupplyCurrent().in(Amps);
    }

    @Override
    public void setSupplyCurrentLimitAmps(double totalAmps) {
        io.setSupplyCurrentLimit(Amps.of(totalAmps / io.getMotorCount()));
    }
}
