package frc.lib.bases;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.TunableNumber;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.Logger;

/**
 * Base subsystem for any subsystem that uses motors.
 * * @param <C> The configuration class specific to the hardware library (e.g., SparkMaxConfig or TalonFXConfiguration).
 * @param <IO> The generic MotorIO interface managing that configuration.
 */
public class MotorSubsystem<C, IO extends MotorIO<C>> extends SubsystemBase {
    protected final IO io;
    protected final String name;
    protected boolean tuningMode = false;
    
    // Tunable Number Usage
    protected TunableNumber kP0;
    protected TunableNumber kI0;
    protected TunableNumber kD0;
    protected TunableNumber kV0;
    protected TunableNumber kS0;
    protected TunableNumber kG0;

    protected TunableNumber kP1;
    protected TunableNumber kI1;
    protected TunableNumber kD1;
    protected TunableNumber kV1;
    protected TunableNumber kS1;
    protected TunableNumber kG1;

    protected TunableNumber kP2;
    protected TunableNumber kI2;
    protected TunableNumber kD2;
    protected TunableNumber kV2;
    protected TunableNumber kS2;
    protected TunableNumber kG2;

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
     * Note: If your MotorIO abstract base class handles PID tracking across hardware via methods 
     * like getSlot0kP(), getSlot0kI(), etc., replace the direct hardware object calls below with those.
     * * If you prefer changing the hardware configuration blocks entirely out of this layer, 
     * this method can be overriden or delegated to a custom vendor-specific implementation.
     */
    public void tuning(String name) {
        if (DriverStation.isDisabled() && tuningMode) {
            // Placeholder: Accessing parameters through universal abstractions or reflections 
            // is best handled via base methods in MotorIO if direct dot notation is needed.
            kP0 = new TunableNumber(name + "kP0", 0.0);
            kI0 = new TunableNumber(name + "kI0", 0.0);
            kD0 = new TunableNumber(name + "kD0", 0.0);
            kV0 = new TunableNumber(name + "kV0", 0.0);
            kS0 = new TunableNumber(name + "kS0", 0.0);
            kG0 = new TunableNumber(name + "kG0", 0.0);

            kP1 = new TunableNumber(name + "kP1", 0.0);
            kI1 = new TunableNumber(name + "kI1", 0.0);
            kD1 = new TunableNumber(name + "kD1", 0.0);
            kV1 = new TunableNumber(name + "kV1", 0.0);
            kS1 = new TunableNumber(name + "kS1", 0.0);
            kG1 = new TunableNumber(name + "kG1", 0.0);

            kP2 = new TunableNumber(name + "kP2", 0.0);
            kI2 = new TunableNumber(name + "kI2", 0.0);
            kD2 = new TunableNumber(name + "kD2", 0.0);
            kV2 = new TunableNumber(name + "kV2", 0.0);
            kS2 = new TunableNumber(name + "kS2", 0.0);
            kG2 = new TunableNumber(name + "kG2", 0.0);
        } else {
            kP0 = null; kI0 = null; kD0 = null; kV0 = null; kS0 = null; kG0 = null;
            kP1 = null; kI1 = null; kD1 = null; kV1 = null; kS1 = null; kG1 = null;
            kP2 = null; kI2 = null; kD2 = null; kV2 = null; kS2 = null; kG2 = null;
        }
    }

    /**
     * Utilizes the Tunable Numbers across hardware generic boundaries.
     * Concrete implementations can override this if direct field mutators are used.
     */
    public void useTunableNumbers() {
        UnaryOperator<C> configChanger = (C config) -> {
            // Implement universal settings conversions or override in concrete device subsystems
            return config;
        };

        io.changeMainConfig(configChanger);
        io.changeFollowerConfig(configChanger);
    }

    @Override
    public void periodic() {
        io.updateInputs();
        io.logInputs(name + "/Inputs");
        Logger.recordOutput(name + "/Enabled", io.getEnabled());
        Logger.recordOutput(name + "/Setpoint/Mode", io.getSetpoint().mode);
        Logger.recordOutput(name + "/Setpoint/Value", io.getSetpointDoubleInUnits());
        outputTelemetry();
        if (DriverStation.isDisabled() && tuningMode) {
            if (kP0.hasChanged()
                    || kI0.hasChanged()
                    || kD0.hasChanged()
                    || kV0.hasChanged()
                    || kS0.hasChanged()
                    || kG0.hasChanged()
                    || kP1.hasChanged()
                    || kI1.hasChanged()
                    || kD1.hasChanged()
                    || kV1.hasChanged()
                    || kS1.hasChanged()
                    || kG1.hasChanged()
                    || kP2.hasChanged()
                    || kI2.hasChanged()
                    || kD2.hasChanged()
                    || kV2.hasChanged()
                    || kS2.hasChanged()
                    || kG2.hasChanged()) {
                useTunableNumbers();
            }
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
}
