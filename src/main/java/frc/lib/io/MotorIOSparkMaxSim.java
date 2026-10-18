package frc.lib.io;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import frc.lib.sim.MechanismSim;

/**
 * Class used to control a main SparkMax and any number of followers for a simulated mechanism.
 */
public class MotorIOSparkMaxSim extends MotorIOSparkMax {
    private final MechanismSim sim;
    private final Notifier notifier;
    
    // REVLib handles simulation via dedicated companion classes
    private final SparkMaxSim mainSim;
    private final SparkMaxSim[] followerSims;

    /**
     * Creates a MotorIOSparkMaxSim from a provided configuration.
     *
     * @param config Configuration to create MotorIOSparkMaxSim from.
     * @param simObject Object used to simulate mechanism.
     */
    public MotorIOSparkMaxSim(MotorIOSparkMaxConfig config, MechanismSim simObject) {
        super(config);
        this.sim = simObject;
        
        // Initialize simulation handles using a standard motor profile (e.g., NEO)
        this.mainSim = new SparkMaxSim(main, DCMotor.getNEO(1));
        this.followerSims = new SparkMaxSim[followers.length];
        for (int i = 0; i < followers.length; i++) {
            this.followerSims[i] = new SparkMaxSim(followers[i], DCMotor.getNEO(1));
        }

        notifier = new Notifier(this::runSimulation);
        notifier.startPeriodic(0.005);
    }

    /**
     * Applies the config right away. The real IO configures on a background thread, which in sim
     * races with startup code that reads the conversion factors (e.g. setCurrentPosition).
     */
    @Override
    public void applyConfig(SparkMax spark, SparkMaxConfig config) {
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // REV's sim works in the logical (already inverted) direction, so the applied output and encoder
    // need no inversion handling here.
    private void runSimulation() {
        double appliedVoltage = mainSim.getAppliedOutput() * mainSim.getBusVoltage();
        sim.setVoltage(Units.Volts.of(appliedVoltage));
        sim.simulate();
        updateSimStates();
    }

    /** Encoder position (after the Spark's conversion factor) for a mechanism angle. */
    private double toEncoderUnits(Angle mechanismPosition) {
        return sim.mechanismToRotor(mechanismPosition).in(Units.Rotations)
                * main.configAccessor.encoder.getPositionConversionFactor();
    }

    @Override
    protected void updateMotorInputs(Inputs inputsToUpdate, SparkMax motor) {
        // Same units the real Spark reports (encoder rotations after the conversion factor)
        inputsToUpdate.position = Units.Rotations.of(toEncoderUnits(sim.getPosition()));
        inputsToUpdate.velocity = Units.RotationsPerSecond.of(
                sim.mechanismToRotor(sim.getVelocity()).in(Units.RotationsPerSecond)
                        * main.configAccessor.encoder.getVelocityConversionFactor());
        inputsToUpdate.statorCurrent = sim.getStatorCurrent();
        inputsToUpdate.supplyCurrent = Units.Amps.of(mainSim.getMotorCurrent());
        inputsToUpdate.motorVoltage = Units.Volts.of(mainSim.getAppliedOutput() * mainSim.getBusVoltage());
    }

    /** Moves the simulated mechanism to the given position, so the physics and the Spark agree. */
    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        double encoderUnitsPerMechanismRotation = toEncoderUnits(Units.Rotations.of(1.0));
        sim.setState(
                Units.Rotations.of(mechanismPosition.in(Units.Rotations) / encoderUnitsPerMechanismRotation),
                sim.getVelocity());
        mainSim.setPosition(mechanismPosition.in(Units.Rotations));
    }

    /**
     * Updates the main motor's and followers' simulation states.
     */
    private void updateSimStates() {
        // iterate() expects velocity in encoder units per minute (RPM with a conversion factor of 1)
        double rotorRpm = sim.mechanismToRotor(sim.getVelocity()).in(Units.RPM);
        double velocity = rotorRpm * main.configAccessor.encoder.getVelocityConversionFactor();

        mainSim.setBusVoltage(12.0);
        mainSim.iterate(velocity, mainSim.getBusVoltage(), 0.005);
        // iterate() integrates position from a filtered, noisy velocity, which drifts. Keep the Spark's
        // encoder (used by its closed loop and soft limits) exactly on the physics position.
        mainSim.setPosition(toEncoderUnits(sim.getPosition()));

        for (SparkMaxSim followerSim : followerSims) {
            followerSim.setBusVoltage(12.0);
            followerSim.iterate(rotorRpm, followerSim.getBusVoltage(), 0.005);
        }
    }
}