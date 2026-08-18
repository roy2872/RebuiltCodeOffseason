package frc.lib.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
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
     * Gets a multiplier to account for the motor inversion in the simulation state.
     *
     * @return -1.0 if output is inverted or 1.0 if not
     */
    private double getMotorInvertMultiplier() {
        // Query the inversion status via the modern configAccessor interface
        return main.configAccessor.getInverted() ? -1.0 : 1.0;
    }

    private void runSimulation() {
        // Retrieve applied output and bus voltage from the simulation instance
        double appliedVoltage = mainSim.getAppliedOutput() * mainSim.getBusVoltage();
        sim.setVoltage(Units.Volts.of(appliedVoltage).times(getMotorInvertMultiplier()));
        sim.simulate();
        updateSimStates();
    }

    @Override
    protected void updateMotorInputs(Inputs inputsToUpdate, SparkMax motor) {
        inputsToUpdate.position = sim.getPosition();
        inputsToUpdate.velocity = sim.getVelocity();
        inputsToUpdate.statorCurrent = sim.getStatorCurrent();
        
        // Feed the calculated simulation values back into the inputs tracking structure
        inputsToUpdate.supplyCurrent = 
                Units.Amps.of(mainSim.getMotorCurrent()).times(getMotorInvertMultiplier());
        inputsToUpdate.motorVoltage = 
                Units.Volts.of(mainSim.getAppliedOutput() * mainSim.getBusVoltage()).times(getMotorInvertMultiplier());
    }

    /**
     * Updates the main motor's and followers' simulation states.
     */
    private void updateSimStates() {
        mainSim.setBusVoltage(12.0);
        
        // Pass the physics loop iteration parameters: velocity (in RPS), voltage, and step time (dt)
        mainSim.iterate(
            sim.getVelocity().times(getMotorInvertMultiplier()).in(Units.RotationsPerSecond),
            mainSim.getBusVoltage(),
            0.005
        );

        for (SparkMaxSim followerSim : followerSims) {
            followerSim.setBusVoltage(12.0);
            followerSim.iterate(
                sim.getVelocity().times(getMotorInvertMultiplier()).in(Units.RotationsPerSecond),
                followerSim.getBusVoltage(),
                0.005
            );
        }
    }
}