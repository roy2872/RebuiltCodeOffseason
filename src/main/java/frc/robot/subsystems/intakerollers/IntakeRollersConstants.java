package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;

public class IntakeRollersConstants {
    public static final double GEARING = 1.0;

    public static final Voltage INTAKE_VOLTAGE = Volts.of(12.0);
    public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-12.0);

    public static SparkMaxConfig getMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(70);
        
        return config;
    }

    public static MotorIOSparkMaxConfig getIOConfig() {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.mainID = Ports.INTAKE_ROLLERS_MAIN.id;
        config.mainConfig = getMotorConfig();

        return config;
    }

    public static MotorIOSparkMax getMotorIO() {
        return new MotorIOSparkMax(getIOConfig());
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants config = new RollerSimConstants();
        config.gearing = GEARING;
        config.momentOfInertia = 0.03;
        config.motor = DCMotor.getNEO(1);
        return config;
    }
}
