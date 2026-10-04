package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMaxSim;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.io.MotorIOTalonSRX; 
import frc.lib.io.MotorIOTalonSRX.MotorIOTalonSRXConfig;

import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class IntakeRollersConstants {
    public static final double GEARING = 1.0;

    public static final Voltage INTAKE_VOLTAGE = Volts.of(10.0);
    public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-10.0);

    // ==========================================
    // SPARK MAX METHODS
    // ==========================================

    public static SparkMaxConfig getMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(30)
            .secondaryCurrentLimit(40);
        
        return config;
    }

    public static MotorIOSparkMaxConfig getIOConfig() {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.mainID = Ports.INTAKE_ROLLERS_MAIN.id;
        config.mainConfig = getMotorConfig();

        return config;
    }

    public static MotorIOSparkMax getMotorIO() {
        if(Robot.isReal())
            return new MotorIOSparkMax(getIOConfig());
        else
            return new MotorIOSparkMaxSim(getIOConfig(), new RollerSim(getSimConstants()));
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants config = new RollerSimConstants();
        config.gearing = GEARING;
        config.momentOfInertia = 0.03;
        config.motor = DCMotor.getNEO(1);
        return config;
    }

    // ==========================================
    // TALON SRX METHODS
    // ==========================================

    public static TalonSRXConfiguration getMotorConfigTalonSRX() {
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        
        config.continuousCurrentLimit = 40; 
        config.peakCurrentLimit = 70;
        config.peakCurrentDuration = 100; // milliseconds before dropping to continuous
        return config;
    }

    public static MotorIOTalonSRXConfig getIOConfigTalonSRX() {
        MotorIOTalonSRXConfig config = new MotorIOTalonSRXConfig();
        config.mainID = Ports.INTAKE_ROLLERS_MAIN.id;
        config.mainConfig = getMotorConfigTalonSRX();

        return config;
    }

    public static MotorIOTalonSRX getMotorIOTalonSRX() {
        return new MotorIOTalonSRX(getIOConfigTalonSRX()).invert(false);
    }

    public static RollerSimConstants getSimConstantsTalonSRX() {
        RollerSimConstants config = new RollerSimConstants();
        config.gearing = GEARING;
        config.momentOfInertia = 0.03;
        config.motor = DCMotor.getVex775Pro(1); 
        return config;
    }
}