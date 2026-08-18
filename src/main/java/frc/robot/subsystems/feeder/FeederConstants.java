package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMaxSim;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class FeederConstants {

    public static final double GEARING = 1.0/4;
    // Voltage Setpoints
    public static final Voltage FEED_VOLTAGE = Volts.of(8.0);
    public static final Voltage SLOW_FEED_VOLTAGE = Volts.of(4.0); // TODO: Adjust as needed
    public static final Voltage SLOW_REVERSE_VOLTAGE = Volts.of(-4.0); // TODO: Adjust as needed

    // Velocity Setpoint
    public static final AngularVelocity REVERSE_VELOCITY = RPM.of(-1500.0); // TODO: Adjust as needed

    // CAN IDs
    public static final int BOTTOM_FOLLOWER_ID = 54;
    public static final int LEFT_FOLLOWER_ID = 53;

    /**
     * Creates the master SparkMaxConfig for the main Belt Drive motor.
     */
    public static SparkMaxConfig getMainConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)
            .secondaryCurrentLimit(40)
            .inverted(true);

        return config;
    }

    /**
     * Builds the complete MotorIOSparkMaxConfig mapping all main and follower motors.
     */
    public static MotorIOSparkMaxConfig getIOConfig() {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.mainID = Ports.FEEDER_ROLLERS_MAIN.id;
        config.mainConfig = getMainConfig();

        config.followerIDs = new int[] {
            Ports.FEEDER_ROLLERS_FOLLOWER1.id, 
            Ports.FEEDER_ROLLERS_FOLLOWER2.id
        };

        // Bottom follower (54) follow(55) -> Aligned
        // Left follower (53) follow(55, true) -> Opposed
        config.followerAlignment = new MotorAlignmentValue[] {
            MotorAlignmentValue.Aligned,
            MotorAlignmentValue.Opposed
        };

        config.followerConfig = getMainConfig();

        return config;
    }

    public static MotorIOSparkMax getMotorIO() {
        if(Robot.isReal())
            return new MotorIOSparkMax(getIOConfig());
        else
            return new MotorIOSparkMaxSim(getIOConfig(), new RollerSim(getSimConstants()));
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants simConstants = new RollerSimConstants();
        simConstants.motor = DCMotor.getNeo550(3);
        simConstants.gearing = GEARING;
        simConstants.momentOfInertia = 0.0166570492;
        return simConstants;
    }
}