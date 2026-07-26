package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig; // Assuming this wrapper exists similarly to your Talon setup
import frc.lib.io.MotorIOSparkMaxSim; // Assuming your simulator interface exists for Spark Max
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class ShooterConstants {

    public static final double kGearing = 1/1.0;

    public static final AngularVelocity kEpsilonThreshold = RPM.of(10.0);

    public static final Time VELOCITY_THRESHOLD_DEBOUNCE_TIME = Seconds.of(0.0);

    public static final AngularVelocity kTestShot = RPM.of(4000.0);
    public static final AngularVelocity kFerry = RPM.of(3000.0);
    public static final AngularVelocity kSlow = RPM.of(2100.0);

    public static SparkMaxConfig ShooterSparkConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Handle physical variables & limits
        config.idleMode(IdleMode.kCoast)
              .inverted(false) // Equivalent to CounterClockwise_Positive assuming right-hand configuration
              .smartCurrentLimit(30)
              .secondaryCurrentLimit(80); 

        // Slot 1: Active Velocity control slots
        double kP = 1;
        double kI = 0.0;
        double kD = 0.0;
        // Converting kV calculation format safely over to method chaining parameters
        double kV = ((12.0) / (5676.0) * kGearing) * 0.995;
                
        config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0).feedForward.apply(new FeedForwardConfig().kS(0.0).kV(kV).kA(0.0));

        // Slot 0: Flatlined defaults matching your Talon template
        config.closedLoop.pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1);

        // Max Motion constraints maps over from MotionMagic parameters
        config.closedLoop.maxMotion.cruiseVelocity(5676.0 * kGearing)
              .maxAcceleration(5676.0 * kGearing);

        return config;
    }

    public static MotorIOSparkMaxConfig getIOconfig() {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.mainConfig = ShooterSparkConfig();
        config.followerConfig = ShooterSparkConfig();
        config.time = Units.Minutes;
        config.unit = Units.Rotations;
        config.mainID = Ports.SHOOTER_MAIN.id;
        
        // SparkMax followers require explicit IDs. 
        // We look at your original alignment array (Aligned, Opposed, Opposed) to map inversions
        config.followerIDs = new int[] {
            Ports.SHOOTER_FOLLOWER_1.id, 
            Ports.SHOOTER_FOLLOWER_2.id, 
            Ports.SHOOTER_FOLLOWER_3.id
        };
        config.followerAlignment = new MotorAlignmentValue[] {
            MotorAlignmentValue.Aligned, // Aligned
            MotorAlignmentValue.Opposed,  // Opposed
            MotorAlignmentValue.Opposed   // Opposed
        };

        return config;
    }

    public static MotorIOSparkMax getMotorIO() {
        if (Robot.isReal()) {
            return new MotorIOSparkMax(getIOconfig());
        } else {
            return new MotorIOSparkMaxSim(getIOconfig(), new RollerSim(getSimConstants()));
        }
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants simConstants = new RollerSimConstants();
        // Updated to NEO/NeoVortex variants if using SparkMax infrastructure 
        simConstants.motor = DCMotor.getNEO(4); 
        simConstants.gearing = kGearing;
        simConstants.momentOfInertia = 0.0166570492;
        return simConstants;
    }
}