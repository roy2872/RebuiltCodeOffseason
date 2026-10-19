package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig; // Assuming this wrapper exists similarly to your Talon setup
import frc.lib.io.MotorIOSparkMaxSim; // Assuming your simulator interface exists for Spark Max
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.TunableNumber;
import frc.robot.Ports;
import frc.robot.Constants;

public class ShooterConstants {

    public static final double kGearing = 1/1.0;

    public static final AngularVelocity kEpsilonThreshold = RPM.of(10.0);

    public static final Time VELOCITY_THRESHOLD_DEBOUNCE_TIME = Seconds.of(0.0);

    public static final AngularVelocity kTestShot = RPM.of(4000.0);
    public static final AngularVelocity kFerry = RPM.of(1000.0);
    public static final AngularVelocity kSlow = RPM.of(2100.0);

    // Ball launch model, the same as the shooting table generator (BallSimWithMagnusAverageWheels.cpp).
    // RobotState converts table speeds to flywheel RPS with this diameter.
    public static final Distance FLYWHEEL_DIAMETER = Inches.of(4.0);
    // The back (hood) wheel runs slower than the main wheel, which puts backspin on the ball
    public static final double HOOD_WHEEL_SPEED_RATIO = 0.5;
    // Ball speed out of the shooter / average surface speed of the two wheels (slip and compression loss).
    // 1.0 is no slip. Teams report real shooters at about 0.7-0.9 of that; nobody has measured it for fuel.
    // Tune it until the real shots match the table: shooting short means it is lower than this.
    public static final TunableNumber BALL_SPEED_FRACTION = new TunableNumber("Shooter/BallSpeedFraction", 0.85);
    // Where the ball leaves the shooter, in the robot frame. The shooter faces -x.
    public static final Translation3d LAUNCH_POSITION_ON_ROBOT = new Translation3d(-0.14, 0.0, 0.66);

    /** @return the main wheel surface speed (m/s) that launches the ball at ballSpeed (m/s) */
    public static double mainWheelSpeedForBallSpeed(double ballSpeed) {
        return ballSpeed / ballSpeedPerMainWheelSpeed();
    }

    /** @return the speed (m/s) the ball leaves at when the main wheel surface moves at mainWheelSpeed (m/s) */
    public static double ballSpeedForMainWheelSpeed(double mainWheelSpeed) {
        return mainWheelSpeed * ballSpeedPerMainWheelSpeed();
    }

    private static double ballSpeedPerMainWheelSpeed() {
        return (1.0 + HOOD_WHEEL_SPEED_RATIO) / 2.0 * BALL_SPEED_FRACTION.getAsDouble();
    }

    public static SparkMaxConfig ShooterSparkConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Handle physical variables & limits
        config.idleMode(IdleMode.kCoast)
              .inverted(false) // Equivalent to CounterClockwise_Positive assuming right-hand configuration
              .smartCurrentLimit(30)
              .secondaryCurrentLimit(40); 

        // Slot 1: Active Velocity control slots
        double kP = 1;
        double kI = 0.0;
        double kD = 0.0;
        // Converting kV calculation format safely over to method chaining parameters
        double kV = ((12.0) / (5676.0) * kGearing) * 0.995;
                
        config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot1).feedForward.apply(new FeedForwardConfig().kS(0.0).kV(kV).kA(0.0));

        // Slot 0: Flatlined defaults matching your Talon template
        config.closedLoop.pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0);

        // Max Motion constraints maps over from MotionMagic parameters
        // config.closedLoop.maxMotion
        // .cruiseVelocity(5676.0 * kGearing)
        //       .maxAcceleration(5676.0 * kGearing);

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
            MotorAlignmentValue.Opposed, // Aligned
            MotorAlignmentValue.Opposed,  // Opposed
            MotorAlignmentValue.Opposed   // Opposed
        };

        return config;
    }

    public static MotorIOSparkMax getMotorIO() {
        // Replay uses the real IO too: its inputs are overwritten from the log
        if (Constants.currentMode != Constants.Mode.SIM) {
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
        simConstants.momentOfInertia = 0.0066570492;
        return simConstants;
    }
}