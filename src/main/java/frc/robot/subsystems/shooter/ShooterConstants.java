package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIO.Gains;
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
    // MAXMotion velocity profile, in motor RPM and RPM per second. 4 NEOs at 30A on this flywheel
    // can reach roughly 4000 RPM/s, so the profile asks a bit less than that to stay trackable
    // Slot 1 = velocity, used by every shooter setpoint. Shows up as TunableNumbers/Shooter/Slot1/
    public static final int kVelocitySlot = 1;
    public static final Gains kVelocityGains = new Gains(
            0.0008, // kP: duty cycle per RPM of error, so 3% output at 100 RPM off. kV does most of the work
            0.0, // kI
            0.07, // kD: velocity is too noisy for D on a flywheel
            0.00232, // kV: volts per motor RPM. A NEO's back-EMF constant is 473 RPM/V
            0.0, // kS: volts to overcome friction
            0.0); // kG: no gravity on a flywheel

    public static final double kMaxMotionCruiseVelocity = 5676.0;
    public static final double kMaxMotionMaxAcceleration = 3000.0;

    public static SparkMaxConfig ShooterSparkConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Handle physical variables & limits
        config.idleMode(IdleMode.kCoast)
              .inverted(false) // Equivalent to CounterClockwise_Positive assuming right-hand configuration
              .smartCurrentLimit(40)
              .secondaryCurrentLimit(60);

        // The NEO's built-in encoder averages velocity over 32ms x 8 samples by default, which lags
        // so much the velocity loop oscillates. Shorter window = faster, slightly noisier reading
        config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

        // Gains are set in getIOconfig() from kVelocityGains, so they land on the right slot

        // Never drive the flywheel backwards to slow it down, just let it coast. Reversing at
        // speed (plugging) dumps the wheel's energy into the motor as heat
        config.closedLoop.outputRange(0.0, 1.0, ClosedLoopSlot.kSlot1);

        // MAXMotion velocity control runs on slot 1 with the velocity gains above
        config.closedLoop.maxMotion
                .cruiseVelocity(kMaxMotionCruiseVelocity, ClosedLoopSlot.kSlot1)
                .maxAcceleration(kMaxMotionMaxAcceleration, ClosedLoopSlot.kSlot1);

        return config;
    }

    public static MotorIOSparkMaxConfig getIOconfig() {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.mainConfig = ShooterSparkConfig();
        config.followerConfig = ShooterSparkConfig();
        config.time = Units.Minutes;
        config.unit = Units.Rotations;
        config.mainID = Ports.SHOOTER_MAIN.id;
        config.gains[0] = Gains.ZERO; // Slot 0 (MAXMotion position) is unused
        config.gains[kVelocitySlot] = kVelocityGains;
        
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