package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.config.SparkMaxConfig; // Import the configuration class type
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.util.power.PowerPriority;

// Specify both the configuration type and the IO implementation type
public class Shooter extends FlywheelMotorSubsystem<SparkMaxConfig, MotorIOSparkMax> {

  // Specify the explicit unit generic type (AngularVelocity) to prevent raw setpoint warnings
  public static final Setpoint<AngularVelocity> TESTSHOT = Setpoint.withVelocitySetpoint(ShooterConstants.kTestShot, 1);
  public static final Setpoint<AngularVelocity> IDLE = Setpoint.withNeutralSetpoint();
  public static final Setpoint<AngularVelocity> FERRY = Setpoint.withVelocitySetpoint(ShooterConstants.kFerry, 1);
  public static final Setpoint<AngularVelocity> SLOW = Setpoint.withVelocitySetpoint(ShooterConstants.kSlow);

  public static final Shooter mInstance = new Shooter();

  public Shooter() {
    super(
        ShooterConstants.getMotorIO(),
        "Shooter",
        RPM.of(80.0),
        ShooterConstants.VELOCITY_THRESHOLD_DEBOUNCE_TIME,
        true);
    // 4 motors x 30A smart limit
    enablePowerManagement(PowerPriority.HIGH, 60.0, 120.0);
  }
}