package frc.robot.subsystems.intakerollers;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;

public class IntakeRollers extends MotorSubsystem<SparkMaxConfig, MotorIOSparkMax> {
	public static final Setpoint<Voltage> IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint<Voltage> INTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.INTAKE_VOLTAGE);
	public static final Setpoint<Voltage> OUTTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.OUTTAKE_VOLTAGE);

    public static final IntakeRollers mInstance = new IntakeRollers();

    public IntakeRollers() {
        super(IntakeRollersConstants.getMotorIO(), "Intake Rollers");
    }
}
