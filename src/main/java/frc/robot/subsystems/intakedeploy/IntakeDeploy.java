package frc.robot.subsystems.intakedeploy;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;

public class IntakeDeploy extends ServoMotorSubsystem<SparkMaxConfig, MotorIOSparkMax> {

	public static final Setpoint<Angle> STOWED = Setpoint.withPositionSetpoint(IntakeDeployConstants.INTAKE_STOWED_ANGLE);
	public static final Setpoint<Angle> DEPLOYED =
			Setpoint.withPositionSetpoint(IntakeDeployConstants.INTAKE_DEPLOYED_ANGLE);
	public static final Setpoint<Angle> PARTIAL_IN = Setpoint.withPositionSetpoint(IntakeDeployConstants.INTAKE_PARTIAL_IN_ANGLE);

	public static final IntakeDeploy mInstance = new IntakeDeploy();

	private IntakeDeploy() {
		super(
				IntakeDeployConstants.getMotorIO(), "Intake Deploy", IntakeDeployConstants.INTAKE_ANGLE_TOLERANCE,
				IntakeDeployConstants.getServoHomingConfig()
				);
		setCurrentPosition(IntakeDeployConstants.INTAKE_STOWED_ANGLE);
	}
}