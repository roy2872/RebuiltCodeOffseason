package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonSRX;

public class IntakeRollers extends MotorSubsystem<TalonSRXConfiguration, MotorIOTalonSRX> {
	public static final Setpoint<Voltage> IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint<Voltage> INTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.INTAKE_VOLTAGE);
	public static final Setpoint<Voltage> OUTTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.OUTTAKE_VOLTAGE);

    public static final IntakeRollers mInstance = new IntakeRollers();

    public IntakeRollers() {
        super(IntakeRollersConstants.getMotorIOTalonSRX(), "Intake Rollers");
    }
}
