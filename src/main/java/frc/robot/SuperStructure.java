// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Constants.ForceHomeConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SuperStructure extends SubsystemBase {

  public static final SuperStructure mInstance = new SuperStructure();

  private final Debouncer intakeForceHomeDebouncer =
		new Debouncer(ForceHomeConstants.INTAKE_FORCE_DEBOUNCE.in(Seconds));

  private final Debouncer hoodForceHomeDebouncer =
    new Debouncer(ForceHomeConstants.HOOD_FORCE_DEBOUNCE.in(Seconds));

  @Override
  public void periodic() {
  }

  public Command intakeForceHomeCommand() {
		return Commands.sequence(
						Commands.deadline(
								Commands.waitUntil(() -> intakeForceHomeDebouncer.calculate(
										IntakeDeploy.mInstance.getVelocity().abs(DegreesPerSecond)
														<= ForceHomeConstants.INTAKE_MIN_HOME_VELOCITY.in(
																DegreesPerSecond)
												&& IntakeDeploy.mInstance.getSetpoint().mode == Mode.VOLTAGE)),
								IntakeDeploy.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Volts.of(5.0)))),
						IntakeDeploy.mInstance.setpointCommand(Setpoint.withNeutralSetpoint()),
						IntakeDeploy.mInstance.runOnce(
								() -> IntakeDeploy.mInstance.setCurrentPosition(IntakeDeployConstants.INTAKE_STOWED_ANGLE)))
				.ignoringDisable(true)
				.withName("Force Intake Home");
	}

  public Command hoodForceHomeCommand() {
		return Commands.sequence(
						Commands.deadline(
								Commands.waitUntil(() -> hoodForceHomeDebouncer.calculate(
										Hood.mInstance.getVelocity().abs(DegreesPerSecond)
														<= ForceHomeConstants.HOOD_MIN_HOME_VELOCITY.in(
																DegreesPerSecond)
												&& Hood.mInstance.getSetpoint().mode == Mode.VOLTAGE)),
								Hood.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Volts.of(5.0).unaryMinus()))),
						Hood.mInstance.setpointCommand(Setpoint.withNeutralSetpoint()),
						Hood.mInstance.runOnce(
								() -> Hood.mInstance.setCurrentPosition(HoodConstants.HOOD_MIN_ANGLE)))
				.ignoringDisable(true)
				.withName("Force Hood Home");
	}
}
