// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Constants.ForceHomeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.units.Units.*;

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

	public Command stopCommand() {
		return Commands.parallel(
			Feeder.mInstance.setpointCommand(Feeder.IDLE),
			Hood.mInstance.setpointCommand(Hood.STOWED),
			IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED),
			IntakeRollers.mInstance.setpointCommand(IntakeRollers.IDLE),
			Shooter.mInstance.setpointCommand(Shooter.IDLE)
		);
	}

	public Command intakeExhaustCommand() {
		return Commands.sequence(
			IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED),
			Commands.parallel(
				IntakeRollers.mInstance.setpointCommand(IntakeRollers.OUTTAKE),
				Feeder.mInstance.setpointCommand(Feeder.SLOW_REVERSE)
			)
		);
	}

	public Command intakeCommand() {
		return Commands.sequence(
			IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED),
			IntakeRollers.mInstance.setpointCommand(IntakeRollers.OUTTAKE)
		).withName("Intake");
	}

	public Command idleIntakeRollers() {
		return IntakeRollers.mInstance.setpointCommand(IntakeRollers.IDLE)
			.withName("Idle Intake");
	}

	public Command shoot() {
		return Commands.print("this").andThen(Commands.parallel(
						// Cameras.mInstance.setStdDevCommand(CamerasConstants.ALIGN_STD_DEVATION),
						Commands.runOnce(() -> {
							Feeder.mInstance.applySetpoint(Feeder.IDLE);
						}),
						Shooter.mInstance.followSetpointCommand(
								() -> Setpoint.withVelocitySetpoint(
									Units.RotationsPerSecond.of(RobotState.mInstance.getShootInfo().get(1)))),
						Hood.mInstance.followSetpointCommand(() -> 
							Setpoint.withPositionSetpoint(
												// positionInputs.getHoodSetpoint()
												Units.Degrees.of(RobotState.mInstance.getShootInfo().get(1)))),
						DriveCommands.autoAlignAngle(Drive.mInstance, () -> Rotation2d.fromDegrees(RobotState.mInstance.getShootInfo().get(1))),
						Commands.sequence(
								Commands.waitUntil(() -> {
									boolean driveReady = RobotState.mInstance.atAngle(Rotation2d.fromDegrees(RobotState.mInstance.getShootInfo().get(1)), Degrees.of(1));
									
									boolean spunUp =
											!RobotState.mInstance.getShootType() ? shooterAndHoodSpunUp(Units.RPM.of(150)) : shooterAndHoodSpunUp();
									return driveReady
											&& spunUp;
								}
								),
								Commands.parallel(
									Feeder.mInstance.setpointCommand(Feeder.FEED_VOLTAGE)
									// Drive x pose

								))))
				// .finallyDo(() -> Cameras.mInstance.setSTDDeviations(CamerasConstants.DEFAULT_STD_DEVIATION))
				.withName("Shoot");
	}

	public Command stopShooting() {
		return Commands.parallel(
			Feeder.mInstance.setpointCommand(Feeder.IDLE),
			Hood.mInstance.setpointCommand(Hood.STOWED),
			Shooter.mInstance.setpointCommand(Shooter.IDLE)
		);
	}

	public boolean shooterAndHoodSpunUp() {
		boolean shooterReady = Shooter.mInstance.spunUpDebounced();
		boolean hoodReady = Hood.mInstance.nearPositionSetpoint();
		SmartDashboard.putBoolean("Shooter Spun Up", shooterReady);
		SmartDashboard.putBoolean("Hood In Position", hoodReady);
		return shooterReady && hoodReady;
	}

	public boolean shooterAndHoodSpunUp(AngularVelocity epsilon) {
		boolean shooterReady = Shooter.mInstance.spunUpDebounced(epsilon);
		boolean hoodReady = Hood.mInstance.nearPositionSetpoint();
		SmartDashboard.putBoolean("Shooter Spun Up", shooterReady);
		SmartDashboard.putBoolean("Hood In Position", hoodReady);
		return shooterReady && hoodReady;
	}

}
