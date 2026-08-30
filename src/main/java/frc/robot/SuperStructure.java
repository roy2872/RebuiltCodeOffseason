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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

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
    return IntakeDeploy.mInstance.runEnd(
            // Drive toward the stow hard stop for a bounded time.
            () -> IntakeDeploy.mInstance.applySetpoint(Setpoint.withVoltageSetpoint(Volts.of(-7.0))),
            // Always neutral the motor first, including when canceled.
            () -> IntakeDeploy.mInstance.applySetpoint(Setpoint.withNeutralSetpoint())
        )
        // runEnd() does not finish by itself; make this command end after one second.
        .withTimeout(1.0)
        // finallyDo runs both after the timeout and after scheduler cancellation.
        .finallyDo(
            () ->
                IntakeDeploy.mInstance.setCurrentPosition(
                    IntakeDeployConstants.INTAKE_STOWED_ANGLE))
        .ignoringDisable(true)
        .withName("Force Intake Home and Zero");
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
				IntakeRollers.mInstance.runEnd(
					() -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.OUTTAKE),
					() -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.IDLE)),
				Feeder.mInstance.runEnd(
					() -> Feeder.mInstance.applySetpoint(Feeder.SLOW_REVERSE),
					() -> Feeder.mInstance.applySetpoint(Feeder.IDLE))
			)
		);
	}

	public Command intakeCommand() {
		return Commands.sequence(
			IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED),
			IntakeRollers.mInstance.runEnd(
				() -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.INTAKE),
				() -> IntakeRollers.mInstance.applySetpoint(IntakeRollers.IDLE))
		).withName("Intake");
	}

	public Command idleIntakeRollers() {
		return IntakeRollers.mInstance.setpointCommand(IntakeRollers.IDLE)
			.withName("Idle Intake");
	}

	public Command shoot() {
		return Commands.parallel(
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
												Units.Degrees.of(RobotState.mInstance.getShootInfo().get(0)))),
						DriveCommands.autoAlignAngle(Drive.mInstance, () -> Rotation2d.fromDegrees(RobotState.mInstance.getShootInfo().get(2))),
						Commands.sequence(
								Commands.waitUntil(() -> {
									boolean driveReady = RobotState.mInstance.atAngle(Rotation2d.fromDegrees(RobotState.mInstance.getShootInfo().get(2)), Degrees.of(1));
									
									boolean spunUp =
											!RobotState.mInstance.getShootType() ? shooterAndHoodSpunUp(Units.RPM.of(150)) : shooterAndHoodSpunUp();
									return driveReady
											&& spunUp;
								}
								),
								Commands.parallel(
									Feeder.mInstance.setpointCommand(Feeder.FEED_VOLTAGE)
									// Drive x pose

								)))
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
