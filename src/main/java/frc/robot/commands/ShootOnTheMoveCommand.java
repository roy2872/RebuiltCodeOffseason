// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;

public class ShootOnTheMoveCommand extends SequentialCommandGroup {

  public ShootOnTheMoveCommand(
      BeltDrive beltDrive, Drive drive, Hood hood, Hopper hopper, Leds leds, Shooter shooter) {
    addRequirements(getRequirements());
    addCommands(
      Commands.parallel(
        Commands.run(() -> drive.setState(DriveStates.SHOOT_ON_THE_MOVE), drive),
        Commands.run(() -> shooter.runExitVelocity(() -> RobotState.getInstance().getShootOnTheMoveScoringInfo().get(1)), shooter),
        Commands.run(() -> hood.setTargetAngle(() -> RobotState.getInstance().getShootOnTheMoveScoringInfo().get(0)), hood),
        Commands.waitUntil(() -> shooter.atVelocity() && hood.atSetpoint() && drive.isAtShootOnTheMoveSetpoint(1.0)) // TODO: can calculate needed accuracy for drive
          .andThen(Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive)
          .alongWith(Commands.runOnce(() -> hopper.setState(HopperStates.ACTIVE), hopper)
          .alongWith(Commands.runOnce(() -> leds.setState(ledsStates.PURPLE), leds)))
      )
    ));
  }
}
