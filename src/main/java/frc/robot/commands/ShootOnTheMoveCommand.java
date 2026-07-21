// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.feeder.BeltDrive;
import frc.robot.subsystems.feeder.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;

public class ShootOnTheMoveCommand extends SequentialCommandGroup {

  private final Supplier<Vector<N3>> shootingDataSupplier;

  public ShootOnTheMoveCommand(
      BeltDrive beltDrive, Drive drive, Hood hood, Leds leds, Shooter shooter) {
    shootingDataSupplier = () -> RobotState.getInstance().getShootOnTheMoveScoringInfo();
    addCommands(
      Commands.parallel(
        Commands.runOnce(() -> drive.setState(DriveStates.SHOOT_ON_THE_MOVE), drive),
        Commands.runOnce(() -> hood.setTargetAngle(() -> shootingDataSupplier.get().get(0)), hood),
        Commands.runOnce(() -> shooter.runVelocity(() -> shootingDataSupplier.get().get(1)),  shooter)),
        Commands.waitUntil(() -> shooter.atVelocity() && hood.atSetpoint()), // TODO: can calculate needed accuracy for drive
        Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive),
        Commands.runOnce(() -> leds.setState(ledsStates.AQUA), leds)
    );
  }
}
