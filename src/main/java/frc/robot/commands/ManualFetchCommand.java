package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;

public class ManualFetchCommand extends SequentialCommandGroup {
    public ManualFetchCommand(
        BeltDrive beltDrive,
        Drive drive,
        Hood hood,
        Leds leds,
        Shooter shooter) {
          addCommands(
      Commands.parallel(
        Commands.runOnce(() -> drive.setStateAutoAlignAngle(() -> AllianceFlipping.apply(Rotation2d.kZero)), drive),
        Commands.runOnce(() -> hood.setTargetAngle(() -> Constants.FETCHING_ANGLE), hood),
        Commands.runOnce(() -> shooter.runVelocity(() -> Constants.FETCHING_VELOCITY),  shooter)),
        Commands.waitUntil(() -> shooter.atVelocity() && hood.atSetpoint()), // TODO: can calculate needed accuracy for drive
        Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive),
        Commands.runOnce(() -> leds.setState(ledsStates.AQUA), leds)
        );
    }
}
