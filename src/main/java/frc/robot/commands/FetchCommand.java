package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;

public class FetchCommand extends SequentialCommandGroup {
    public FetchCommand(
        BeltDrive beltDrive,
        Drive drive,
        Hood hood,
        Leds leds,
        Shooter shooter) {
          addCommands(
      Commands.parallel(
        Commands.run(() -> drive.setStateAutoAlignAngle(() -> AllianceFlipping.apply(Rotation2d.kZero)), drive),
        Commands.run(() -> shooter.runVelocity(() -> Constants.FETCH_VELOCITY), shooter),
        Commands.run(() -> hood.setTargetAngle(() -> Constants.FETCH_ANGLE), hood))
          .raceWith(Commands.waitUntil(() -> shooter.atVelocity() && hood.atSetpoint()) // TODO: can calculate needed accuracy for drive
          .andThen(Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive)
          .alongWith(
            // Commands.runOnce(() -> hopper.setState(HopperStates.ACTIVE), hopper).alongWith(
              Commands.runOnce(() -> leds.setState(ledsStates.PURPLE), leds))
              // )
      )
    ));
    }
}
