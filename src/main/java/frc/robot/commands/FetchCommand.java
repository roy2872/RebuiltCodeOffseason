package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
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
        Commands.runOnce(() -> drive.setStateAutoAlignAngle(() -> AllianceFlipping.apply(Rotation2d.k180deg)), drive),
        Commands.runOnce(() -> shooter.runExitVelocity(() -> Constants.FETCH_VELOCITY), shooter),
        Commands.runOnce(() -> hood.setTargetAngle(() -> Constants.FETCH_ANGLE), hood),
        Commands.waitUntil(() -> shooter.atVelocity() && hood.atSetpoint() && drive.isAtAutoAlignAngleSetpoint(1.0))
          .andThen(Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive)
          .alongWith(
            // Commands.runOnce(() -> hopper.setState(HopperStates.ACTIVE), hopper).alongWith(
              Commands.runOnce(() -> leds.setState(ledsStates.PURPLE), leds))
              // )
      )
    ));
    }
}
