package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;

public class FetchCommand extends SequentialCommandGroup {
    public FetchCommand(
        BeltDrive beltDrive,
        Drive drive,
        Hood hood,
        Leds leds,
        Shooter shooter) {
      addCommands();
    }
}
