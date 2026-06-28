package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;

public class RealDepotAuto extends SequentialCommandGroup{

    BeltDrive beltDrive;
    Drive drive;
    Hood hood;
    Shooter shooter;
    
        public RealDepotAuto(
        BeltDrive beltDrive,
        Drive drive,
        Hood hood,
        Intake intake,
        Leds leds,
        Shooter shooter) {
            addCommands(
                new DepotAuto(beltDrive, drive, hood, intake, leds, shooter),
                new ShootCommand(beltDrive, drive, hood, leds, shooter, RobotState.getInstance()::getShootCloseInfo, 
            () -> !SmartDashboard.getBoolean("DriverControlWhenShooting", false))
            );
        }
}
