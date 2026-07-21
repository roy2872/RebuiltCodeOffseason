package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AllianceFlipping;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.BeltDrive;
import frc.robot.subsystems.feeder.BeltDrive.BeltDriveStates;
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

        // 1. Safe DoubleSuppliers that protect against null or missing vector data
        DoubleSupplier safeHoodAngle = () -> {
            var data = RobotState.getInstance().getFetchingInfo();
            return (data != null && data.getNumRows() > 0) ? data.get(0) : 0.0;
        };

        DoubleSupplier safeShooterVelocity = () -> {
            var data = RobotState.getInstance().getFetchingInfo();
            return (data != null && data.getNumRows() > 1) ? data.get(1) : 0.0;
        };

        addCommands(
            // Step 1: CONTINUOUSLY align, spin up shooter, and move hood UNTIL everything is ready
            Commands.parallel(
                Commands.run(() -> drive.setStateAutoAlignAngle(() -> AllianceFlipping.apply(Rotation2d.kZero)), drive),
                Commands.run(() -> hood.setTargetAngle(safeHoodAngle), hood),
                Commands.run(() -> shooter.runVelocity(safeShooterVelocity), shooter)
            ).until(() -> shooter.atVelocity() && hood.atSetpoint() && drive.isAtAutoAlignAngleSetpoint(2.0)),
            
            // Step 2: Now that it's aligned, turn on the belt drive while MAINTAINING position/speed
            Commands.parallel(
                Commands.run(() -> drive.setStateAutoAlignAngle(() -> AllianceFlipping.apply(Rotation2d.kZero)), drive),
                Commands.run(() -> hood.setTargetAngle(safeHoodAngle), hood),
                Commands.run(() -> shooter.runVelocity(safeShooterVelocity), shooter),
                Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive),
                Commands.runOnce(() -> leds.setState(ledsStates.AQUA), leds)
            )
        );
    }
}