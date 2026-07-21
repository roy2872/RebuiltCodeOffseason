package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AllianceFlipping;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.feeder.BeltDrive;
import frc.robot.subsystems.feeder.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    
    public ShootCommand(
        BeltDrive beltDrive,
        Drive drive,
        Hood hood,
        Leds leds,
        Shooter shooter,
        Supplier<Vector<N3>> dataSupplier,
        BooleanSupplier shouldAutoAlignToTarget) { // Passed as a supplier here

        // Safe DoubleSuppliers that extract indexes safely on every loop iteration
        DoubleSupplier safeHoodAngle = () -> {
            Vector<N3> data = dataSupplier.get();
            return (data != null && data.getNumRows() > 0) ? data.get(0) : 0.0;
        };

        DoubleSupplier safeShooterVelocity = () -> {
            Vector<N3> data = dataSupplier.get();
            return (data != null && data.getNumRows() > 1) ? data.get(1) : 0.0;
        };

        addCommands(
            // Step 1: Spin up and align continuously until target is met
            Commands.parallel(
                Commands.either(Commands.run(() -> drive.setStateAutoAlignAngle(() -> Rotation2d.fromDegrees(dataSupplier.get().get(2))), drive),
                Commands.run(() -> drive.setState(DriveStates.FIELD_DRIVE), drive), shouldAutoAlignToTarget),
                Commands.run(() -> hood.setTargetAngle(safeHoodAngle), hood),
                Commands.run(() -> shooter.runVelocity(safeShooterVelocity), shooter)
            ).until(() -> shooter.atVelocity() && hood.atSetpoint() && 
            (drive.isAtAutoAlignAngleSetpoint(2.0) || !shouldAutoAlignToTarget.getAsBoolean()))
            .withTimeout(4.0),
            
            // Step 2: Keep targets active while feeding the system
            Commands.parallel(
                Commands.either(Commands.run(() -> drive.setStateAutoAlignAngle(() -> Rotation2d.fromDegrees(dataSupplier.get().get(2))), drive),
                Commands.run(() -> drive.setState(DriveStates.FIELD_DRIVE), drive), shouldAutoAlignToTarget),
                Commands.run(() -> hood.setTargetAngle(safeHoodAngle), hood),
                Commands.run(() -> shooter.runVelocity(safeShooterVelocity), shooter),
                Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive),
                Commands.runOnce(() -> leds.setState(ledsStates.AQUA), leds)
            )
        );
    }
}