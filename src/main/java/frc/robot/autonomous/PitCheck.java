package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.BeltDrive;
import frc.robot.subsystems.feeder.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterStates;

public class PitCheck {

    /**
     * Creates a sequential pit test command that handles intaking a ball, 
     * setting the hood to 60 degrees, and firing at 2.0 mps.
     */
    public static Command getCommand(
            Intake intake, 
            BeltDrive beltDrive, 
            Hood hood, 
            Shooter shooter, 
            Leds leds) {
            
        return Commands.sequence(
            // --- STEP 1: INITIALIZE & INTAKE ---
            Commands.runOnce(() -> System.out.println("====== STARTING SUPERSTRUCTURE PIT CHECK ======")),
            Commands.runOnce(() -> leds.setState(ledsStates.PURPLE)),
            
            Commands.runOnce(() -> System.out.println("Deploying Intake")),
            Commands.runOnce(() -> intake.setState(IntakeStates.INTAKE), intake),
            Commands.waitSeconds(5), // Give the human time to feed a ball into the intake

            // --- STEP 2: STOW INTAKE & PREPARE SUPERSTRUCTURE ---
            Commands.runOnce(() -> System.out.println("Stowing intake. Prepping shooter: 2.0 mps @ 60 degrees...")),
            Commands.runOnce(() -> intake.setState(IntakeStates.IDLE), intake),
            // --- STEP 3: SPIN UP & AIM ---
            Commands.parallel(
                Commands.run(() -> hood.setTargetAngle(() -> 60.0), hood),
                Commands.run(() -> shooter.runVelocity(() -> 2.0), shooter)
            ).until(() -> shooter.atVelocity() && hood.atSetpoint())
             .withTimeout(4.0), // Failsafe timeout in case sensors are broken

            // --- STEP 4: SHOOT (FEED THE BALL) ---
            Commands.runOnce(() -> System.out.println("Firing...")),
            // Keep the shooter and hood powered WHILE the belt drive feeds the ball out
            Commands.parallel(
                Commands.run(() -> hood.setTargetAngle(() -> 60.0), hood),
                Commands.run(() -> shooter.runVelocity(() -> 2.0), shooter),
                Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.ACTIVE), beltDrive)
            ).withTimeout(3.0), // Run indexers long enough to completely clear the ball

            // --- STEP 5: CLEANUP & SHUTDOWN ---
            Commands.runOnce(() -> System.out.println("====== PIT CHECK COMPLETE ======")),
            Commands.runOnce(() -> leds.setState(ledsStates.AQUA)),
            Commands.runOnce(() -> intake.setState(IntakeStates.OPEN), intake),
            Commands.runOnce(() -> beltDrive.setState(BeltDriveStates.IDLE), beltDrive),
            Commands.runOnce(() -> shooter.setState(ShooterStates.IDLE), shooter), // Assuming your subsystem has a stop method
            Commands.runOnce(() -> hood.setState(HoodStates.IDLE))
        );
    }
}