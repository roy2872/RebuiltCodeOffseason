package frc.robot.autonomous;

import static frc.robot.Constants.currentMode;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipping;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodStates;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterStates;

public class RightMiddleFuelAuto extends Command {

    private Pose2d startingPose = new Pose2d(3.61, 2.23, Rotation2d.kCCW_Pi_2);
    private Pose2d afterBumpPose = new Pose2d(5.70, 2.23, Rotation2d.kCCW_90deg);
    private Pose2d bumpReturnPose = new Pose2d(5.70, 2.75, Rotation2d.kZero);
    private Pose2d returnToAlliancePose = new Pose2d(3.61, 2.75, Rotation2d.kZero);

    private enum AutoSteps {
        CROSS_BUMP_TO_MIDDLE,
        COLLECT_FUEL_MIDDLE,
        RETURN_TO_BUMP,
        CROSS_TO_ALLIANCE,
        SHOOT
    }

    @AutoLogOutput (key = "AutoState")
    private AutoSteps currentState = AutoSteps.CROSS_BUMP_TO_MIDDLE;

    private final BeltDrive beltDrive;
    private final Drive drive;
    private final Hood hood;
    private final Intake intake;
    private final Leds leds;
    private final Shooter shooter;

    private final boolean shouldSwitchLeft;
    private boolean flag = false;
    private Timer timer;
    private double timestamp;

    public RightMiddleFuelAuto(
        BeltDrive beltDrive,
        Drive drive,
        Hood hood,
        Intake intake,
        Leds leds,
        Shooter shooter,
        boolean shouldSwitchLeft) {
        
        addRequirements(beltDrive, drive, hood, intake, leds, shooter);
        this.beltDrive = beltDrive;
        this.drive = drive;
        this.hood = hood;
        this.intake = intake;
        this.leds = leds;
        this.shooter = shooter;
        this.timestamp = -1.0;
        this.shouldSwitchLeft = shouldSwitchLeft;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        if(shouldSwitchLeft) {
            startingPose = AllianceFlipping.mirrorPoseRightLeft(startingPose);
            afterBumpPose = AllianceFlipping.mirrorPoseRightLeft(afterBumpPose);
            bumpReturnPose = AllianceFlipping.mirrorPoseRightLeft(bumpReturnPose);
            returnToAlliancePose = AllianceFlipping.mirrorPoseRightLeft(returnToAlliancePose);
            alignToClimbPose = AllianceFlipping.mirrorPoseRightLeft(alignToClimbPose);
            climbPose = AllianceFlipping.mirrorPoseRightLeft(climbPose);
        }

        startingPose = AllianceFlipping.apply(startingPose);
        afterBumpPose = AllianceFlipping.apply(afterBumpPose);
        bumpReturnPose = AllianceFlipping.apply(bumpReturnPose);
        returnToAlliancePose = AllianceFlipping.apply(returnToAlliancePose);
        alignToClimbPose = AllianceFlipping.apply(alignToClimbPose);
        climbPose = AllianceFlipping.apply(climbPose);

        var yawObservation = RobotState.getInstance().getLimelightYawObservation();
        if(RobotController.getFPGATime() * (1e-6) - yawObservation.timestamp() < 0.5) {
            startingPose = new Pose2d(startingPose.getX(), startingPose.getY(), yawObservation.yaw());
        }
        RobotState.getInstance().resetPose(startingPose);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        switch(currentState) {
            case CROSS_BUMP_TO_MIDDLE:
                drive.setStateAutoAlign(() -> afterBumpPose, 1.0, Math.PI);
                hood.setState(HoodStates.IDLE);
                beltDrive.setState(BeltDriveStates.IDLE);
                intake.setState(IntakeStates.IDLE);
                leds.setState(ledsStates.PURPLE);
                shooter.setState(ShooterStates.IDLE);
                if(drive.isAtAlignSetpoint(0.10, 2)) {
                    currentState = AutoSteps.COLLECT_FUEL_MIDDLE;
                    drive.startChoreoPath("middleFuelPath", false, shouldSwitchLeft);
                }
                break;

            case COLLECT_FUEL_MIDDLE:
                intake.setState(IntakeStates.INTAKE);
                leds.setState(ledsStates.PINK);
                if(drive.isChoreoPathFinished()) {
                    currentState = AutoSteps.RETURN_TO_BUMP;
                }
                break;
            case RETURN_TO_BUMP:
                intake.setState(IntakeStates.OPEN);
                drive.setStateAutoAlign(() -> bumpReturnPose, 3.0, 2 * Math.PI);
                 if(drive.isAtAlignSetpoint(0.10, 2)) {
                    currentState = AutoSteps.CROSS_TO_ALLIANCE;
                }
                break;
            case CROSS_TO_ALLIANCE:
                drive.setStateAutoAlign(() -> returnToAlliancePose, 1.0, 1 * Math.PI);
                hood.setTargetAngle(() -> RobotState.getInstance().getShootOnTheMoveScoringInfo().get(0));
                shooter.runVelocity(() -> RobotState.getInstance().getShootOnTheMoveScoringInfo().get(1));
                 if(drive.isAtAlignSetpoint(0.10, 2)) {
                    currentState = AutoSteps.SHOOT_WHILE_TRAVEL;
                    flag = false;
                }
                break;
            case SHOOT_WHILE_TRAVEL:
                drive.setStatePathAndShoot(() -> alignToClimbPose.getTranslation(), 1.0, 2 * Math.PI);
                hood.setTargetAngle(() -> RobotState.getInstance().getShootOnTheMoveScoringInfo().get(0));
                shooter.runVelocity(() -> RobotState.getInstance().getShootOnTheMoveScoringInfo().get(1));
                if(!flag) {
                    if((currentMode == Mode.SIM || (hood.atSetpoint() && shooter.atVelocity())) 
                        && drive.isAtPathAndShootSetpointTranslation(0.1)) {
                        flag = true;
                    }
                } else {
                    beltDrive.setState(BeltDriveStates.ACTIVE);
                    intake.setState(IntakeStates.INTAKE);
                    leds.setState(ledsStates.FINISH_SCORE);
                }
                if(timer.get() >= 20.0 - 5.0) {
                    currentState = AutoSteps.ALIGN_BEFORE_CLIMB;
                }
                break;
        }
    }
}