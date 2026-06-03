package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberStates;
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
import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class TestClimbAuto extends Command {

    private Pose2d startingPose = new Pose2d(3.61, 2.0, Rotation2d.kPi);
    private Pose2d humanStationPose = new Pose2d(0.52, 0.64, Rotation2d.kPi);
    private Pose2d shootingPose = new Pose2d(1.07, 2.52, Rotation2d.kPi);
    private Pose2d alignToClimbPose = new Pose2d(1.04, 2.55, Rotation2d.fromDegrees(270));
    private Pose2d climbPose = new Pose2d(1.06, 2.86, Rotation2d.fromDegrees(270));

    private enum AutoSteps {
        ALIGN_BEFORE_CLIMB,
        ALIGN_CLIMB,
        CLIMB
    }

    @AutoLogOutput (key = "AutoState")
    private AutoSteps currentState = AutoSteps.ALIGN_BEFORE_CLIMB;

    private final BeltDrive beltDrive;
    private final Climber climber;
    private final Drive drive;
    private final Hood hood;
    private final Hopper hopper;
    private final Intake intake;
    private final Leds leds;
    private final Shooter shooter;

    private double timestamp;

    public TestClimbAuto(
      BeltDrive beltDrive,
      Climber climber,
      Drive drive,
      Hood hood,
      Hopper hopper,
      Intake intake,
      Leds leds,
      Shooter shooter) {
        this.beltDrive = beltDrive;
        this.climber = climber;
        this.drive = drive;
        this.hood = hood;
        this.hopper = hopper;
        this.intake = intake;
        this.leds = leds;
        this.shooter = shooter;

      }

      @Override
      public void initialize() {
        startingPose = AllianceFlipping.apply(startingPose);
        humanStationPose = AllianceFlipping.apply(humanStationPose);
        shootingPose = AllianceFlipping.apply(shootingPose);
        alignToClimbPose = AllianceFlipping.apply(alignToClimbPose);
        climbPose = AllianceFlipping.apply(climbPose);
        currentState = AutoSteps.ALIGN_BEFORE_CLIMB;
        var yawObservation = RobotState.getInstance().getLimelightYawObservation();
        if(RobotController.getFPGATime() * (1e-6) - yawObservation.timestamp() < 0.5) {
            startingPose = new Pose2d(startingPose.getX(), startingPose.getY(), yawObservation.yaw());
        }
        // RobotState.getInstance().resetPose(startingPose);
      }

      @Override
      public void execute() {
        switch(currentState) {
            case ALIGN_BEFORE_CLIMB:
                drive.setStateAutoAlign(() -> (alignToClimbPose));
                hood.setState(HoodStates.CLOSED);
                shooter.setState(ShooterStates.IDLE);
                beltDrive.setState(BeltDriveStates.IDLE);
                hopper.setState(HopperStates.IDLE);
                leds.setState(ledsStates.PURPLE);
                climber.setState(ClimberStates.OPEN_NO_CLIMB);
                if (drive.isAtAlignSetpoint(0.03, 1)) {
                    currentState = AutoSteps.ALIGN_CLIMB;
                    timestamp = Timer.getTimestamp();
                }
                break;
            case ALIGN_CLIMB:
                drive.setStateAutoAlign(() -> (climbPose), 0.5, 1 * Math.PI);
                hood.setState(HoodStates.CLOSED);
                shooter.setState(ShooterStates.IDLE);
                beltDrive.setState(BeltDriveStates.IDLE);
                hopper.setState(HopperStates.IDLE);
                leds.setState(ledsStates.WHITE);
                climber.setState(ClimberStates.OPEN_NO_CLIMB);
                if (drive.isAtAlignSetpoint(0.03, 1)) {
                    currentState = AutoSteps.CLIMB;
                    timestamp = Timer.getTimestamp();
                }
                break;
            case CLIMB:
                drive.setDriveState(DriveStates.IDLE);
                hood.setState(HoodStates.CLOSED);
                shooter.setState(ShooterStates.IDLE);
                beltDrive.setState(BeltDriveStates.IDLE);
                hopper.setState(HopperStates.IDLE);
                leds.setState(ledsStates.FINISH_SCORE);
                climber.setState(ClimberStates.CLOSE_CLIMB);
                break;
        }
      }
}
