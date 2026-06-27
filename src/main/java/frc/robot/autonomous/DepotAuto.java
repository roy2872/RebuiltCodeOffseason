package frc.robot.autonomous;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.beltDrive.BeltDrive;
import frc.robot.subsystems.beltDrive.BeltDrive.BeltDriveStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodStates;
import frc.robot.subsystems.hopper.Hopper.HopperStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterStates;

public class DepotAuto extends Command {
    
  private enum AutoStates {
    START,
    ALIGN_TO_DEPOT,
    COLLECT_DEPOT,
    EXIT_DEPOT,
    ALIGN_TO_SHOOTER,
    PREPARE_TO_SHOOT,
    SHOOT,
    END
  }
  @AutoLogOutput(key = "DepotAuto/currentState")
  private AutoStates currentState = AutoStates.START;
  private double startTimeSeconds;
  private double timestamp = 0.0;

    private final BeltDrive beltDrive;
    private final Drive drive;
    private final Hood hood;
    private final Intake intake;
    private final Leds leds;
    private final Shooter shooter;

    private Pose2d startingPose = new Pose2d(3.55, 4.0, Rotation2d.kPi);
    private Pose2d depotAlignPose = new Pose2d(1.3, 5.9, Rotation2d.kPi);
    private Pose2d DepotCollectPose = new Pose2d(0.6, 5.9, Rotation2d.kPi);
    private Pose2d depotExitPose = new Pose2d(1.3, 5.9, Rotation2d.kPi);
    private Pose2d shootingPose = new Pose2d(3.26, 4.0, Rotation2d.kPi);

  public DepotAuto(
    BeltDrive beltDrive,
      Drive drive,
      Hood hood,
      Intake intake,
      Leds leds,
      Shooter shooter
  ) {
    this.beltDrive = beltDrive;
    this.drive = drive;
    this.hood = hood;
    this.intake = intake;
    this.leds = leds;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    currentState = AutoStates.START;
    startTimeSeconds = RobotController.getFPGATime() / 1e6;

    startingPose = AllianceFlipping.apply(startingPose);
    depotAlignPose = AllianceFlipping.apply(depotAlignPose);
    DepotCollectPose = AllianceFlipping.apply(DepotCollectPose);
    depotExitPose = AllianceFlipping.apply(depotExitPose);
    shootingPose = AllianceFlipping.apply(shootingPose);

    var yawObservation = RobotState.getInstance().getLimelightYawObservation();
    if(RobotController.getFPGATime() * (1e-6) - yawObservation.timestamp() < 0.5) {
        startingPose = new Pose2d(startingPose.getX(), startingPose.getY(), yawObservation.yaw());
    }
    RobotState.getInstance().resetPose(startingPose);
  }

    @Override
  public void execute() {
    switch (currentState) {
      case START:
        RobotState.getInstance().resetPose(startingPose);
        intake.setState(IntakeStates.OPEN);
        hood.setState(HoodStates.BOOT_SEQUENCE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        leds.setState(ledsStates.PURPLE);
        currentState = AutoStates.ALIGN_TO_DEPOT;
        break;
      case ALIGN_TO_DEPOT:
        intake.setState(IntakeStates.OPEN);
        hood.setState(HoodStates.IDLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        drive.setStateAutoAlign(()->depotAlignPose, 1, 2.0);
        if(drive.isAtAlignSetpoint(0.05, 2))
            currentState = AutoStates.COLLECT_DEPOT;
        break;
      case COLLECT_DEPOT:
        intake.setState(IntakeStates.INTAKE);
        hood.setState(HoodStates.IDLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        drive.setStateAutoAlign(()->DepotCollectPose, 0.2, 1.0);
        if(drive.isAtAlignSetpoint(0.05, 2))
            currentState = AutoStates.EXIT_DEPOT;
        break;
      case EXIT_DEPOT:
        intake.setState(IntakeStates.INTAKE);
        hood.setState(HoodStates.IDLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        drive.setStateAutoAlign(()->depotExitPose, 0.2, 2.0);
        if(drive.isAtAlignSetpoint(0.05, 2))
            currentState = AutoStates.ALIGN_TO_SHOOTER;
        break;
      case ALIGN_TO_SHOOTER:
        intake.setState(IntakeStates.IDLE);
        hood.setState(HoodStates.IDLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        drive.setStateAutoAlign(()->shootingPose, 1.0, 2.0);
        if(drive.isAtAlignSetpoint(0.05, 2)) {
            currentState = AutoStates.PREPARE_TO_SHOOT;
        }
        break;
      case PREPARE_TO_SHOOT:
        intake.setState(IntakeStates.OPEN);
        hood.setState(HoodStates.TRACKING);
        hood.setTargetAngle(() -> Constants.SHOOT_CLOSE_ANGLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.runVelocity(() -> Constants.SHOOT_CLOSE_VELOCITY);
        drive.setState(DriveStates.X_LOCK);
        if(shooter.atVelocity() && hood.atSetpoint())
        timestamp = RobotController.getFPGATime() * (1e-6);
            currentState = AutoStates.SHOOT;
        break;
      case SHOOT:
        intake.setState(IntakeStates.OPEN);
        hood.setState(HoodStates.TRACKING);
        hood.setTargetAngle(() -> Constants.SHOOT_CLOSE_ANGLE);
        beltDrive.setState(BeltDriveStates.ACTIVE);
        shooter.runVelocity(() -> Constants.SHOOT_CLOSE_VELOCITY);
        drive.setState(DriveStates.X_LOCK);
        if(RobotController.getFPGATime() * (1e-6) - timestamp > 6.0)
            currentState = AutoStates.END;
        break;
      case END:
        intake.setState(IntakeStates.OPEN);
        hood.setState(HoodStates.IDLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        drive.setState(DriveStates.FIELD_DRIVE);
        break;
    }
  }

    @Override
    public void end(boolean interrupted) {
        intake.setState(IntakeStates.OPEN);
        hood.setState(HoodStates.IDLE);
        beltDrive.setState(BeltDriveStates.IDLE);
        shooter.setState(ShooterStates.IDLE);
        drive.setState(DriveStates.FIELD_DRIVE);
    }

    @Override
    public boolean isFinished() {
        return currentState == AutoStates.END;
    }
}
