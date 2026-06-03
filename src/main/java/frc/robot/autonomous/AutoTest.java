// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotState.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipping;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;

public class AutoTest extends Command {

  private enum AutoStates {
    START,
    ALIGN1,
    ALIGN_REEF1,
    PLACE1,
    DRIVE1,
    DRIVE2,
    ALIGN2,
    ALIGN_REEF2,
    PLACE2,
    END
  }

  private AutoStates currentState;
  private Timer overallTimer;
  private Timer timer;
  // private Runnable resetGyro;
  private Drive drive;
  private Leds leds;

  public AutoTest(Drive drive, Leds leds) {
    this.drive = drive;
    this.leds = leds;
    overallTimer = new Timer();
    timer = new Timer();
    addRequirements(drive, leds);
    // this.resetGyro = resetGyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // resetGyro.run();
    timer.reset();
    timer.start();
    currentState = AutoStates.START;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState) {
      case START:
        currentState = AutoStates.ALIGN1;
        break;
      case ALIGN1:
        drive.setStateAutoAlign(() -> (AllianceFlipping.apply(new Pose2d())));
        if (drive.isAtAlignSetpoint(0.06, 3)) currentState = AutoStates.ALIGN_REEF1;
        break;
      case ALIGN_REEF1:
        drive.setStateAutoAlign(() -> (AllianceFlipping.apply(new Pose2d())));
        if (drive.isAtAlignSetpoint(0.03, 3)) {
          currentState = AutoStates.PLACE1;
          timer.reset();
          timer.start();
        }
        break;
      case PLACE1:
        if (timer.get() < 1.0) {
          leds.setState(ledsStates.FINISH_SCORE);
        } else {
          currentState = AutoStates.DRIVE1;
          timer.reset();
        }
        break;
      case DRIVE1:
        drive.setStateAutoAlign(
            () -> AllianceFlipping.apply(new Pose2d(4.65, 1.1, Rotation2d.fromDegrees(90))));
        if (drive.isAtAlignSetpoint(0.3, 6)) {
          currentState = AutoStates.DRIVE2;
          timer.reset();
        }
        break;
      case DRIVE2:
        drive.setStateAutoAlign(
            () -> AllianceFlipping.apply(new Pose2d(2, 2, Rotation2d.fromDegrees(60))));
        if (drive.isAtAlignSetpoint(0.05, 2)) {
          currentState = AutoStates.ALIGN2;
          timer.reset();
        }
        break;
      case ALIGN2:
        drive.setStateAutoAlign(() -> (AllianceFlipping.apply(new Pose2d())));
        if (drive.isAtAlignSetpoint(0.06, 3)) currentState = AutoStates.ALIGN_REEF2;
        break;
      case ALIGN_REEF2:
        drive.setStateAutoAlign(() -> (AllianceFlipping.apply(new Pose2d())));
        if (drive.isAtAlignSetpoint(0.03, 3)) {
          currentState = AutoStates.PLACE2;
          timer.reset();
          timer.start();
        }
        break;
      case PLACE2:
        if (timer.get() < 1.0) {
          leds.setState(ledsStates.FINISH_SCORE);
        } else {
          currentState = AutoStates.END;
          timer.reset();
        }
        break;
      default:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentState == AutoStates.END;
  }
}
