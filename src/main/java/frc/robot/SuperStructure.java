// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FetchCommand;
import frc.robot.commands.ShootCloseCommand;
import frc.robot.commands.ShootInPlaceCommand;
import frc.robot.commands.ShootOnTheMoveCommand;
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
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {

  public enum SuperStructureStates {
    AUTO,
    TRAVEL,
    SHOOT,
    SHOOT_CLOSE,
    SHOOT_ON_THE_MOVE,
    FETCH,
    PURGE_INTAKE
  }

  public enum StructureIntakeStates {
    INTAKING,
    IDLE,
    PURGE,
    CLOSED
  }

  @AutoLogOutput(key = "SuperStructure/intakeState")
  private StructureIntakeStates intakeState =  StructureIntakeStates.IDLE;

  @AutoLogOutput(key = "SuperStructure/currentState")
  private SuperStructureStates currentState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/wantedState")
  private SuperStructureStates wantedState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/previousState")
  private SuperStructureStates previousState = SuperStructureStates.TRAVEL;

  public Command currentCommand = null;

  private final BeltDrive beltDrive;
  private final Drive drive;
  private final Hood hood;
  // private final Hopper hopper;
  private final Intake intake;
  private final Leds leds;
  private final Shooter shooter;
  private final Vision vision;

  private boolean shouldPurgeIntake = false;

  public SuperStructure(
      BeltDrive beltDrive,
      Drive drive,
      Hood hood,
      // Hopper hopper,
      Intake intake,
      Leds leds,
      Shooter shooter,
      Vision vision) {
    this.beltDrive = beltDrive;
    this.drive = drive;
    this.hood = hood;
    // this.hopper = hopper;
    this.intake = intake;
    this.leds = leds;
    this.shooter = shooter;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    previousState = currentState;
    if (wantedState != currentState) currentState = handleStateTransition(wantedState);
    wantedState = currentState;
    intakeStateMachine();
    stateMachine();
  }

  private SuperStructureStates handleStateTransition(SuperStructureStates wantedState) {
    return switch (wantedState) {
      case AUTO -> {
        yield SuperStructureStates.AUTO;
      }

      case TRAVEL -> {
          yield SuperStructureStates.TRAVEL;
      }

      case SHOOT -> {
          yield SuperStructureStates.SHOOT;
      }

      case SHOOT_CLOSE -> {
        yield SuperStructureStates.SHOOT_CLOSE;
      }

      case SHOOT_ON_THE_MOVE -> {
          yield SuperStructureStates.SHOOT_ON_THE_MOVE;
      }

      case FETCH -> {
          yield SuperStructureStates.FETCH;
      }

      case PURGE_INTAKE -> {
          yield SuperStructureStates.PURGE_INTAKE;
      }

      default -> {
        System.out.println("SuperStructure: Invalid state transition requested: " + wantedState);
        yield currentState;
      }
    };
  }

  private void stateMachine() {
    switch (currentState) {
      case AUTO -> auto();

      case TRAVEL -> travel();

      case SHOOT -> shoot();

      case SHOOT_ON_THE_MOVE -> shootOnTheMove();

      case SHOOT_CLOSE -> shootClose();

      case FETCH -> fetch();

      case PURGE_INTAKE -> purgeIntake();
        
      default -> {}
    }
  }

  private void intakeStateMachine() {
    switch (intakeState) {
      case INTAKING -> {
        if (shouldPurgeIntake) intake.setState(Intake.IntakeStates.PURGE);
        else intake.setState(Intake.IntakeStates.INTAKE);
      }

      case IDLE -> {
        if(shouldPurgeIntake) intake.setState(Intake.IntakeStates.PURGE);
        else intake.setState(Intake.IntakeStates.OPEN);
      }

      case CLOSED -> {
        if(shouldPurgeIntake) intake.setState(Intake.IntakeStates.SHUFFLE);
        else intake.setState(Intake.IntakeStates.CLOSED);
      }

      default -> System.out.println("SuperStructure: Invalid intake state requested: " + intakeState);
    }
  }

  private void auto() {
    if (currentState != previousState) {
      // if (currentCommand != null) currentCommand.cancel();
      // currentCommand.schedule();
    }
  }

  private void travel() {
    if (currentState != previousState) {
      if (currentCommand != null) currentCommand.cancel();
    }
    drive.setState(DriveStates.FIELD_DRIVE);
    hood.setState(HoodStates.IDLE);
    beltDrive.setState(BeltDriveStates.IDLE);
    // hopper.setState(HopperStates.IDLE);
    shooter.setState(ShooterStates.IDLE);
    leds.setState(ledsStates.OFF);
    // intake function
  }

  private void shoot() {
    if (currentState != previousState) {
      if (currentCommand != null) currentCommand.cancel();
      currentCommand = 
          new ShootInPlaceCommand(beltDrive, drive, hood, leds, shooter);
      CommandScheduler.getInstance().schedule(currentCommand);
    }
  }

  private void shootClose() {
    if (currentState != previousState) {
      if (currentCommand != null) currentCommand.cancel();
        currentCommand = new ShootCloseCommand(beltDrive, drive, hood, leds, shooter);
        CommandScheduler.getInstance().schedule(currentCommand);
    }
  }

  private void shootOnTheMove() {
    if (currentState != previousState) {
      if (currentCommand != null) currentCommand.cancel();
      currentCommand = 
          new ShootOnTheMoveCommand(beltDrive, drive, hood, leds, shooter);
      CommandScheduler.getInstance().schedule(currentCommand);
    }
  }

  private void fetch() {
    if (currentState != previousState) {
      if (currentCommand != null) currentCommand.cancel();
      currentCommand = 
          new FetchCommand(beltDrive, drive, hood, leds, shooter);
      CommandScheduler.getInstance().schedule(currentCommand);
    }
  }

  private void purgeIntake() {
    if (currentState != previousState) {
      if (currentCommand != null) currentCommand.cancel();
    }
    intake.setState(IntakeStates.PURGE);
    hood.setState(HoodStates.IDLE);
    beltDrive.setState(BeltDriveStates.PURGE);
    // hopper.setState(HopperStates.PURGE);
    shooter.setState(ShooterStates.IDLE);
    drive.setState(DriveStates.FIELD_DRIVE);
  }

  public void setWantedState(SuperStructureStates wantedState) {
    this.wantedState = wantedState;
  }

  public void teleopInit() {
    if (currentCommand != null) currentCommand.cancel();
    else wantedState = SuperStructureStates.TRAVEL;
  }

  public void setIntakeState(StructureIntakeStates intakeState) {
    this.intakeState = intakeState;
  }

  public Command setWantedStateCommand(SuperStructureStates wantedState) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }

  public Command setIntakeStateCommand(StructureIntakeStates intakeState) {
    return new InstantCommand(() -> setIntakeState(intakeState));
  }

  public Command shootButtonCommand() {
    return Commands.either(setWantedStateCommand(SuperStructureStates.TRAVEL), 
      setWantedStateCommand(SuperStructureStates.SHOOT), () -> currentState == SuperStructureStates.SHOOT);
  }

  public Command shootCloseButtonCommand() {
    return Commands.either(setWantedStateCommand(SuperStructureStates.TRAVEL), 
      setWantedStateCommand(SuperStructureStates.SHOOT_CLOSE), () -> currentState == SuperStructureStates.SHOOT_CLOSE);
  }
  
  public Command shootOnTheMoveButtonCommand() {
    return Commands.either(setWantedStateCommand(SuperStructureStates.TRAVEL), 
      setWantedStateCommand(SuperStructureStates.SHOOT_ON_THE_MOVE), () -> currentState == SuperStructureStates.SHOOT_ON_THE_MOVE);
  }

  public Command intakeButtonCommand() {
    return Commands.either(setIntakeStateCommand(StructureIntakeStates.IDLE), 
      setIntakeStateCommand(StructureIntakeStates.INTAKING), () -> intakeState == StructureIntakeStates.INTAKING);
  }

  public Command closeIntakeButtonCommand() {
    return Commands.either(setIntakeStateCommand(StructureIntakeStates.IDLE), 
      setIntakeStateCommand(StructureIntakeStates.CLOSED), () -> intakeState == StructureIntakeStates.CLOSED);
  }

  public Command fetchButtonCommand() {
    return Commands.either(setWantedStateCommand(SuperStructureStates.TRAVEL), 
      setWantedStateCommand(SuperStructureStates.FETCH), () -> currentState == SuperStructureStates.FETCH);
  }

  public Command purgeIntakeButtonTrueCommand() {
    return Commands.runOnce(() -> {shouldPurgeIntake = true;});
  }

  public Command purgeIntakeButtonFalseCommand() {
    return Commands.runOnce(() -> {shouldPurgeIntake = false;});
  }
}
