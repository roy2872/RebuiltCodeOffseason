package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeDeployConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorSubsystemConfig;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeDeploy.IntakeDeployStates;
import frc.robot.subsystems.intake.IntakeRollers.IntakeRollerStates;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private enum IntakeRollerOverrideMode {
    DISABLED,
    VOLTAGE
  }

  private enum IntakeDeployOverrideMode {
    DISABLED,
    VOLTAGE,
    POSITION
  }

  private static final String DASHBOARD_PREFIX = "Intake/Override/";
  private static final String ROLLER_MODE_CHOOSER_KEY = DASHBOARD_PREFIX + "RollersMode";
  private static final String ROLLER_VOLTAGE_KEY = DASHBOARD_PREFIX + "RollersVoltage";
  private static final String DEPLOY_MODE_CHOOSER_KEY = DASHBOARD_PREFIX + "DeployMode";
  private static final String DEPLOY_VOLTAGE_KEY = DASHBOARD_PREFIX + "DeployVoltage";
  private static final String DEPLOY_POSITION_KEY = DASHBOARD_PREFIX + "DeployPosition";

  public enum IntakeStates {
    IDLE,
    INTAKE,
    OPEN,
    PURGE,
    CLOSED,
    SHUFFLE
  }

  @AutoLogOutput(key = "Intake/currentState")
  private IntakeStates currentState = IntakeStates.IDLE;

  private final IntakeRollers rollers;
  private final IntakeDeploy deploy;
  private SimpleMotorFeedforward rollersFFController;
  private ArmFeedforward deployFFController;
  private final SendableChooser<IntakeRollerOverrideMode> rollerOverrideModeChooser =
      new SendableChooser<>();
  private final SendableChooser<IntakeDeployOverrideMode> deployOverrideModeChooser =
      new SendableChooser<>();
  
  private SysIdRoutine rollersRoutine;
  private SysIdRoutine deployRoutine;

  public Intake(
      MotorSubsystemConfig rollerConfig,
      MotorIO rollerIO,
      MotorSubsystemConfig deployConfig,
      MotorIO deployIO) {

    rollersFFController =
        new SimpleMotorFeedforward(
            INTAKE_ROLLER_FF.kS(),
            INTAKE_ROLLER_FF.kV(),
            INTAKE_ROLLER_FF.kA(),
            Constants.CYCLE_TIME);

    deployFFController =
        new ArmFeedforward(
            INTAKE_DEPLOY_FF.kS(),
            INTAKE_DEPLOY_FF.kg(),
            INTAKE_DEPLOY_FF.kV(),
            INTAKE_DEPLOY_FF.kA(),
            Constants.CYCLE_TIME);

    rollers = new IntakeRollers(rollersFFController, rollerConfig, rollerIO);
    deploy = new IntakeDeploy(deployFFController, deployConfig, deployIO);

    rollersRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        (state) -> Logger.recordOutput("Intake/rollerSysidState", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> rollers.setVoltageOutput(volts.in(Volts)),
        null, 
        rollers
      )
    );

    deployRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds), 
        Volts.of(4), 
        Seconds.of(2),
        (state) -> Logger.recordOutput("Intake/deploySysidState", state.toString())
      ), 
      new SysIdRoutine.Mechanism(
        (volts) -> deploy.setVoltageOutput(volts.in(Volts)),
        null,
        deploy
      )
    );

    rollerOverrideModeChooser.setDefaultOption("Disabled", IntakeRollerOverrideMode.DISABLED);
    rollerOverrideModeChooser.addOption("Voltage", IntakeRollerOverrideMode.VOLTAGE);
    SmartDashboard.putData(ROLLER_MODE_CHOOSER_KEY, rollerOverrideModeChooser);
    SmartDashboard.putNumber(ROLLER_VOLTAGE_KEY, 0.0);

    deployOverrideModeChooser.setDefaultOption("Disabled", IntakeDeployOverrideMode.DISABLED);
    deployOverrideModeChooser.addOption("Voltage", IntakeDeployOverrideMode.VOLTAGE);
    deployOverrideModeChooser.addOption("Position", IntakeDeployOverrideMode.POSITION);
    SmartDashboard.putData(DEPLOY_MODE_CHOOSER_KEY, deployOverrideModeChooser);
    SmartDashboard.putNumber(DEPLOY_VOLTAGE_KEY, 0.0);
    SmartDashboard.putNumber(DEPLOY_POSITION_KEY, INTAKE_CLOSED_ANGLE);
  }

  @Override
  public void periodic() {
    rollers.realPeriodic();
    deploy.realPeriodic();
    if (!runDashboardOverrides()) {
      stateMachine();
    }
  }

  private boolean runDashboardOverrides() {
    IntakeRollerOverrideMode rollerMode = rollerOverrideModeChooser.getSelected();
    if (rollerMode == null) {
      rollerMode = IntakeRollerOverrideMode.DISABLED;
    }

    IntakeDeployOverrideMode deployMode = deployOverrideModeChooser.getSelected();
    if (deployMode == null) {
      deployMode = IntakeDeployOverrideMode.DISABLED;
    }

    Logger.recordOutput("Intake/overrideRollerMode", rollerMode.toString());
    Logger.recordOutput("Intake/overrideDeployMode", deployMode.toString());

    boolean overrideEnabled =
        rollerMode != IntakeRollerOverrideMode.DISABLED
            || deployMode != IntakeDeployOverrideMode.DISABLED;

    if (!overrideEnabled) {
      return false;
    }

    switch (rollerMode) {
      case DISABLED -> rollers.setVoltageOutput(0.0);
      case VOLTAGE -> rollers.setVoltageOutput(SmartDashboard.getNumber(ROLLER_VOLTAGE_KEY, 0.0));
    }

    switch (deployMode) {
      case DISABLED -> deploy.setVoltageOutput(0.0);
      case VOLTAGE -> deploy.setVoltageOutput(SmartDashboard.getNumber(DEPLOY_VOLTAGE_KEY, 0.0));
      case POSITION -> deploy.setOverridePosition(
          SmartDashboard.getNumber(DEPLOY_POSITION_KEY, INTAKE_CLOSED_ANGLE));
    }

    return true;
  }

  private void stateMachine() {

    switch (currentState) {
      case IDLE -> {
        rollers.setState(IntakeRollerStates.IDLE);
        deploy.setState(IntakeDeployStates.IDLE);
      }
      case INTAKE -> {
        rollers.setState(IntakeRollerStates.INTAKE);
        deploy.setState(IntakeDeployStates.OPEN);
      }
      case OPEN -> {
        rollers.setState(IntakeRollerStates.IDLE);
        deploy.setState(IntakeDeployStates.OPEN);
      }
      case PURGE -> {
        rollers.setState(IntakeRollerStates.PURGE);
        deploy.setState(IntakeDeployStates.OPEN);
      }
      case CLOSED -> {
        rollers.setState(IntakeRollerStates.IDLE);
        deploy.setState(IntakeDeployStates.CLOSED);
      }
      case SHUFFLE -> {
        rollers.setState(IntakeRollerStates.INTAKE);
        deploy.setState(IntakeDeployStates.CLOSED);
      }
    }
  }

  public void setState(IntakeStates wantedState) {
    if (currentState != wantedState) currentState = wantedState;
  }

  public Command rollersSysId(boolean quasistatic, SysIdRoutine.Direction direction) {
    return Commands.either(
      rollersRoutine.quasistatic(direction),
      rollersRoutine.dynamic(direction),
      () -> quasistatic
    );
  }

    public Command deploySysId(boolean quasistatic, SysIdRoutine.Direction direction) {
      return Commands.print(
        "Running sysid on this mechanism is dangerous, " +
        "please uncomment the following code if you are sure of what you're doing."
      );
      // return Commands.either(
      //   deployRoutine.quasistatic(direction),
      //   deployRoutine.dynamic(direction),
      //   () -> quasistatic
      // );
  }
  
}
