package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystemWithFollowers;
import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {

  private enum ShooterOverrideMode {
    DISABLED,
    VOLTAGE,
    VELOCITY
  }

  private static final String DASHBOARD_PREFIX = "Shooter/Override/";
  private static final String MODE_CHOOSER_KEY = DASHBOARD_PREFIX + "Mode";
  private static final String VOLTAGE_KEY = DASHBOARD_PREFIX + "Voltage";
  private static final String VELOCITY_KEY = DASHBOARD_PREFIX + "Velocity";

  public enum ShooterStates {
    IDLE,
    SHOOTING
    }

  @AutoLogOutput(key = "Shooter/currentState")
  private ShooterStates currentState = ShooterStates.IDLE;

  // private final ShooterWheel mainWheel, hoodWheel;
  private SimpleMotorFeedforward mainWheelFFController; //hoodWheelFFController;
  private DoubleSupplier mainWheelVelocity;// hoodWheelVelocity;
  private final SendableChooser<ShooterOverrideMode> overrideModeChooser = new SendableChooser<>();

  private SysIdRoutine mainWheelRoutine;
  // private SysIdRoutine hoodWheelRoutine;

  //leader input io inputs ios
  public Shooter(
    MotorSubsystemWithFollowersConfig config,
    MotorIO io,
    MotorIO followerIo
  ) {
    super(config, new MotorInputsAutoLogged(), io, new MotorInputsAutoLogged[]{new MotorInputsAutoLogged()}, new MotorIO[]{followerIo} );
    mainWheelFFController =
        new SimpleMotorFeedforward(
            ShooterConstants.MAIN_WHEEL_FF_CONSTANTS.kS(),
            ShooterConstants.MAIN_WHEEL_FF_CONSTANTS.kV(),
            ShooterConstants.MAIN_WHEEL_FF_CONSTANTS.kA(),
            Constants.CYCLE_TIME);

    // hoodWheelFFController =
    //     new SimpleMotorFeedforward(
    //         ShooterConstants.HOOD_WHEEL_FF_CONSTANTS.kS(),
    //         ShooterConstants.HOOD_WHEEL_FF_CONSTANTS.kV(),
    //         ShooterConstants.HOOD_WHEEL_FF_CONSTANTS.kA(),
    //         Constants.CYCLE_TIME);

    // mainWheel = new ShooterWheel(mainWheelFFController, "MainWheel", mainWheelConfig, mainMotorIO);
    // hoodWheel = new ShooterWheel(hoodWheelFFController, "HoodWheel", hoodWheelConfig, hoodMotorIO);

    mainWheelVelocity = () -> 0;
    // hoodWheelVelocity = () -> 0;

    overrideModeChooser.setDefaultOption("Disabled", ShooterOverrideMode.DISABLED);
    overrideModeChooser.addOption("Voltage", ShooterOverrideMode.VOLTAGE);
    overrideModeChooser.addOption("Velocity", ShooterOverrideMode.VELOCITY);
    SmartDashboard.putData(MODE_CHOOSER_KEY, overrideModeChooser);
    SmartDashboard.putNumber(VOLTAGE_KEY, 0.0);
    SmartDashboard.putNumber(VELOCITY_KEY, 0.0);

    mainWheelRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(10),
        null,
        (state) -> Logger.recordOutput("Shooter/mainWheelSysidState", state.toString())
      ), 
      new SysIdRoutine.Mechanism(
        (volt) -> super.setVoltageOutput(volt.in(Volts)),
        null, 
        this)
      );
  }

  @Override
  public void periodic() {
    super.periodic();
    if (!runDashboardOverride()) {
      stateMachine();
      super.setMaxMotionSetpointVelocity(
              mainWheelVelocity.getAsDouble(),
              mainWheelFFController.calculate(mainWheelVelocity.getAsDouble()));
    }
  }

  private boolean runDashboardOverride() {
    ShooterOverrideMode mode = overrideModeChooser.getSelected();
    if (mode == null) {
      mode = ShooterOverrideMode.DISABLED;
    }

    Logger.recordOutput("Shooter/overrideMode", mode.toString());

    switch (mode) {
      case DISABLED -> {
        return false;
      }
      case VOLTAGE -> {
        super.setVoltageOutput(SmartDashboard.getNumber(VOLTAGE_KEY, 0.0));
        return true;
      }
      case VELOCITY -> {
        double velocitySetpoint = SmartDashboard.getNumber(VELOCITY_KEY, 0.0);
        super.setMaxMotionSetpointVelocity(
            velocitySetpoint, mainWheelFFController.calculate(velocitySetpoint));
        return true;
      }
    }

    return false;
  }

  private void stateMachine() {

    switch (currentState) {
      case IDLE -> {
        setStateIdle();
      }
      case SHOOTING -> {
        super.setMaxMotionSetpointVelocity(mainWheelVelocity.getAsDouble(), mainWheelFFController.calculate(mainWheelVelocity.getAsDouble()));
        // mainWheel.runVelocity(()->0);
        // hoodWheel.runVelocity(()->0);
      }
      // case HOLD -> {
      //   holdVelocity(mainWheelVelocity);
      // }
    }
  }

  public void setState(ShooterStates wantedState) {
    if (currentState != wantedState) currentState = wantedState;
  }

  public void setStateIdle() {
    currentState = ShooterStates.IDLE;
    mainWheelVelocity = () -> 0;
  }

  // /**
  //  * Runs the shooter wheels at the supplied velocities.
  //  * @param mainWheelVelocity Main Wheel velocity [rps]
  //  * @param hoodWheelVelocity Hood Wheel velocity [rps]
  //  */
  // public void runVelocity(DoubleSupplier mainWheelVelocity, DoubleSupplier hoodWheelVelocity) {
  //   this.mainWheelVelocity = mainWheelVelocity;
  //   this.hoodWheelVelocity = hoodWheelVelocity;
  //   setState(ShooterStates.SHOOTING);
  // }

  /**
   * Runs the shooter wheels at the same velocity.
   * @param shooterVelocity [rps]
   */
  public void runVelocity(DoubleSupplier shooterVelocity) {
    // runVelocity(shooterVelocity, shooterVelocity);
    this.mainWheelVelocity = shooterVelocity;
    setState(ShooterStates.SHOOTING);
  }

  /**
   * Runs the shooter wheels at the velocity needed to achieve the supplied exit velocity.
   * @param exitVelocity [m/s]
   */
  public void runExitVelocity(DoubleSupplier exitVelocity) {
    double mainWheelRPM = velocityToRPM(exitVelocity.getAsDouble(), ShooterConstants.MAIN_WHEEL_DIAMETER)
      * ShooterConstants.COMPENSATION_FACTOR * ShooterConstants.BACKSPIN_FACTOR;

    // double hoodWheelRPM = velocityToRPM(exitVelocity.getAsDouble(), ShooterConstants.HOOD_WHEEL_DIAMETER)
    //   * ShooterConstants.COMPENSATION_FACTOR;
    runVelocity(() -> mainWheelRPM/60); // to rps
  }

    public Command shooterMainSysidRoutine(boolean quasistatic, SysIdRoutine.Direction direction) {
    return Commands.either(
      mainWheelRoutine.quasistatic(direction),
      mainWheelRoutine.dynamic(direction),
      () -> quasistatic
    );
  }

  public boolean atVelocity() {
    return Math.abs(inputs.velocityUnitsPerSecond - mainWheelVelocity.getAsDouble()) < 5;
  }
  private double velocityToRPM(double velocityMetersPerSec, double diameterMeters) {
    return (60 * velocityMetersPerSec) / (Math.PI * diameterMeters);
  }
}

// package frc.robot.subsystems.shooter;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.lib.subsystems.MotorIO;
// import frc.lib.subsystems.MotorSubsystemConfig;
// import frc.robot.Constants;

// import static edu.wpi.first.units.Units.Volts;
// import static frc.robot.Constants.RobotState.SHOOTER_DISTANCE_FROM_CENTER;

// import java.util.concurrent.ForkJoinPool.ManagedBlocker;
// import java.util.function.DoubleSupplier;
// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

// public class Shooter extends SubsystemBase {

//   public enum ShooterStates {
//     IDLE,
//     SHOOTING,
//     HOLD
//   }

//   @AutoLogOutput(key = "Shooter/currentState")
//   private ShooterStates currentState = ShooterStates.IDLE;

//   private final ShooterWheel mainWheel, hoodWheel;
//   private SimpleMotorFeedforward mainWheelFFController, hoodWheelFFController;
//   private DoubleSupplier mainWheelVelocity;// hoodWheelVelocity;

//   private SysIdRoutine mainWheelRoutine;
//   private SysIdRoutine hoodWheelRoutine;

//   public Shooter(
//       MotorSubsystemConfig mainWheelConfig,
//       MotorIO mainMotorIO,
//       MotorSubsystemConfig hoodWheelConfig,
//       MotorIO hoodMotorIO) {

//     mainWheelFFController =
//         new SimpleMotorFeedforward(
//             ShooterConstants.MAIN_WHEEL_FF_CONSTANTS.kS(),
//             ShooterConstants.MAIN_WHEEL_FF_CONSTANTS.kV(),
//             ShooterConstants.MAIN_WHEEL_FF_CONSTANTS.kA(),
//             Constants.CYCLE_TIME);

//     hoodWheelFFController =
//         new SimpleMotorFeedforward(
//             ShooterConstants.HOOD_WHEEL_FF_CONSTANTS.kS(),
//             ShooterConstants.HOOD_WHEEL_FF_CONSTANTS.kV(),
//             ShooterConstants.HOOD_WHEEL_FF_CONSTANTS.kA(),
//             Constants.CYCLE_TIME);

//     mainWheel = new ShooterWheel(mainWheelFFController, "MainWheel", mainWheelConfig, mainMotorIO);
//     hoodWheel = new ShooterWheel(hoodWheelFFController, "HoodWheel", hoodWheelConfig, hoodMotorIO);

//     mainWheelVelocity = () -> 0;
//     // hoodWheelVelocity = () -> 0;

//     mainWheelRoutine = new SysIdRoutine(
//       new SysIdRoutine.Config(
//         null,
//         Volts.of(10),
//         null,
//         (state) -> Logger.recordOutput("Shooter/mainWheelSysidState", state.toString())
//       ), 
//       new SysIdRoutine.Mechanism(
//         (volt) -> mainWheel.setVoltageOutput(volt.in(Volts)),
//         null, 
//         mainWheel)
//       );

//     hoodWheelRoutine = new SysIdRoutine(
//       new SysIdRoutine.Config(
//         null,
//         Volts.of(10),
//         null,
//         (state) -> Logger.recordOutput("Shooter/hoodWheelSysidState", state.toString())
//       ), 
//       new SysIdRoutine.Mechanism(
//         (volt) -> hoodWheel.setVoltageOutput(volt.in(Volts)),
//         null, 
//         hoodWheel)
//       );
//   }

//   @Override
//   public void periodic() {
//     mainWheel.realPeriodic();
//     hoodWheel.realPeriodic();
//     stateMachine();
//   }

//   private void stateMachine() {

//     switch (currentState) {
//       case IDLE -> {
//         mainWheel.setStateIdle();
//         hoodWheel.setStateIdle();
//       }
//       case SHOOTING -> {
//         mainWheel.runVelocity(mainWheelVelocity);
//         hoodWheel.runVelocity(mainWheelVelocity);
//         // mainWheel.runVelocity(()->0);
//         // hoodWheel.runVelocity(()->0);
//       }
//       case HOLD -> {
//         mainWheel.holdVelocity(mainWheelVelocity);
//         hoodWheel.holdVelocity(mainWheelVelocity);
//       }
//     }
//   }

//   public void setState(ShooterStates wantedState) {
//     if (currentState != wantedState) currentState = wantedState;
//   }

//   // /**
//   //  * Runs the shooter wheels at the supplied velocities.
//   //  * @param mainWheelVelocity Main Wheel velocity [rps]
//   //  * @param hoodWheelVelocity Hood Wheel velocity [rps]
//   //  */
//   // public void runVelocity(DoubleSupplier mainWheelVelocity, DoubleSupplier hoodWheelVelocity) {
//   //   this.mainWheelVelocity = mainWheelVelocity;
//   //   this.hoodWheelVelocity = hoodWheelVelocity;
//   //   setState(ShooterStates.SHOOTING);
//   // }

//   /**
//    * Runs the shooter wheels at the same velocity.
//    * @param shooterVelocity [rps]
//    */
//   public void runVelocity(DoubleSupplier shooterVelocity) {
//     // runVelocity(shooterVelocity, shooterVelocity);
//     this.mainWheelVelocity = shooterVelocity;
//     setState(ShooterStates.SHOOTING);
//   }

//   /**
//    * Runs the shooter wheels at the velocity needed to achieve the supplied exit velocity.
//    * @param exitVelocity [m/s]
//    */
//   public void runExitVelocity(DoubleSupplier exitVelocity) {
//     double mainWheelRPM = velocityToRPM(exitVelocity.getAsDouble(), ShooterConstants.MAIN_WHEEL_DIAMETER)
//       * ShooterConstants.COMPENSATION_FACTOR * ShooterConstants.BACKSPIN_FACTOR;

//     // double hoodWheelRPM = velocityToRPM(exitVelocity.getAsDouble(), ShooterConstants.HOOD_WHEEL_DIAMETER)
//     //   * ShooterConstants.COMPENSATION_FACTOR;
//     runVelocity(() -> mainWheelRPM/60); // to rps
//   }

//     public Command shooterMainSysidRoutine(boolean quasistatic, SysIdRoutine.Direction direction) {
//     return Commands.either(
//       mainWheelRoutine.quasistatic(direction),
//       mainWheelRoutine.dynamic(direction),
//       () -> quasistatic
//     );
//   }

//   public Command shooterHoodSysidRoutine(boolean quasistatic, SysIdRoutine.Direction direction) {
//     return Commands.either(
//       hoodWheelRoutine.quasistatic(direction),
//       hoodWheelRoutine.dynamic(direction),
//       () -> quasistatic
//     );
//   }

//   public boolean atVelocity() {
//     return mainWheel.atVelocity() && hoodWheel.atVelocity();
//   }
//   private double velocityToRPM(double velocityMetersPerSec, double diameterMeters) {
//     return (60 * velocityMetersPerSec) / (Math.PI * diameterMeters);
//   }
// }
