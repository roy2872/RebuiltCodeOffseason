package frc.lib.io;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.UnaryOperator;

/**
 * Abstract class used to control a main motor and any number of followers for a mechanism.
 * * @param <C> The configuration object type specific to the motor controller vendor (e.g., TalonFXConfiguration or SparkMaxConfig).
 */
public abstract class MotorIO<C> implements Sendable {
  public final AngleUnit unitType;
  public final TimeUnit time;
  protected final Inputs inputs;
  protected final Inputs[] followerInputs;
  // Use a wildcard to decouple Setpoint initialization from strict configuration binding details
  private Setpoint<?> setpoint = Setpoint.withNeutralSetpoint();
  private boolean enabled = true;

  /**
   * Updates MotorIO's inputs with values from the motor.
   */
  public abstract void updateInputs();

  /**
   * Set's the mechanism's current location as a given position.
   *
   * @param mechanismPosition the mechanism's position to set location as.
   */
  public abstract void setCurrentPosition(Angle mechanismPosition);

  /**
   * Set's the mechanism's current location as zero.
   */
  public abstract void zeroSensors();

  /**
   * Sets the motor to brake or coast.
   *
   * @param wantsBrake Whether to brake or coast. True is brake, false is coast.
   */
  public abstract void setNeutralBrake(boolean wantsBrake);

  /**
   * Gets the active device configurations tracker.
   */
  public abstract C getMotorIOConfig();

  /**
   * Gets whether or not the config is applied successfully to the motor
   */
  public abstract boolean getConfigFailed();

  /**
   * Sets whether to enable or disable soft limits.
   *
   * @param enable Whether to enable or disable soft limits. True is enable, false is disable.
   */
  public abstract void useSoftLimits(boolean enable);

  /**
   * Sets the motor to be idle. Should not be called directly, only applied through Setpoints.
   */
  protected abstract void setNeutralSetpoint();

  /**
   * Sets the motor to be coasting. Should not be called directly, only applied through Setpoints.
   */
  protected abstract void setCoastSetpoint();

  /**
   * Sets the motor to run at at given voltage. Should not be called directly, only applied through Setpoints.
   *
   * @param voltage Voltage to run at.
   */
  protected abstract void setVoltageSetpoint(Voltage voltage);

  /**
   * Sets the motor to use motion magic/smart motion control to go to a given position. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismPosition Mechanism position to go to.
   * @param slot The PID slot to assign
   */
  protected abstract void setMotionMagicSetpoint(Angle mechanismPosition, int slot);

  /**
   * Sets the motor to use motion magic/smart motion control to go to a given position. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismPosition Mechanism position to go to.
   */
  protected abstract void setMotionMagicSetpoint(Angle mechanismPosition);

  /**
   * Sets the motor to go to a given velocity. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismVelocity Mechanism velocity to go to.
   * @param slot The PID slot to assign
   */
  protected abstract void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot);

  /**
   * Sets the motor to go to a given velocity. Should not be called directly, only applied through Setpoints. DEFAULTS TO SLOT 1
   *
   * @param mechanismVelocity Mechanism velocity to go to.
   */
  protected abstract void setVelocitySetpoint(AngularVelocity mechanismVelocity);

  /**
   * Sets the motor to run at a percentage of it's max voltage. Should not be called directly, only applied through Setpoints.
   *
   * @param percent Percentage of max voltage to run at.
   */
  protected abstract void setDutyCycleSetpoint(Dimensionless percent);

  /**
   * Sets the motor to use PID control to go to a given position. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismPosition Mechanism position to go to.
   * @param slot The PID slot to assign
   */
  protected abstract void setPositionSetpoint(Angle mechanismPosition, int slot);

  /**
   * Sets the motor to use PID control to go to a given position. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismPosition Mechanism position to go to.
   */
  protected abstract void setPositionSetpoint(Angle mechanismPosition);

  public abstract void setMainConfig(C configuration);

  public abstract void changeMainConfig(UnaryOperator<C> configChanger);

  public abstract void changeFollowerConfig(UnaryOperator<C> configChanger);

  /**
   * Applies a Setpoint to the MotorIO.
   *
   * @param setpointToApply
   */
@SuppressWarnings("unchecked")
  public final void applySetpoint(Setpoint<C> setpointToApply) {
    setpoint = setpointToApply;
    if (enabled) {
      setpointToApply.apply(this);
    }
  }

  @SuppressWarnings("unchecked")
  public final void enable() {
    enabled = true;
    ((Setpoint<C>) setpoint).apply(this);
  }

  @SuppressWarnings("unchecked")
  public final void disable() {
    enabled = false;
    ((Setpoint<C>) Setpoint.withNeutralSetpoint()).apply(this);
  }

  public abstract void disabledPeriodic();

  /**
   * Gets whether this MotorIO is enabled.
   *
   * @return True if enabled, false if disabled.
   */
  public boolean getEnabled() {
    return enabled;
  }

  /**
   * Constructs a MotorIO with no follower motors.
   *
   * @param unit Units to measure.
   * @param time Time units to measure.
   */
  protected MotorIO(AngleUnit unit, TimeUnit time) {
    this(unit, time, 0);
  }

  /**
   * Constructs a MotorIO with a given number of follower motors.
   *
   * @param unit Units to measure in.
   * @param time Time units to measure.
   * @param numFollowers The number of follower motors.
   */
  protected MotorIO(AngleUnit unit, TimeUnit time, int numFollowers) {
    this.unitType = unit;
    this.time = time;
    inputs = new Inputs();

// Suppress the unchecked warning caused by the generic array cast
    @SuppressWarnings("unchecked")
    Inputs[] tempFollowers = (Inputs[]) new MotorIO.Inputs[numFollowers];
    followerInputs = tempFollowers;

    for (int i = 0; i < numFollowers; i++) {
        followerInputs[i] = new Inputs();
    }
  }

  /**
   * Gets the last read velocity of the main motor.
   *
   * @return Velocity of mechanism.
   */
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /**
   * Gets the last read position of the main motor.
   *
   * @return Position of mechanism.
   */
  public Angle getPosition() {
    return inputs.position;
  }

  /**
   * Gets the last read stator current of the main motor.
   *
   * @return Stator current.
   */
  public Current getStatorCurrent() {
    return inputs.statorCurrent;
  }

  /**
   * Gets the last read supply current of the main motor.
   *
   * @return Supply current.
   */
  public Current getSupplyCurrent() {
    return inputs.supplyCurrent;
  }

  /**
   * Gets the last read output voltage of the main motor.
   *
   * @return Output voltage.
   */
  public Voltage getMotorVoltage() {
    return inputs.motorVoltage;
  }

  /**
   * Gets the last applied setpoint of the MotorIO.
   *
   * @return Last applied Setpoint.
   */
  public Setpoint<?> getSetpoint() {
    return setpoint;
  }

  /**
   * Gets the current setpoint value of the MotorIO using units of the MotorIO.
   *
   * @return Setpoint in mechanism units.
   */
  public double getSetpointDoubleInUnits() {
    Setpoint<?> currentSetpoint = getSetpoint();
    switch (currentSetpoint.mode) {
      case POSITIONPID:
      case MOTIONMAGIC:
        AngleUnit positionUnit = unitType;
        return positionUnit.ofBaseUnits(currentSetpoint.baseUnits).in(positionUnit);
      case VELOCITY:
        AngularVelocityUnit velocityUnit = unitType.per(time);
        return velocityUnit.ofBaseUnits(currentSetpoint.baseUnits).in(velocityUnit);
      case VOLTAGE:
        VoltageUnit voltageUnit = Units.Volts;
        return voltageUnit.ofBaseUnits(currentSetpoint.baseUnits).in(voltageUnit);
      case DUTY_CYCLE:
        DimensionlessUnit percentUnit = Units.Percent;
        return percentUnit.ofBaseUnits(currentSetpoint.baseUnits).in(percentUnit);
      case IDLE:
      default:
        return currentSetpoint.baseUnits;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Enabled", () -> getEnabled(), null);
    builder.addStringProperty("Setpoint Type:", () -> getSetpoint().mode.toString(), null);
    builder.addDoubleProperty("Setpoint Value as Double:", () -> getSetpointDoubleInUnits(), null);
    inputs.initSendable(builder);
    if (followerInputs.length > 0) {
      builder.addDoubleArrayProperty(
          "Followers/Velocity " + unitType.name() + " per " + time.name() + ":",
          () -> {
            double[] velocitiesUnits = new double[followerInputs.length];
            for (int i = 0; i < followerInputs.length; i++) {
              velocitiesUnits[i] = followerInputs[i].velocity.in(unitType.per(time));
            }
            return velocitiesUnits;
          },
          null);
      builder.addDoubleArrayProperty(
          "Followers/Position " + unitType.name() + ":",
          () -> {
            double[] positionsUnits = new double[followerInputs.length];
            for (int i = 0; i < followerInputs.length; i++) {
              positionsUnits[i] = followerInputs[i].position.in(unitType);
            }
            return positionsUnits;
          },
          null);
      builder.addDoubleArrayProperty(
          "Followers/Stator Current:",
          () -> {
            double[] statorCurrents = new double[followerInputs.length];
            for (int i = 0; i < followerInputs.length; i++) {
              statorCurrents[i] = followerInputs[i].statorCurrent.in(Units.Amps);
            }
            return statorCurrents;
          },
          null);
      builder.addDoubleArrayProperty(
          "Followers/Supply Current:",
          () -> {
            double[] supplyCurrents = new double[followerInputs.length];
            for (int i = 0; i < followerInputs.length; i++) {
              supplyCurrents[i] = followerInputs[i].supplyCurrent.in(Units.Amps);
            }
            return supplyCurrents;
          },
          null);
      builder.addDoubleArrayProperty(
          "Followers/Motor Voltage:",
          () -> {
            double[] motorVoltages = new double[followerInputs.length];
            for (int i = 0; i < followerInputs.length; i++) {
              motorVoltages[i] = followerInputs[i].motorVoltage.in(Units.Volts);
            }
            return motorVoltages;
          },
          null);
      builder.addDoubleArrayProperty(
          "Followers/Motor Temperature Celsius:",
          () -> {
            double[] motorTemperatures = new double[followerInputs.length];
            for (int i = 0; i < followerInputs.length; i++) {
              motorTemperatures[i] = followerInputs[i].motorTemperature.in(Units.Celsius);
            }
            return motorTemperatures;
          },
          null);
      builder.addBooleanProperty("ConfigFailed", () -> getConfigFailed(), null);
    }
  }

  public class Inputs implements Sendable {
    public AngularVelocity velocity = BaseUnits.AngleUnit.of(0.0).per(Units.Second);
    public Angle position = BaseUnits.AngleUnit.of(0.0);
    public Current statorCurrent = BaseUnits.CurrentUnit.of(0.0);
    public Current supplyCurrent = BaseUnits.CurrentUnit.of(0.0);
    public Voltage motorVoltage = BaseUnits.VoltageUnit.of(0.0);
    public Voltage pidVoltage = BaseUnits.VoltageUnit.of(0.0);
    public Temperature motorTemperature = BaseUnits.TemperatureUnit.of(0.0);
    public AngularAcceleration acceleration = BaseUnits.AngleUnit.of(0.0).per(Units.Seconds).per(Units.Seconds);

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty(
          "Velocity " + unitType.name() + " per " + time.name() + ":",
          () -> velocity.in(unitType.per(time)),
          null);
      builder.addDoubleProperty("Position " + unitType.name() + ":", () -> position.in(unitType), null);
      builder.addDoubleProperty("Stator Current Amps:", () -> statorCurrent.in(Units.Amps), null);
      builder.addDoubleProperty("Supply Current Amps:", () -> supplyCurrent.in(Units.Amps), null);
      builder.addDoubleProperty("Motor Voltage:", () -> motorVoltage.in(Units.Volts), null);
      builder.addDoubleProperty("PID Voltage:", () -> pidVoltage.in(Units.Volts), null);
      builder.addDoubleProperty("Motor Temperature Celsius:", () -> motorTemperature.in(Units.Celsius), null);
      builder.addDoubleProperty(
          "Acceleration" + unitType.name() + " per " + time.name() + " per " + time.name() + ":",
          () -> acceleration.in(unitType.per(time).per(time)),
          null);
    }
  }

  public enum Mode {
    IDLE,
    VOLTAGE,
    MOTIONMAGIC,
    VELOCITY,
    DUTY_CYCLE,
    POSITIONPID;

    public boolean isPositionControl() {
      return switch (this) {
        case MOTIONMAGIC, POSITIONPID -> true;
        default -> false;
      };
    }

    public boolean isVelocityControl() {
      return switch (this) {
        case VELOCITY -> true;
        default -> false;
      };
    }

    public boolean isNeutralControl() {
      return switch (this) {
        case IDLE -> true;
        default -> false;
      };
    }

    public boolean isVoltageControl() {
      return switch (this) {
        case VOLTAGE, DUTY_CYCLE -> true;
        default -> false;
      };
    }
  }

  /**
   * Setpoint bound to a specific hardware configuration track.
   */
  public static class Setpoint<C> {

    @SuppressWarnings("rawtypes")
    public static final Setpoint COAST = new Setpoint<>(
        io -> {
          io.setCoastSetpoint();
          return io;
        },
        Mode.IDLE,
        0.0);

    @SuppressWarnings("rawtypes")
    public static final Setpoint NEUTRAL = new Setpoint<>(
        io -> {
          io.setNeutralSetpoint();
          return io;
        },
        Mode.IDLE,
        0);

    private final UnaryOperator<MotorIO<C>> applier;
    public final Mode mode;
    public final double baseUnits;

    private Setpoint(UnaryOperator<MotorIO<C>> applier, Mode mode, double baseUnits) {
      this.applier = applier;
      this.mode = mode;
      this.baseUnits = baseUnits;
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withCustomSetpoint(UnaryOperator<MotorIO<C>> applier, Mode mode, double baseUnits) {
      return new Setpoint<>(applier, mode, baseUnits);
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withMotionMagicSetpoint(Angle motionMagicSetpoint) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setMotionMagicSetpoint(motionMagicSetpoint);
        return io;
      };
      return new Setpoint<>(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withPositionSetpoint(Angle positionSetpoint) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setPositionSetpoint(positionSetpoint);
        return io;
      };
      return new Setpoint<>(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withVelocitySetpoint(AngularVelocity velocitySetpoint) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setVelocitySetpoint(velocitySetpoint);
        return io;
      };
      return new Setpoint<>(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withVoltageSetpoint(Voltage voltage) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setVoltageSetpoint(voltage);
        return io;
      };
      return new Setpoint<>(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withMotionMagicSetpoint(Angle motionMagicSetpoint, int slot) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setMotionMagicSetpoint(motionMagicSetpoint, slot);
        return io;
      };
      return new Setpoint<>(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withPositionSetpoint(Angle positionSetpoint, int slot) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setPositionSetpoint(positionSetpoint, slot);
        return io;
      };
      return new Setpoint<>(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withVelocitySetpoint(AngularVelocity velocitySetpoint, int slot) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setVelocitySetpoint(velocitySetpoint, slot);
        return io;
      };
      return new Setpoint<>(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withDutyCycleSetpoint(Dimensionless percent) {
      UnaryOperator<MotorIO<C>> applier = (MotorIO<C> io) -> {
        io.setDutyCycleSetpoint(percent);
        return io;
      };
      return new Setpoint<>(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withNeutralSetpoint() {
      return NEUTRAL;
    }

    @SuppressWarnings("unchecked")
    public static <C> Setpoint<C> withCoastSetpoint() {
      return COAST;
    }

    public void apply(MotorIO<C> io) {
      applier.apply(io);
    }
  }
}