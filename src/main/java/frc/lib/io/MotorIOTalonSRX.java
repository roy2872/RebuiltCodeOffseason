package frc.lib.io;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * Class used to control a main TalonSRX and any number of followers for a real mechanism.
 */
public class MotorIOTalonSRX extends MotorIO<TalonSRXConfiguration> {
    protected final TalonSRX main;
    protected final TalonSRX[] followers;
    protected TalonSRXConfiguration config;
    protected TalonSRXConfiguration followerConfig;
    protected final double ticksPerRotation;
    private final ControlModeSetter modeSetter;
    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor =
            new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);
    private boolean configFailed = false;

    public void applyConfig(TalonSRX talon, TalonSRXConfiguration config) {
        threadPoolExecutor.submit(() -> {
            for (int i = 0; i < 5; i++) {
                var result = talon.configAllSettings(config);
                if (result == ErrorCode.OK) {
                    break;
                } else if (i == 4) {
                    configFailed = true;
                }
            }
        });
    }

    @Override
    public void updateInputs() {
        updateMotorInputs(inputs, main);

        for (int i = 0; i < followers.length; i++) {
            updateMotorInputs(followerInputs[i], followers[i]);
        }
    }

    /**
     * Updates one Inputs from readings of a TalonSRX motor
     *
     * @param inputsToUpdate Inputs to update from reading.
     * @param motor Motor to read from.
     */
    protected void updateMotorInputs(Inputs inputsToUpdate, TalonSRX motor) {
        // Position: ticks converted to Rotations
        inputsToUpdate.position = Units.Rotations.of(motor.getSelectedSensorPosition() / ticksPerRotation);
        
        // Velocity: ticks per 100ms converted to Rotations per Second (100ms * 10 = 1s)
        double rps = (motor.getSelectedSensorVelocity() * 10.0) / ticksPerRotation;
        inputsToUpdate.velocity = Units.RotationsPerSecond.of(rps);
        
        inputsToUpdate.statorCurrent = Units.Amps.of(motor.getStatorCurrent());
        inputsToUpdate.supplyCurrent = Units.Amps.of(motor.getSupplyCurrent()); 
        inputsToUpdate.motorVoltage = Units.Volts.of(motor.getMotorOutputVoltage());
        inputsToUpdate.pidVoltage = Units.Volts.of(motor.getMotorOutputVoltage()); 
        inputsToUpdate.motorTemperature = Units.Celsius.of(motor.getTemperature());
        inputsToUpdate.acceleration = Units.RotationsPerSecondPerSecond.of(0.0); 
    }

    @Override
    public void setNeutralSetpoint() {
        main.neutralOutput();
    }

    @Override
    public void setCoastSetpoint() {
        main.neutralOutput();
    }

    @Override
    protected void setVoltageSetpoint(Voltage voltage) {
        modeSetter.setVoltage(main, voltage);
    }

    @Override
    protected void setDutyCycleSetpoint(Dimensionless percent) {
        modeSetter.setDutyCycle(main, percent);
    }

    @Override
    protected void setMotionMagicSetpoint(Angle mechanismPosition) {
        setMotionMagicSetpoint(mechanismPosition, 0);
    }

    @Override
    protected void setMotionMagicSetpoint(Angle mechanismPosition, int slot) {
        modeSetter.setMotionMagic(main, mechanismPosition, slot, ticksPerRotation);
    }

    @Override
    protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
        setVelocitySetpoint(mechanismVelocity, 1);
    }

    @Override
    protected void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot) {
        modeSetter.setVelocity(main, mechanismVelocity, slot, ticksPerRotation);
    }

    @Override
    protected void setPositionSetpoint(Angle mechanismPosition) {
        setPositionSetpoint(mechanismPosition, 2);
    }

    @Override
    protected void setPositionSetpoint(Angle mechanismPosition, int slot) {
        modeSetter.setPosition(main, mechanismPosition, slot, ticksPerRotation);
    }

    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        threadPoolExecutor.submit(() -> {
            main.setSelectedSensorPosition(mechanismPosition.in(Units.Rotations) * ticksPerRotation);
        });
    }

    @Override
    public void zeroSensors() {
        setCurrentPosition(Units.Rotations.of(0.0));
    }

    private void setNeutralMode(TalonSRX talon, NeutralMode neutralMode) {
        SmartDashboard.putNumber("TALON SRX IDLE MODE SET!!", Timer.getFPGATimestamp());
        threadPoolExecutor.submit(() -> {
            talon.setNeutralMode(neutralMode);
        });
    }

    @Override
    public void setNeutralBrake(boolean wantsBrake) {
        var neutralMode = wantsBrake ? NeutralMode.Brake : NeutralMode.Coast;
        setNeutralMode(main, neutralMode);
        for (TalonSRX talon : followers) {
            setNeutralMode(talon, neutralMode);
        }
    }

    @Override
    public void useSoftLimits(boolean enable) {
        UnaryOperator<TalonSRXConfiguration> configChanger = (config) -> {
            config.forwardSoftLimitEnable = enable;
            config.reverseSoftLimitEnable = enable;
            return config;
        };

        changeMainConfig(configChanger);
    }

    @Override
    public TalonSRXConfiguration getMotorIOConfig() {
        return config;
    }

    @Override
    public void disabledPeriodic() {}

    public void setMainConfig(TalonSRXConfiguration configuration) {
        config = configuration;
        applyConfig(main, config);
    }

    public void changeMainConfig(UnaryOperator<TalonSRXConfiguration> configChanger) {
        setMainConfig(configChanger.apply(config));
    }

    public void setFollowerConfig(TalonSRXConfiguration configuration) {
        followerConfig = configuration;
        for (TalonSRX talon : followers) {
            applyConfig(talon, followerConfig);
        }
    }

    public void changeFollowerConfig(UnaryOperator<TalonSRXConfiguration> configChanger) {
        setFollowerConfig(configChanger.apply(followerConfig));
    }

    /**
     * Creates a MotorIOTalonSRX from a provided configuration.
     *
     * @param config Configuration to create MotorIOTalonSRX from.
     */
    public MotorIOTalonSRX(MotorIOTalonSRXConfig config) {
        super(config.unit, config.time, config.followerIDs.length);
        this.modeSetter = config.modeSetter;
        this.ticksPerRotation = config.ticksPerRotation;
        
        main = new TalonSRX(config.mainID);
        setMainConfig(config.mainConfig);

        followers = new TalonSRX[config.followerIDs.length];
        for (int i = 0; i < config.followerIDs.length; i++) {
            followers[i] = new TalonSRX(config.followerIDs[i]);
            followers[i].follow(main);

            // Evaluates alignment rules
            boolean isOpposed = (config.followerAlignment.length > i) 
                && (config.followerAlignment[i] == MotorAlignmentValue.Opposed);

            followers[i].setInverted(isOpposed ? InvertType.OpposeMaster : InvertType.FollowMaster);
        }

        setFollowerConfig(config.followerConfig);
    }

    /**
     * Configuration for a MotorIOTalonSRX. Motion Magic control is on slot 0, velocity on slot 1, and position PID on slot 2.
     */
    public static class MotorIOTalonSRXConfig {
        public AngleUnit unit = Units.Rotations;
        public TimeUnit time = Units.Seconds;
        public int mainID = -1;
        public double ticksPerRotation = 4096.0; // Default CPR for CTRE Mag Encoder
        public TalonSRXConfiguration mainConfig = new TalonSRXConfiguration();
        public int[] followerIDs = new int[0];
        public MotorAlignmentValue[] followerAlignment = new MotorAlignmentValue[0];
        public TalonSRXConfiguration followerConfig = new TalonSRXConfiguration();
        public ControlModeSetter modeSetter = new ControlModeSetter();
    }

    public static class ControlModeSetter {
        public void setVoltage(TalonSRX talon, Voltage voltage) {
            // Converts voltage setpoint relative to current bus voltage
            double busVoltage = talon.getBusVoltage();
            if (busVoltage > 0.0) {
                talon.set(ControlMode.PercentOutput, voltage.in(Units.Volts) / busVoltage);
            }
        }

        public void setDutyCycle(TalonSRX talon, Dimensionless percent) {
            talon.set(ControlMode.PercentOutput, percent.in(Units.Percent) / 100.0); 
        }

        public void setMotionMagic(TalonSRX talon, Angle mechanismPosition, int slot, double ticksPerRotation) {
            talon.selectProfileSlot(slot, 0);
            double targetTicks = mechanismPosition.in(Units.Rotations) * ticksPerRotation;
            talon.set(ControlMode.MotionMagic, targetTicks);
        }

        public void setVelocity(TalonSRX talon, AngularVelocity mechanismVelocity, int slot, double ticksPerRotation) {
            talon.selectProfileSlot(slot, 0);
            // Converts RPS to ticks per 100ms
            double rps = mechanismVelocity.in(AngularVelocityUnit.combine(Rotation, Units.Second));
            double targetTicksPer100ms = (rps * ticksPerRotation) / 10.0;
            talon.set(ControlMode.Velocity, targetTicksPer100ms);
        }

        public void setPosition(TalonSRX talon, Angle mechanismPosition, int slot, double ticksPerRotation) {
            talon.selectProfileSlot(slot, 0);
            double targetTicks = mechanismPosition.in(Units.Rotations) * ticksPerRotation;
            talon.set(ControlMode.Position, targetTicks);
        }
    }

    @Override
    public boolean getConfigFailed() {
        return configFailed;
    }

    public TalonSRX getMain() {
        return main;
    }
}