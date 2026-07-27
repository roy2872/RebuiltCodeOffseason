package frc.lib.io;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.signals.MotorAlignmentValue; // Import the target alignment enum type

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
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

/**
 * Class used to control a main SparkMax and any number of followers for a real mechanism.
 */
public class MotorIOSparkMax extends MotorIO<SparkMaxConfig> {
    protected final SparkMax main;
    protected final SparkMax[] followers;
    protected SparkMaxConfig config;
    protected SparkMaxConfig followerConfig;
    private final ControlModeSetter modeSetter;
    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor =
            new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);
    private boolean configFailed = false;

    public void applyConfig(SparkMax spark, SparkMaxConfig config) {
        threadPoolExecutor.submit(() -> {
            for (int i = 0; i < 5; i++) {
                var result = spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                if (result == com.revrobotics.REVLibError.kOk) {
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
     * Updates one Inputs from readings of a SparkMax motor
     *
     * @param inputsToUpdate Inputs to update from reading.
     * @param motor Motor to read from.
     */
    protected void updateMotorInputs(Inputs inputsToUpdate, SparkMax motor) {
        inputsToUpdate.position = Units.Rotations.of(motor.getEncoder().getPosition());
        inputsToUpdate.velocity = Units.RotationsPerSecond.of(motor.getEncoder().getVelocity() / 60.0); // RPM to RPS
        inputsToUpdate.statorCurrent = Units.Amps.of(motor.getOutputCurrent());
        inputsToUpdate.supplyCurrent = Units.Amps.of(motor.getOutputCurrent()); 
        inputsToUpdate.motorVoltage = Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
        inputsToUpdate.pidVoltage = Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput()); 
        inputsToUpdate.motorTemperature = Units.Celsius.of(motor.getMotorTemperature());
        inputsToUpdate.acceleration = Units.RotationsPerSecondPerSecond.of(0.0); 
    }

    @Override
    public void setNeutralSetpoint() {
        main.stopMotor();
    }

    @Override
    public void setCoastSetpoint() {
        main.stopMotor();
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
        modeSetter.setSmartMotion(main, mechanismPosition, slot);
    }

    @Override
    protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
        setVelocitySetpoint(mechanismVelocity, 1);
    }

    @Override
    protected void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot) {
        modeSetter.setVelocity(main, mechanismVelocity, slot);
    }

    @Override
    protected void setPositionSetpoint(Angle mechanismPosition) {
        setPositionSetpoint(mechanismPosition, 2);
    }

    @Override
    protected void setPositionSetpoint(Angle mechanismPosition, int slot) {
        modeSetter.setPosition(main, mechanismPosition, slot);
    }

    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        threadPoolExecutor.submit(() -> {
            main.getEncoder().setPosition(mechanismPosition.in(Units.Rotations));
        });
    }

    @Override
    public void zeroSensors() {
        setCurrentPosition(Units.Rotations.of(0.0));
    }

    private void setNeutralMode(SparkMax spark, com.revrobotics.spark.config.SparkBaseConfig.IdleMode idleMode) {
        SmartDashboard.putNumber("SPARK MAX IDLE MODE SET!!", Timer.getFPGATimestamp());
        threadPoolExecutor.submit(() -> {
            SparkMaxConfig tempConfig = new SparkMaxConfig();
            tempConfig.idleMode(idleMode);
            spark.configure(tempConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    @Override
    public void setNeutralBrake(boolean wantsBrake) {
        var idleMode = wantsBrake ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
        config.idleMode(idleMode);
        setNeutralMode(main, idleMode);
        for (SparkMax spark : followers) {
            setNeutralMode(spark, idleMode);
        }
    }

    @Override
    public void useSoftLimits(boolean enable) {
        UnaryOperator<SparkMaxConfig> configChanger = (config) -> {
            config.softLimit.forwardSoftLimitEnabled(enable);
            config.softLimit.reverseSoftLimitEnabled(enable);
            return config;
        };

        changeMainConfig(configChanger);
    }

    @Override
    public SparkMaxConfig getMotorIOConfig() {
        return config;
    }

    @Override
    public void disabledPeriodic() {}

    public void setMainConfig(SparkMaxConfig configuration) {
        config = configuration;
        applyConfig(main, config);
    }

    public void changeMainConfig(UnaryOperator<SparkMaxConfig> configChanger) {
        setMainConfig(configChanger.apply(config));
    }

    public void setFollowerConfig(SparkMaxConfig configuration) {
        followerConfig = configuration;
        for (SparkMax spark : followers) {
            applyConfig(spark, followerConfig);
        }
    }

    public void changeFollowerConfig(UnaryOperator<SparkMaxConfig> configChanger) {
        setFollowerConfig(configChanger.apply(followerConfig));
    }

    /**
     * Creates a MotorIOSparkMax from a provided configuration.
     *
     * @param config Configuration to create MotorIOSparkMax from.
     */
    public MotorIOSparkMax(MotorIOSparkMaxConfig config) {
        super(config.unit, config.time, config.followerIDs.length);
        modeSetter = config.modeSetter;
        main = new SparkMax(config.mainID, MotorType.kBrushless);
        setMainConfig(config.mainConfig);

        followers = new SparkMax[config.followerIDs.length];
        for (int i = 0; i < config.followerIDs.length; i++) {
            followers[i] = new SparkMax(config.followerIDs[i], MotorType.kBrushless);
            
            // Evaluates alignment rules: Opposed sets invert parameter to true, Aligned sets it to false
            boolean isInverted = (config.followerAlignment.length > i) 
                && (config.followerAlignment[i] == MotorAlignmentValue.Opposed);

            SparkMaxConfig fConfig = new SparkMaxConfig();
            fConfig.follow(config.mainID, isInverted);
            followers[i].configure(fConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        setFollowerConfig(config.followerConfig);
    }

    private static ClosedLoopSlot getClosedLoopSlot(int slot) {
        switch (slot) {
            case 0:
                return ClosedLoopSlot.kSlot0;
            case 1:
                return ClosedLoopSlot.kSlot1;
            case 2:
                return ClosedLoopSlot.kSlot2;
            case 3:
                return ClosedLoopSlot.kSlot3;
            default:
                throw new IllegalArgumentException("Invalid slot number: " + slot);
        }
    }

    /**
     * Configuration for a MotorIOSparkMax. Smart Motion control is on slot 0, velocity on slot 1, and position PID on slot 2.
     */
    public static class MotorIOSparkMaxConfig {
        public AngleUnit unit = Units.Rotations;
        public TimeUnit time = Units.Seconds;
        public int mainID = -1;
        public SparkMaxConfig mainConfig = new SparkMaxConfig();
        public int[] followerIDs = new int[0];
        public MotorAlignmentValue[] followerAlignment = new MotorAlignmentValue[0]; // Mapped replacement array
        public SparkMaxConfig followerConfig = new SparkMaxConfig();
        public ControlModeSetter modeSetter = new ControlModeSetter();
    }

    public static class ControlModeSetter {
        public void setVoltage(SparkMax spark, Voltage voltage) {
            spark.setVoltage(voltage.in(Units.Volts));
        }

        public void setDutyCycle(SparkMax spark, Dimensionless percent) {
            spark.set(percent.in(Units.Percent) / 100.0); 
        }

        public void setSmartMotion(SparkMax spark, Angle mechanismPosition, int slot) {
            spark.getClosedLoopController().setSetpoint(
                    mechanismPosition.in(Units.Rotations), 
                    ControlType.kMAXMotionPositionControl, 
                    ClosedLoopSlot.kSlot0
            );
        }

        public void setVelocity(SparkMax spark, AngularVelocity mechanismVelocity, int slot) {
            spark.getClosedLoopController().setSetpoint(
                    mechanismVelocity.in(AngularVelocityUnit.combine(Rotation, Minute)), 
                    ControlType.kVelocity, 
                    getClosedLoopSlot(slot)
            );
        }

        public void setPosition(SparkMax spark, Angle mechanismPosition, int slot) {
            spark.getClosedLoopController().setSetpoint(
                    mechanismPosition.in(Units.Rotations), 
                    ControlType.kPosition, 
                    getClosedLoopSlot(slot)
            );
        }
    }

    @Override
    public boolean getConfigFailed() {
        return configFailed;
    }

    public SparkMax getMain() {
        return main;
    }
}