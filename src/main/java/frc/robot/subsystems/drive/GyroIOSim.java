package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.lib.util.PhoenixUtil;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;
  // maple-sim is flat, so tilt comes from a model of the field (pitch, roll in degrees)
  private final Supplier<double[]> tiltDegrees;

  public GyroIOSim(GyroSimulation gyroSimulation, Supplier<double[]> tiltDegrees) {
    this.gyroSimulation = gyroSimulation;
    this.tiltDegrees = tiltDegrees;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
    double[] tilt = tiltDegrees.get();
    inputs.pitchDegrees = tilt[0];
    inputs.rollDegrees = tilt[1];

    // maple-sim's physics sub-ticks stand in for the high-rate odometry samples
    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
