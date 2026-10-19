package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.sim.FuelSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Shots straight from the shooting table must score in the fuel sim, or the two models disagree. */
class RobotFuelSimTest {
  // Closer than this, the robot would be inside the hub
  private static final double MIN_DISTANCE = 1.3;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  @Test
  void tableShotsScore() throws IOException {
    List<String> misses = new ArrayList<>();
    int shots = 0;
    List<String> lines = Files.readAllLines(Path.of("src/main/deploy/hubSolutions.csv"));
    List<String> header = List.of(lines.get(0).split(","));
    for (String line : lines.subList(1, lines.size())) {
      String[] row = line.split(",");
      double distance = Double.parseDouble(row[header.indexOf("distance")]);
      double radialVelocity = Double.parseDouble(row[header.indexOf("radialVelocity")]);
      double ballSpeed = Double.parseDouble(row[header.indexOf("exitVelocity")]);
      double launchAngle = Double.parseDouble(row[header.indexOf("solutionAngle")]);
      // The wheel speed the robot would command, so the test also covers BALL_SPEED_FRACTION
      double wheelSpeed = ShooterConstants.mainWheelSpeedForBallSpeed(ballSpeed);
      if (distance < MIN_DISTANCE) continue;

      shots++;
      if (!scores(distance, radialVelocity, wheelSpeed, launchAngle)) {
        misses.add(line);
      }
    }

    assertTrue(shots > 50, "too few table rows were tested");
    assertEquals(List.of(), misses, "table shots that missed (distance,radialVelocity,speed,angle,...)");
  }

  /** Shoots at the blue hub from the given distance, with the robot moving toward it at radialVelocity. */
  private static boolean scores(double distance, double radialVelocity, double wheelSpeed, double launchAngle) {
    Translation2d hub = FuelSim.Hub.BLUE_HUB.getCenter();
    // Facing away from the hub, so the -x shooter points at it. The shooter sits behind the robot center.
    double shooterOffset = -ShooterConstants.LAUNCH_POSITION_ON_ROBOT.getX();
    Pose2d robot = new Pose2d(hub.getX() - distance - shooterOffset, hub.getY(), Rotation2d.k180deg);
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(radialVelocity, 0.0, 0.0);

    FuelSim fuelSim = new FuelSim("Test/Fuel");
    fuelSim.enableAirResistance();
    FuelSim.Hub.BLUE_HUB.resetScore();
    RobotFuelSim.Launch launch =
        RobotFuelSim.launch(robot, fieldSpeeds, wheelSpeed, Math.toRadians(launchAngle));
    fuelSim.spawnFuel(launch.position(), launch.velocity(), launch.spin());

    for (int i = 0; i < 200 && FuelSim.Hub.BLUE_HUB.getScore() == 0; i++) {
      fuelSim.stepSim();
    }
    return FuelSim.Hub.BLUE_HUB.getScore() == 1;
  }
}
