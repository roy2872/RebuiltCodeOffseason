package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    if (Constants.simMode == Constants.Mode.REPLAY) {
        System.out.println("Starting Replay mode!");
    }
    RobotBase.startRobot(Robot::new);
  }
}
