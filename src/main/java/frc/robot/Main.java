package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // Do not access Constants before startRobot(). Loading Constants constructs the
    // subsystem singletons, some of which create/start timers that require the FPGA
    // clock. startRobot() initializes the HAL and FPGA clock before it constructs Robot.
    RobotBase.startRobot(Robot::new);
  }
}
