package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A place the robot can start auto.
 *
 * @param bluePose the starting pose on the blue alliance; flipped automatically for red
 */
public record AutoStartPosition(String name, Pose2d bluePose) {}
