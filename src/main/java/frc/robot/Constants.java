// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.AllianceFlipping;
import frc.lib.util.NestedInterpolatingTreeMap;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double CYCLE_TIME = 0.02;
  public static final double FIELD_LENGTH = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getFieldLength();
  public static final double FIELD_WIDTH = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getFieldWidth();
  public static final double POSE_BUFFER_SIZE = 1.2; // seconds
  public static final double SHOOT_CLOSE_VELOCITY = 8.5; // m/s
  public static final double SHOOT_CLOSE_ANGLE = 79.0; // deg

  public static final double FETCH_VELOCITY = 12.0; // m/s
  public static final double FETCH_ANGLE = 70.0; // deg

  public static final class FieldConstants {

    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final double fieldLength = aprilTagLayout.getFieldLength();

    public static final double fieldWidth = aprilTagLayout.getFieldWidth();
    ;
    public static final double startingLineX = // TODO: update
        Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double fuelDiameter = Units.inchesToMeters(5.91);
    public static final int aprilTagCount = 32;
  }

  public static final class RobotState {
    // public static final InterpolatingTreeMap<Translation2d, Translation3d> a = new InterpolatingTreeMap<>(null, null); 
        // Create the 2D table
        public static NestedInterpolatingTreeMap<Double, Translation3d> shooterTableData =
            new NestedInterpolatingTreeMap<>(
                (a, b, query) -> (query - a) / (b - a),      // inverse interpolator for keys
                Translation3d::interpolate                   // value interpolator
            );

    public static final Pose2d HUB_2D_COORDS = new Pose2d(4.62 , 4.035, new Rotation2d());
    public static final Pose3d UPPER_HUB_3D_COORDS = new Pose3d(HUB_2D_COORDS.getX(), HUB_2D_COORDS.getY(), 1.825, Rotation3d.kZero);
    public static final Pose3d LOWER_HUB_3D_COORDS = new Pose3d(HUB_2D_COORDS.getX(), HUB_2D_COORDS.getY(), 1.43, Rotation3d.kZero);
      /**
       * @param input A Vector<N2> representing the distance[m] and robots radial velocity[m/s]
       * @return A Vector<N3> representing the optimal shooting angle[deg], optimal shooting speed[m/s], and time of flight[s]
       */
    public static final Function<Vector<N2>, Vector<N3>> getShootingData = new Function<Vector<N2>, Vector<N3>>() {
      @Override
      public Vector<N3> apply(Vector<N2> input) {
        Translation3d tableOutput = shooterTableData.get(input.get(0), input.get(1));
        Logger.recordOutput("RobotState/ShootingTableOutput", tableOutput);
        return VecBuilder.fill(tableOutput.getX(), tableOutput.getY(), tableOutput.getZ());
      }
    };

    public static final double SHOOTER_DISTANCE_FROM_CENTER = 0.2;
    public static final Matrix<N2, N2> ROTATION_MATRIX_90 = new Matrix<>(N2.instance, N2.instance);
  static {
    ROTATION_MATRIX_90.set(0, 0, 0.0);
    ROTATION_MATRIX_90.set(0, 1, -1.0);
    ROTATION_MATRIX_90.set(1, 0, 1.0);
    ROTATION_MATRIX_90.set(1, 1, 0.0);
    }

    public static final double SHOOTING_DELAY = 0.04; // seconds
    public static final double AIR_DRAG_APPROXIMATE_DECAY = 0.9;
    public static final double TUNEABLE_DRAG_COEFFICIENT = 0.15; // [1/m]
  }
}
