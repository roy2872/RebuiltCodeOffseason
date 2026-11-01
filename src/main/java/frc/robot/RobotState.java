package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotState.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.AllianceFlipping;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import lombok.Getter;
import lombok.Setter;

public class RobotState {

  public static final RobotState mInstance = new RobotState();

  private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.aprilTagLayout.getTagPose(i).map(Pose3d::toPose2d).orElse(Pose2d.kZero));
    }
  }
  @AutoLogOutput
  @Getter  private Pose2d odometryPose = Pose2d.kZero;
  @Getter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  // private double lastLimelightYawUpdate = 0.0;

  @AutoLogOutput(key = "RobotState/RobotVelocity")
  @Getter @Setter private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  @Getter @Setter private ChassisSpeeds robotSetpointVelocity = new ChassisSpeeds();
  // From the gyro, unlike robotVelocity which comes from the wheels and lies when pushed
  @AutoLogOutput(key = "RobotState/GyroYawRateRadPerSec")
  @Getter @Setter private double gyroYawRateRadPerSec = 0.0;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE);

  // Odometry (process) noise. Together with the vision std devs this sets how far each vision
  // measurement pulls the estimate. Starts from the 6328 template values; tune against logs.
  private static final TunableNumber odometryStdDevX = new TunableNumber("RobotState/OdometryStdDevX", 0.003);
  private static final TunableNumber odometryStdDevY = new TunableNumber("RobotState/OdometryStdDevY", 0.003);
  private static final TunableNumber odometryStdDevTheta = new TunableNumber("RobotState/OdometryStdDevTheta", 0.002);

  // The bump. While tilted, the wheels measure distance along the slope and the cameras look
  // through a tilted mount (MegaTag2 assumes the robot is flat), so vision is rejected. Right after
  // landing, odometry is trusted less so the first good vision poses fix what the bump did to it.
  private static final TunableNumber tiltThresholdDegrees =
      new TunableNumber("RobotState/Bump/TiltThresholdDegrees", 7.0);
  private static final TunableNumber bumpRecoverySeconds =
      new TunableNumber("RobotState/Bump/RecoverySeconds", 1.0);
  private static final TunableNumber bumpOdometryStdDevScale =
      new TunableNumber("RobotState/Bump/OdometryStdDevScale", 10.0);
  /** Extra time around a tilted stretch whose images are still rejected. */
  private static final double TILT_VISION_MARGIN_SECONDS = 0.05;

  @AutoLogOutput(key = "RobotState/Bump/TiltDegrees")
  @Getter private double tiltDegrees = 0.0;
  private double tiltedSince = Double.NEGATIVE_INFINITY;
  private double lastTiltedTimestamp = Double.NEGATIVE_INFINITY;
  private double lastTiltTimestamp = 0.0;
  private boolean wasTilted = false;

  private final SwerveDriveKinematics kinematics
   = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
  private SwerveModulePosition[] lastWheelPosition =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d gyroOffset = new Rotation2d();

  @AutoLogOutput
  public LimelightYawObservation bestLimelightYawObservation = new LimelightYawObservation(new Rotation2d(), 0.0);

  private Field2d field;
  private boolean shootToHubElseFerry;

  // Flywheel speed trim [m/s] from the dashboard. Logged, so replay uses the same values.
  private final LoggedNetworkNumber flywheelBias =
      new LoggedNetworkNumber("/SmartDashboard/FlywheelBias", 0.0);

  private RobotState() {
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
    // poseBuffer.addSample(Timer.getFPGATimestamp(), pose);
  }

  public Pose2d getEstimatedPose(double timestamp) {
    var sample = poseBuffer.getSample(timestamp);
    // If pose buffer is empty, return the current estimated pose
    if (sample.isEmpty()) {
      return estimatedPose;
    }
    return sample.get();
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPosition, observation.wheelPositions());
    // On a ramp the wheels roll along the slope; only the part along the floor moves the robot
    double floorScale = Math.cos(Math.toRadians(tiltDegrees));
    twist = new Twist2d(twist.dx * floorScale, twist.dy * floorScale, twist.dtheta);
    lastWheelPosition = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);

    observation.gyroAngle.ifPresent(
        angle -> {
          Rotation2d gyroRotation = angle.plus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), gyroRotation);
        });
    // add to pose buffer
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public void addVisionObservation(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (RobotController.getFPGATime() * 1e-6 - POSE_BUFFER_SIZE > timestampSeconds) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(timestampSeconds);
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }
    double translationScale = isRecoveringFromBump() ? bumpOdometryStdDevScale.getAsDouble() : 1.0;
    double[] q = {
      Math.pow(odometryStdDevX.getAsDouble() * translationScale, 2),
      Math.pow(odometryStdDevY.getAsDouble() * translationScale, 2),
      Math.pow(odometryStdDevTheta.getAsDouble(), 2)
    };
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      if (q[row] == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, q[row] / (q[row] + Math.sqrt(q[row] * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, visionRobotPoseMeters);
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }


  /**
   * The gyro's tilt from flat, in any direction. Call every loop before adding odometry.
   *
   * @param timestamp FPGA seconds
   */
  public void updateTilt(double tiltDegrees, double timestamp) {
    this.tiltDegrees = tiltDegrees;
    lastTiltTimestamp = timestamp;
    boolean tilted = isTilted();
    if (tilted && !wasTilted) {
      tiltedSince = timestamp;
    }
    if (tilted) {
      lastTiltedTimestamp = timestamp;
    }
    wasTilted = tilted;
    Logger.recordOutput("RobotState/Bump/Tilted", isTilted());
    Logger.recordOutput("RobotState/Bump/Recovering", isRecoveringFromBump());
  }

  /** On a bump (or otherwise tipped past the threshold). */
  public boolean isTilted() {
    return tiltDegrees >= tiltThresholdDegrees.getAsDouble();
  }

  /** Whether an image taken at {@code timestamp} was taken while the robot was on the last bump. */
  public boolean wasTiltedAt(double timestamp) {
    return timestamp >= tiltedSince - TILT_VISION_MARGIN_SECONDS
        && timestamp <= lastTiltedTimestamp + TILT_VISION_MARGIN_SECONDS;
  }

  /** Just off a bump: odometry is trusted less, so vision pulls the estimate back quickly. */
  public boolean isRecoveringFromBump() {
    return !isTilted() && lastTiltTimestamp - lastTiltedTimestamp < bumpRecoverySeconds.getAsDouble();
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  public ChassisSpeeds getFieldSetpointVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotSetpointVelocity, getRotation());
  }
  
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void periodic() {
    field.setRobotPose(estimatedPose);
    // if(bestLimelightYawObservation != null && (lastLimelightYawUpdate - RobotController.getFPGATime() * (1e-6)) < 5.0) {
    //   var vel = getFieldVelocity();
    //   if(/*(RobotController.getFPGATime() * (1e-6) - bestLimelightYawObservation.timestamp < 0.3)*/ true && 
    //     (Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond) < 1.0) && vel.omegaRadiansPerSecond < 1.0) {
    //       resetPose(new Pose2d(estimatedPose.getMeasureX(), estimatedPose.getMeasureY(), bestLimelightYawObservation.yaw));
    //     }
    // }
  }

  public Vector<N3> getVelocityRelativeToHub(Translation2d pose) {
    ChassisSpeeds currentVelocity = getFieldVelocity();
    Matrix<N2, N1> robotFieldVelocityVector =
        VecBuilder.fill(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
    Matrix<N2, N1> hubRadialVectorNorm = pose.minus(AllianceFlipping.apply(HUB_2D_COORDS.getTranslation())).toVector().unit();
    Matrix<N2, N1> hubTangentialVectorNorm = ROTATION_MATRIX_90.times(hubRadialVectorNorm);

    Double hubRadialVelocity = hubRadialVectorNorm.transpose().times(robotFieldVelocityVector).get(0, 0);
    Double hubTangentialVelocity = hubTangentialVectorNorm.transpose().times(robotFieldVelocityVector).get(0, 0);
    // get distance
    return VecBuilder.fill(hubRadialVelocity, hubTangentialVelocity, currentVelocity.omegaRadiansPerSecond);
  }
  public Vector<N3> getVelocityRelativeToHub() {
    return getVelocityRelativeToHub(getEstimatedPose().getTranslation());
  }

  public Rotation2d getAngleToHub(Translation2d pose) {
    return 
        AllianceFlipping.apply(HUB_2D_COORDS.getTranslation()).minus(pose).getAngle().plus(Rotation2d.k180deg);
  }

  public Rotation2d getAngleToHub() {
    return getAngleToHub(getEstimatedPose().getTranslation());
  }

    public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
    var oldOdometryPose = poseBuffer.getSample(timestamp);
    if (oldOdometryPose.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        this
            .getEstimatedPose()
            .transformBy(
                new Transform2d(
                    this.getOdometryPose(), oldOdometryPose.get())));
  }


  /**
   * 
   * @param shootType true for hub, false for ferry
   */
  public void setShootType(boolean shootType) {
    shootToHubElseFerry = shootType;
  }

    /** @param shootType true for hub, false for ferry */
  public boolean getShootType() {
    return shootToHubElseFerry;
  }

  /** @param manual true for a fixed hub flywheel speed, false for the shooting table */
  public void setManualShooting(boolean manual) {
    DashboardOverrides.MANUAL_SHOOTING.setNow(manual);
  }

  public boolean atAngle(Rotation2d targetRotation, Angle epsilon) {
    return Math.abs(getRotation().minus(targetRotation).getDegrees()) < epsilon.magnitude();
  }


  /**
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[m/s], Robot angle[deg]}
   */
  public Vector<N3> getShootInfo() {
    if (!isShootingAtHub()) {
      return getFetchingInfo();
    }
    return DashboardOverrides.VISION_FALLBACK_PRESETS.get() ? getShootCloseInfo() : getHubShootingInfo();
  }

  /** True when the current shot goes into the hub, false when ferrying (also forced by Force Ferry Mode). */
  public boolean isShootingAtHub() {
    return shootToHubElseFerry && !DashboardOverrides.FORCE_FERRY.get();
  }

  /**
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[rps], Robot angle[deg]}
   */
  public edu.wpi.first.math.Vector<N3> getHubShootingInfo() {
    Vector<N3> hubCenteredVelocity = getVelocityRelativeToHub();
    // TODO: could add here a more accurate depiction of distance
    Vector<N2> inputVector = VecBuilder.fill(
        getEstimatedPose().getTranslation().getDistance(AllianceFlipping.apply(HUB_2D_COORDS.getTranslation())) + SHOOTER_DISTANCE_FROM_CENTER, 
        hubCenteredVelocity.get(0, 0));
    var shootingData = getShootingData.apply(inputVector);
    double wheelSpeed = DashboardOverrides.MANUAL_SHOOTING.get()
        ? 10 + flywheelBias.get()
        : ShooterConstants.mainWheelSpeedForBallSpeed(shootingData.get(0, 0));
    return VecBuilder.fill(
        90 - shootingData.get(1, 0), // hood angle
        wheelSpeed / (Units.Inches.of(4).in(Units.Meters) * Math.PI), // flywheel velocity
        getAngleToHub().getDegrees() // robot angle
    );
  }

    /**
   * 
   * 
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[rps], Robot angle[deg]}
   */
  public Vector<N3> getShootOnTheMoveScoringInfo() {
    ChassisSpeeds robotRelativeVelocity = getRobotVelocity();
    Pose2d futurePose = getEstimatedPose().exp(
      new Twist2d(
      robotRelativeVelocity.vxMetersPerSecond * SHOOTING_DELAY, 
      robotRelativeVelocity.vyMetersPerSecond * SHOOTING_DELAY, 
      robotRelativeVelocity.omegaRadiansPerSecond * SHOOTING_DELAY));
    Vector<N3> hubCenteredVelocity = getVelocityRelativeToHub(futurePose.getTranslation());

    Vector<N2> inputVector = VecBuilder.fill(
        futurePose.getTranslation().getDistance(AllianceFlipping.apply(HUB_2D_COORDS.getTranslation())) + SHOOTER_DISTANCE_FROM_CENTER, 
        hubCenteredVelocity.get(0, 0));
    Vector<N3> shootingData = getShootingData.apply(inputVector);

    // double tangentialDistanceFromTarget = hubCenteredVelocity.get(1) * shootingData.get(2) * AIR_DRAG_APPROXIMATE_DECAY; simple drag model
    double tangentialDistanceFromTarget = (1.0 / TUNEABLE_DRAG_COEFFICIENT) * Math.log(1.0 + 
      TUNEABLE_DRAG_COEFFICIENT * Math.abs(hubCenteredVelocity.get(1)) * shootingData.get(2)) * Math.signum(hubCenteredVelocity.get(1));
    
    
    Translation2d effectiveEstimatedPose = futurePose.getTranslation().plus(
      new Translation2d(
        tangentialDistanceFromTarget, 0).rotateBy(
          futurePose.getTranslation().minus(AllianceFlipping.apply(HUB_2D_COORDS.getTranslation())).getAngle().plus(Rotation2d.kCCW_90deg)
        )
    );
    Logger.recordOutput("RobotState/EffectiveEstimatedPose", new Pose3d(
      new Translation3d(
        effectiveEstimatedPose.getX(),
        effectiveEstimatedPose.getY(),
        0.0),
      new Rotation3d(getEstimatedPose().getRotation())
    ));
    hubCenteredVelocity = getVelocityRelativeToHub(effectiveEstimatedPose);
    inputVector = VecBuilder.fill(
      effectiveEstimatedPose.getDistance(AllianceFlipping.apply(HUB_2D_COORDS.getTranslation())) + SHOOTER_DISTANCE_FROM_CENTER, 
      hubCenteredVelocity.get(0, 0));
    shootingData = getShootingData.apply(inputVector);
    Rotation2d angleToHub = getAngleToHub(effectiveEstimatedPose);

    // Table outputs are {ball speed [m/s], launch angle [deg], time of flight [s]}, same as getHubShootingInfo
    double wheelSpeed = ShooterConstants.mainWheelSpeedForBallSpeed(shootingData.get(0)) + flywheelBias.get();
    return VecBuilder.fill(
        90 - shootingData.get(1), // hood angle
        wheelSpeed / (Units.Inches.of(4).in(Units.Meters) * Math.PI), // flywheel velocity
        angleToHub.getDegrees() // robot angle
    );
  }

      /**
   * 
   * 
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[m/s], Robot angle[deg]}
   */
  public Vector<N3> getFetchingInfo() {
    double input = getFetchingDistance();
    Double fetchVel = fetchingTableData.get(input);
    double wheelSpeed = fetchVel == null ? 10.0 : ShooterConstants.mainWheelSpeedForBallSpeed(fetchVel);
    return VecBuilder.fill(
        90-FETCHING_ANGLE, // hood angle
        (wheelSpeed + flywheelBias.get()) 
        / (Units.Inches.of(4).in(Units.Meters) * Math.PI), // flywheel velocity
        AllianceFlipping.apply(Rotation2d.kZero).getDegrees() // robot angle
    );
  }

    /**
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[m/s], Robot angle[deg]}
   */
  public Vector<N3> getManualFetchingInfo() {
    return VecBuilder.fill(
      FETCHING_ANGLE,
      FETCHING_VELOCITY,
      AllianceFlipping.apply(Rotation2d.kZero).getDegrees()
    );
  }

  /**
   * The fixed close-shot preset, for when vision can't be trusted. SHOOT_CLOSE_VELOCITY is a ball speed
   * and SHOOT_CLOSE_ANGLE a launch angle, converted the same way as table shots.
   *
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[rps], Robot angle[deg]}
   */
  public Vector<N3> getShootCloseInfo() {
    double wheelSpeed = ShooterConstants.mainWheelSpeedForBallSpeed(SHOOT_CLOSE_VELOCITY) + flywheelBias.get();
    return VecBuilder.fill(
        90 - SHOOT_CLOSE_ANGLE, // hood angle
        wheelSpeed / (Units.Inches.of(4).in(Units.Meters) * Math.PI), // flywheel velocity
        getAngleToHub().getDegrees() // robot angle
    );
  }

  public double getFetchingDistance() {
    return Math.abs(getEstimatedPose().getX() - AllianceFlipping.applyX(FETCHING_TARGET_LINE));
  }

  public void addLimelightYawObservation(LimelightYawObservation observation) {
    bestLimelightYawObservation = observation;
  }

  public LimelightYawObservation getLimelightYawObservation() {
    return bestLimelightYawObservation;
  }
  
  public record LimelightYawObservation(
      Rotation2d yaw, double timestamp) {}

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
}
