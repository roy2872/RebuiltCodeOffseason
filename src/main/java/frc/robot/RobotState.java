package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotState.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.AllianceFlipping;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.dyn4j.geometry.Matrix22;
import org.dyn4j.geometry.Rotation;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  // private final double txTyObservationStaleSecs = 0.2; // seconds
  // private final double minDistanceTagPoseBlend = 0.4; // meters
  // private final double maxDistanceTagPoseBlend = 0.9; // meters

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.aprilTagLayout.getTagPose(i).map(Pose3d::toPose2d).orElse(Pose2d.kZero));
    }
  }

  @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  private double lastLimelightYawUpdate = 0.0;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE);

  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPosition =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d gyroOffset = new Rotation2d();

  private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
  // private Set<fuelPoseRecord> fuelPoses = new HashSet<>();

  @AutoLogOutput(key = "RobotState/RobotVelocity")
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @AutoLogOutput
  public LimelightYawObservation bestLimelightYawObservation = new LimelightYawObservation(new Rotation2d(), -500.0);

  // elevator extension

  // intake extension

  // arm position

  private RobotState() {
    for (int i = 0; i < 3; i++) {
      qStdDevs.set(i, 0, Math.pow(i, i)); // change this!
    }

    kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());

    // maybe add here vision standart devs idk wtf
  }

  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
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
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
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

  public void addTxTyObservation(
      TxTyObservation observation) { // TODO: change this to our calculation if doesn't work well
    // Skip if current data for tag is newer
    if (txTyPoses.containsKey(observation.tagId())
        && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) {
      return;
    }

    // Get rotation at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }
    Rotation2d robotRotation =
        estimatedPose.transformBy(new Transform2d(odometryPose, sample.get())).getRotation();

    // Average tx's and ty's
    double tx = 0.0;
    double ty = 0.0;
    for (int i = 0; i < 4; i++) {
      tx += observation.tx()[i];
      ty += observation.ty()[i];
    }
    tx /= 4.0;
    ty /= 4.0;

    Pose3d cameraPose = new Pose3d().plus(VisionConstants.robotToCameras[observation.camera()]);

    // Use 3D distance and tag angles to find robot pose
    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
            .transformBy(
                new Transform3d(new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();

    Rotation2d camToTagRotation =
        robotRotation.plus(
            cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
    var tagPose2d = tagPoses2d.get(observation.tagId());
    if (tagPose2d == null) return;
    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0.0, new Rotation2d()))
            .getTranslation();
    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation, robotRotation.plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
    // Use gyro angle at time for robot rotation
    robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

    // Add transform to current odometry based pose for latency correction
    txTyPoses.put(
        observation.tagId(),
        new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  // public Optional<Translation2d> getBestFuel() {

  //   double minScore = Double.POSITIVE_INFINITY;
  //   Translation2d bestCoral = Translation2d.kZero;
  //   boolean noFuelFlag = true;

    // for (var coralPose : fuelPoses) {
    //   Translation2d difference = coralPose.translation.minus(estimatedPose.getTranslation());
    //   double score =
    //       Math.abs(difference.getAngle().getDegrees() - estimatedPose.getRotation().getDegrees())
    //               * 1
    //           + difference.getNorm()
    //               * 40; // can change the weights if selection method doesnt work well
    //   if (score < minScore) {
    //     minScore = score;
    //     noFuelFlag = false;
    //   }
    // }
    // if (!noFuelFlag) {
    //   return Optional.of(bestCoral);
    // } else {
    //   return Optional.empty();
    // }
  // }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void periodic() {
    if(bestLimelightYawObservation != null && (lastLimelightYawUpdate - RobotController.getFPGATime() * (1e-6)) < 5.0) {
      var vel = getFieldVelocity();
      if(/*(RobotController.getFPGATime() * (1e-6) - bestLimelightYawObservation.timestamp < 0.3)*/ true && 
        (Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond) < 1.0) && vel.omegaRadiansPerSecond < 1.0) {
          resetPose(new Pose2d(estimatedPose.getMeasureX(), estimatedPose.getMeasureY(), bestLimelightYawObservation.yaw));
        }
    }
    // fuelPoses.removeIf(
    //     (fuelPose) ->
    //         Timer.getFPGATimestamp() - fuelPose.timestamp()
    //             > 1); // can change this to something different

    // // Log coral posses
    // Logger.recordOutput(
    //     "RobotState/FuelPoses",
    //     fuelPoses.stream()
    //         .map(
    //             fuelPose ->
    //                 new Pose3d(
    //                     new Translation3d(
    //                         fuelPose.translation().getX(),
    //                         fuelPose.translation().getY(),
    //                         FieldConstants.fuelDiameter / 2.0),
    //                     new Rotation3d()))
    //         .toArray(Pose3d[]::new));
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
        AllianceFlipping.apply(HUB_2D_COORDS.getTranslation()).minus(pose).getAngle();
  }

  public Rotation2d getAngleToHub() {
    return getAngleToHub(getEstimatedPose().getTranslation());
  }

  /**
   * 
   * 
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[m/s], Robot angle[deg]}
   */
  public edu.wpi.first.math.Vector<N3> getShootingInfo() {
    Vector<N3> hubCenteredVelocity = getVelocityRelativeToHub();
    // TODO: could add here a more accurate depiction of distance
    Vector<N2> inputVector = VecBuilder.fill(
        getEstimatedPose().getTranslation().getDistance(AllianceFlipping.apply(HUB_2D_COORDS.getTranslation())) + SHOOTER_DISTANCE_FROM_CENTER, 
        hubCenteredVelocity.get(0, 0));
    var shootingData = getShootingData.apply(inputVector);
    return VecBuilder.fill(
        shootingData.get(0, 0), // hood angle
        shootingData.get(1, 0), // flywheel velocity
        getAngleToHub().plus(Rotation2d.k180deg).getDegrees() // robot angle
    );
  }

    /**
   * 
   * 
   * @return a vector that consists of {Hood angle[deg], Flywheel velocity[m/s], Robot angle[deg]}
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

    return VecBuilder.fill(
        shootingData.get(0), // hood angle
        shootingData.get(1), // flywheel velocity
        angleToHub.getDegrees() // robot angle
    );
  }

  public void addLimelightYawObservation(LimelightYawObservation observation) {
    bestLimelightYawObservation = observation;
  }

  public LimelightYawObservation getLimelightYawObservation() {
    return bestLimelightYawObservation;
  }
  
  public record LimelightYawObservation(
      Rotation2d yaw, double timestamp) {}

  public record TxTyObservation(
      int tagId, int camera, double[] tx, double[] ty, double distance, double timestamp) {}

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  // public record fuelPoseRecord(Translation2d translation, double timestamp) {}

  public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}

  public record ScoringInfo(int reefFace, boolean backside, Pose2d alignPose, Pose2d scorePose) {}

  public record ReefPoseEstimate(Pose2d pose, double blend) {}
}
