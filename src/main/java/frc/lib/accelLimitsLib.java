package frc.lib;

import static frc.robot.Constants.CYCLE_TIME;
import static frc.robot.subsystems.drive.DriveConstants.*;

// Import WPILib standard units
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;

public class accelLimitsLib {

  // Below this speed, "current direction" is just sensor/floating point noise.
  // 1e-6 was far too small - any tiny residual velocity (e.g. right as the
  // robot crosses through zero during a direction reversal) produced an
  // essentially random unit vector, which then got used to "forward limit"
  // in a meaningless direction. 0.05 m/s is a much safer floor.
  private static final double MIN_VEL_FOR_DIRECTION = 0.05;

  public static ChassisSpeeds applyAccLimits(
      ChassisSpeeds wantedChassisVelocityRobotOriented,
      ChassisSpeeds currentChassisVelocityRobotOriented) {

    // --- WPILIB UNITS UNPACKING ---
    double cycleTimeSec = CYCLE_TIME.in(Seconds);

    // Defensive guard: dividing by a zero/negative cycle time produces
    // Infinity/NaN, which then poisons every downstream calculation and can
    // cause genuinely undefined behavior in swerve module setpoints (which
    // can look like "the robot won't stop" or moves unpredictably). This
    // should never trigger in practice, but it's cheap insurance.
    if (cycleTimeSec <= 1e-6) {
      return wantedChassisVelocityRobotOriented;
    }

    double maxSpeedMPS = MAX_SPEED.in(MetersPerSecond);
    double maxAccel = MAX_ACCELERATION;
    double maxFrontAccel = MAX_FRONT_ACCEL;
    double maxSideAccel = MAX_SIDE_ACCEL;
    double maxSkidAccel = MAX_SKID_ACCEL;
    double maxRotAccel = MAX_ROTATIONAL_ACCEL;

    // 1. Convert to 2D Vectors for translation
    Vector<N2> wantedVelocity =
        VecBuilder.fill(
            wantedChassisVelocityRobotOriented.vxMetersPerSecond,
            wantedChassisVelocityRobotOriented.vyMetersPerSecond);
    Vector<N2> currentVelocity =
        VecBuilder.fill(
            currentChassisVelocityRobotOriented.vxMetersPerSecond,
            currentChassisVelocityRobotOriented.vyMetersPerSecond);

    // 2. Calculate RAW wanted acceleration vector
    Vector<N2> wantedAcc = (wantedVelocity.minus(currentVelocity)).div(cycleTimeSec);

    double currentVelMag = currentVelocity.norm();
    // Only trust the "direction of travel" if we're actually moving meaningfully.
    Vector<N2> currentDir =
        currentVelMag > MIN_VEL_FOR_DIRECTION ? currentVelocity.div(currentVelMag) : null;

    // ==========================================================
    // LIMIT 1: TILT LIMIT
    // Limit Acceleration in robot directions to prevent tilting.
    // This is a hard structural limit, so it goes first - everything
    // downstream is only ever allowed to shrink further from here.
    // ==========================================================
    double accelX = wantedAcc.get(0);
    double accelY = wantedAcc.get(1);

    double limitedAccelX = Math.copySign(Math.min(Math.abs(accelX), maxFrontAccel), accelX);
    double limitedAccelY = Math.copySign(Math.min(Math.abs(accelY), maxSideAccel), accelY);

    wantedAcc = VecBuilder.fill(limitedAccelX, limitedAccelY);

    // ==========================================================
    // LIMIT 2: SKID LIMIT
    // Limit the max Acceleration for the robot to not skid.
    // ==========================================================
    double accMag = wantedAcc.norm();

    if (accMag > maxSkidAccel) {
      wantedAcc = wantedAcc.div(accMag).times(maxSkidAccel);
    }

    // ==========================================================
    // LIMIT 3: FORWARD LIMIT
    // Motors provide more torque the slower they spin, so acceleration
    // along the current direction of travel is capped based on current speed.
    //
    // IMPORTANT: this now runs LAST (after tilt + skid), not first.
    //
    // Why: tilt/skid clamp X and Y *independently*. If the raw request had
    // a forward component and a "canceling" lateral component, clamping
    // those two axes by different amounts can shrink the canceling part
    // more than the forward part - which *increases* the net acceleration
    // along the direction of travel. Running forward-limit first let that
    // slip through uncaught, which is what was causing the robot to end up
    // faster (and off-direction) than commanded. Running it last guarantees
    // the final output never exceeds the motor torque limit, no matter what
    // the earlier steps did to the vector.
    // ==========================================================
    if (currentDir != null && maxSpeedMPS > 1e-6) {
      double wantedAccFwd = wantedAcc.dot(currentDir);

      if (wantedAccFwd > 0) {
        double maxFwdAcc = maxAccel * (1.0 - (currentVelMag / maxSpeedMPS));
        maxFwdAcc = Math.max(0.0, maxFwdAcc);

        if (wantedAccFwd > maxFwdAcc) {
          Vector<N2> excessAccFwd = currentDir.times(wantedAccFwd - maxFwdAcc);
          wantedAcc = wantedAcc.minus(excessAccFwd);
        }
      }
    }

    // ==========================================================
    // SAFETY NET: re-apply the skid (magnitude) clamp one more time.
    // The forward-limit correction above can, in rare geometries, nudge the
    // vector slightly back outside the skid circle. This is cheap and a
    // no-op unless that happens, but it guarantees the final acceleration
    // magnitude is always bounded by maxSkidAccel - no combination of the
    // limits above can ever sneak a larger-than-configured acceleration
    // through to the output.
    // ==========================================================
    accMag = wantedAcc.norm();
    if (accMag > maxSkidAccel) {
      wantedAcc = wantedAcc.div(accMag).times(maxSkidAccel);
    }

    // ==========================================================
    // LIMIT 4: ROTATIONAL LIMIT
    // Limit angular acceleration to prevent spinning out or tipping
    // ==========================================================
    double currentOmega = currentChassisVelocityRobotOriented.omegaRadiansPerSecond;
    double wantedOmega = wantedChassisVelocityRobotOriented.omegaRadiansPerSecond;

    double maxDeltaOmega = maxRotAccel * cycleTimeSec;

    double limitedOmega =
        Math.max(
            currentOmega - maxDeltaOmega, Math.min(currentOmega + maxDeltaOmega, wantedOmega));

    // ==========================================================
    // FINAL: Apply the fully limited acceleration to our current velocity
    // ==========================================================
    Vector<N2> finalLimitedVelocity = currentVelocity.plus(wantedAcc.times(cycleTimeSec));

    return new ChassisSpeeds(
        finalLimitedVelocity.get(0), finalLimitedVelocity.get(1), limitedOmega);
  }
}