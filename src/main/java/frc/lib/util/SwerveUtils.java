package frc.lib.util;

public class SwerveUtils {

    /**
     * Snaps the robot's current yaw to the nearest 45-degree diagonal angle.
     * * @param currentYawDegrees The current gyro heading of the robot (can be continuous).
     * @return The closest angle out of 45, 135, 225, or 315.
     */
    public static double getClosestDiagonalAngle(double currentYawDegrees) {
        // 1. Constrain yaw to 0 to 360 degrees
        double normalizedYaw = (currentYawDegrees % 360 + 360) % 360;

        // 2. Shift by 45, round to nearest 90, and shift back
        double snapped = Math.round((normalizedYaw - 45.0) / 90.0) * 90.0 + 45.0;

        // 3. Final safety wrap (e.g., if it rounds up to 405, it becomes 45)
        return (snapped % 360 + 360) % 360;
    }
}