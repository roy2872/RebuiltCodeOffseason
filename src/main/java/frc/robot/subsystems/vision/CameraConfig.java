package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Everything robot-specific about one vision camera, independent of its vendor. Build one per
 * camera in {@link VisionConstants}; vendor-only settings go in the IO's own options.
 *
 * @param name camera name on NetworkTables (Limelight hostname or PhotonVision camera name)
 * @param model camera optics, see {@link CameraModel}
 * @param robotToCamera camera pose in the robot frame, WPILib convention (x forward, y left, z
 *     up). Must match the mounting configured in the camera's own UI.
 * @param roiTuning crop / downscale tuning, see {@link VisionRoiController.Tuning}
 */
public record CameraConfig(
    String name,
    CameraModel model,
    Transform3d robotToCamera,
    VisionRoiController.Tuning roiTuning) {}
