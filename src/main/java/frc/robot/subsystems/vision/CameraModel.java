package frc.robot.subsystems.vision;

/**
 * Optics of any camera: field of view and the capture resolution the pipeline runs at. Build it
 * from whatever the vendor gives you (horizontal/vertical FOV, diagonal FOV, or calibrated focal
 * lengths). Datasheet presets are provided for common FRC cameras.
 */
public record CameraModel(
    String displayName,
    double horizontalFovDeg,
    double verticalFovDeg,
    int widthPixels,
    int heightPixels) {

  // Limelight datasheet values for the native capture mode. The exact intrinsics of a specific
  // unit are shown in the Limelight web UI calibration tab.
  public static final CameraModel LIMELIGHT_3 =
      new CameraModel("Limelight 3", 62.5, 48.9, 1280, 960);
  public static final CameraModel LIMELIGHT_3G =
      new CameraModel("Limelight 3G", 82.0, 56.2, 1280, 800);
  public static final CameraModel LIMELIGHT_4 =
      new CameraModel("Limelight 4", 82.0, 56.2, 1280, 800);

  /** From a diagonal FOV, as PhotonVision and most USB camera datasheets report it. */
  public static CameraModel fromDiagonalFov(
      String displayName, int widthPixels, int heightPixels, double diagonalFovDeg) {
    double tanHalfDiagonal = Math.tan(Math.toRadians(diagonalFovDeg) / 2.0);
    double diagonalPixels = Math.hypot(widthPixels, heightPixels);
    double tanHalfH = tanHalfDiagonal * widthPixels / diagonalPixels;
    double tanHalfV = tanHalfDiagonal * heightPixels / diagonalPixels;
    return new CameraModel(
        displayName,
        Math.toDegrees(2.0 * Math.atan(tanHalfH)),
        Math.toDegrees(2.0 * Math.atan(tanHalfV)),
        widthPixels,
        heightPixels);
  }

  /** From calibrated focal lengths in pixels (fx, fy of the camera matrix). */
  public static CameraModel fromIntrinsics(
      String displayName, int widthPixels, int heightPixels, double fxPixels, double fyPixels) {
    return new CameraModel(
        displayName,
        Math.toDegrees(2.0 * Math.atan(widthPixels / (2.0 * fxPixels))),
        Math.toDegrees(2.0 * Math.atan(heightPixels / (2.0 * fyPixels))),
        widthPixels,
        heightPixels);
  }

  /** Same optics at a different capture resolution (e.g. a binned 640x400 mode). */
  public CameraModel withResolution(int widthPixels, int heightPixels) {
    return new CameraModel(
        displayName, horizontalFovDeg, verticalFovDeg, widthPixels, heightPixels);
  }

  public double tanHalfHorizontalFov() {
    return Math.tan(Math.toRadians(horizontalFovDeg) / 2.0);
  }

  public double tanHalfVerticalFov() {
    return Math.tan(Math.toRadians(verticalFovDeg) / 2.0);
  }

  public double diagonalFovDeg() {
    return Math.toDegrees(
        2.0 * Math.atan(Math.hypot(tanHalfHorizontalFov(), tanHalfVerticalFov())));
  }
}
