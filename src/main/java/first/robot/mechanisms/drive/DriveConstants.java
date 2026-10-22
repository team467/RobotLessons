package first.robot.mechanisms.drive;

import first.robot.constants.FieldConstants;

/** Constants for the drivetrain mechanism. */
public final class DriveConstants {
  private DriveConstants() {}

  // ---- Physical robot dimensions ----
  public static final double WHEEL_RADIUS_METERS = 0.076; // ~3 inches
  public static final double ROBOT_MASS_KG = 25.0; // chassis + battery + mechanisms
  public static final double CHASSIS_LENGTH_METERS = 0.70;
  public static final double CHASSIS_WIDTH_METERS = 0.60;
  public static final double GEARING = 8.45; // 8.45:1 standard toughbox reduction

  // ---- Control ----
  public static final double MAX_SPEED_MPS = 3.5;
  public static final double TELEOP_SPEED_MPS = 5.5; // faster manual drive with keyboard A/D
  public static final double MAX_VOLTS = 9.0;
  public static final double TELEOP_VOLTS = 11.5;
  public static final double KP = 8.5; // volts per meter of error
  public static final double KD = 0.7;
  public static final double TOLERANCE_METERS = 0.03;

  // ---- Field Waypoints ----
  public static final double START_POSITION = FieldConstants.ROBOT_START_X;
  public static final double TABLE_POSITION = FieldConstants.ROBOT_TABLE_X;
}
