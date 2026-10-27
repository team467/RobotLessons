package first.robot.mechanisms.drive;

import first.robot.constants.FieldConstants;

/** Constants for the drivetrain mechanism. */
public final class DriveConstants {
  private DriveConstants() {}

  // ---- Control ----
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
