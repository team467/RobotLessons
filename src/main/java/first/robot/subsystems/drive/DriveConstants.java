package first.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import first.robot.constants.FieldConstants;
import org.wpilib.units.measure.Distance;

/** Constants for the drivetrain mechanism. */
public final class DriveConstants {
  private DriveConstants() {}

  // ---- Control ----
  public static final double TELEOP_SPEED_MPS = 5.5; // faster manual drive with keyboard A/D
  public static final double MAX_VOLTS = 9.0;
  public static final double TELEOP_VOLTS = 11.5;
  public static final double KP = 8.5; // volts per meter of error
  public static final double KD = 1.8;
  public static final Distance TOLERANCE = Meters.of(0.03);
  public static final Distance TOLERANCE_METERS = TOLERANCE;

  // ---- Field Waypoints ----
  public static final Distance START_POSITION = FieldConstants.ROBOT_START_X;
  public static final Distance TABLE_POSITION = FieldConstants.ROBOT_TABLE_X;
}
