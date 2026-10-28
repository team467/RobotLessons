package first.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import first.robot.constants.FieldConstants;
import org.wpilib.units.measure.Distance;

/** Constants for the drivetrain mechanism. */
public final class DriveConstants {
  private DriveConstants() {}

  // ---- Control ----
  /** Manual jogging speed across the field during teleoperation (5.5 m/s). */
  public static final double TELEOP_SPEED_MPS = 5.5;

  /** Standard maximum voltage clamp applied to drive motors during autonomous trajectory driving. */
  public static final double MAX_VOLTS = 9.0;

  /** Boosted voltage limit applied during manual teleop jogging for snappy acceleration. */
  public static final double TELEOP_VOLTS = 11.5;

  /** Proportional gain: volts applied per meter of position error. */
  public static final double KP = 8.5;

  /** Derivative gain: tuned to 1.8 for critical damping, eliminating chassis overshoot. */
  public static final double KD = 1.8;

  /** Position deadband within which the chassis is considered arrived at its destination (3 cm). */
  public static final Distance TOLERANCE = Meters.of(0.03);
  public static final Distance TOLERANCE_METERS = TOLERANCE;

  // ---- Field Waypoints ----
  /** Starting field line x-coordinate where the robot begins and Ball 2 is located (0.0m). */
  public static final Distance START_POSITION = FieldConstants.ROBOT_START_X;

  /** Field x-coordinate where the robot is parked against the table obstacle (0.80m). */
  public static final Distance TABLE_POSITION = FieldConstants.ROBOT_TABLE_X;
}
