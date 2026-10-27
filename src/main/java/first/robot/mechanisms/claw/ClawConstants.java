package first.robot.mechanisms.claw;

/**
 * Constants for the {@link ClawMechanism}. The claw angle is how far each finger swings away from
 * vertical, in radians (0 = fully closed).
 */
public final class ClawConstants {
  private ClawConstants() {}

  // ---- Physical limits ----
  public static final double CLOSED_ANGLE = 0.0;
  public static final double OPEN_ANGLE = Math.toRadians(60);
  public static final double STARTING_ANGLE = CLOSED_ANGLE;

  // ---- Control ----
  public static final double KP = 12.0; // volts per radian of error
  public static final double KD = 0.3;
  public static final double MAX_VOLTS = 6.0;
  public static final double TOLERANCE_RADIANS = Math.toRadians(2);

  /** Volts used to keep squeezing a ball once we have it. Negative closes the claw. */
  public static final double GRIP_VOLTS = -2.0;
}
