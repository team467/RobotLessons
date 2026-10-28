package first.robot.subsystems.claw;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import org.wpilib.units.measure.Angle;

/**
 * Constants for the {@link Claw}. The claw angle is how far each finger swings away from
 * vertical (0 = fully closed).
 */
public final class ClawConstants {
  private ClawConstants() {}

  // ---- Physical limits ----
  /** Pincher angle when fingers are fully closed touching each other (0 radians). */
  public static final Angle CLOSED_ANGLE = Radians.of(0.0);

  /** Pincher angle when fingers are opened wide to encompass a game ball (60 degrees). */
  public static final Angle OPEN_ANGLE = Degrees.of(60);

  /** Initial default pincher angle upon robot start (closed). */
  public static final Angle STARTING_ANGLE = CLOSED_ANGLE;

  // ---- Control ----
  /** Proportional gain: volts applied per radian of angular position error. */
  public static final double KP = 12.0;

  /** Derivative gain: volts applied per (radian/second) of angular velocity to damp oscillations. */
  public static final double KD = 0.3;

  /** Maximum voltage clamp applied to the claw motor during position moves. */
  public static final double MAX_VOLTS = 6.0;

  /** Angular deadband within which the claw is considered settled at its setpoint. */
  public static final Angle TOLERANCE = Degrees.of(2);
  public static final Angle TOLERANCE_RADIANS = TOLERANCE;

  /**
   * Constant voltage used to maintain active pinch grip on a held ball.
   * Negative polarity drives fingers inward; 2.0V provides firm grip without motor overheating.
   */
  public static final double GRIP_VOLTS = -2.0;
}
