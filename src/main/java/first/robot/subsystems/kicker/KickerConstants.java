package first.robot.subsystems.kicker;

import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Distance;

/**
 * Constants for the spring-loaded {@link Kicker} mounted on the front of the robot.
 *
 * <p>A pullback winch motor winds the plunger back against a heavy spring while a one-way ratchet
 * locks it in place. Releasing the solenoid disengages the ratchet, allowing the spring to snap
 * the kicker forward and launch a ball held close to the robot toward the elevated basket.
 */
public final class KickerConstants {
  private KickerConstants() {}

  // ---- Physical limits (spring pullback) ----
  /** Plunger fully extended forward against the mechanical stop / uncocked (0.0m). */
  public static final Distance MIN_PULLBACK = Meters.of(0.0);

  /** Maximum mechanical winch pullback travel before hard stop (0.25m). */
  public static final Distance MAX_PULLBACK = Meters.of(0.25);

  /** Standard cocked pullback distance where the ratchet catches and locks spring tension (0.22m). */
  public static final Distance LOADED_PULLBACK = Meters.of(0.22);

  /** Starting pullback position upon robot launch (uncocked, 0.0m). */
  public static final Distance STARTING_PULLBACK = Meters.of(0.0);

  // ---- Control ----
  /** Proportional gain: volts applied per meter of pullback error. */
  public static final double KP = 45.0;

  /** Derivative gain: volts applied per (meter/second) of velocity to damp winch motion. */
  public static final double KD = 1.0;

  /** Maximum voltage clamp applied to the pullback motor. */
  public static final double MAX_VOLTS = 10.0;

  /** Minimum voltage applied to overcome the heavy spring resistance during winding (8.0V). */
  public static final double PULLBACK_VOLTS = 8.0;

  /** Position deadband within which the plunger is considered loaded (1.5 cm). */
  public static final Distance TOLERANCE = Meters.of(0.015);
  public static final Distance TOLERANCE_METERS = TOLERANCE;
}
