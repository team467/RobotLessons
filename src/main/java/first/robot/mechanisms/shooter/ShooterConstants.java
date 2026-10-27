package first.robot.mechanisms.shooter;

/**
 * Constants for the spring-loaded {@link ShooterMechanism} mounted on the front of the robot.
 *
 * <p>A pullback winch motor winds the plunger back against a heavy spring while a one-way ratchet
 * locks it in place. Releasing the solenoid disengages the ratchet, allowing the spring to snap
 * the kicker forward and launch a ball held close to the robot toward the elevated basket.
 */
public final class ShooterConstants {
  private ShooterConstants() {}

  // ---- Physical limits (meters of spring pullback) ----
  public static final double MIN_PULLBACK = 0.0;
  public static final double MAX_PULLBACK = 0.25;
  public static final double LOADED_PULLBACK = 0.22;
  public static final double STARTING_PULLBACK = 0.0;

  // ---- Control ----
  public static final double KP = 45.0;
  public static final double KD = 1.0;
  public static final double MAX_VOLTS = 10.0;
  public static final double PULLBACK_VOLTS = 8.0;
  public static final double TOLERANCE_METERS = 0.015;
}
