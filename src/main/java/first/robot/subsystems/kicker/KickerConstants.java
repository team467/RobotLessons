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
  public static final Distance MIN_PULLBACK = Meters.of(0.0);
  public static final Distance MAX_PULLBACK = Meters.of(0.25);
  public static final Distance LOADED_PULLBACK = Meters.of(0.22);
  public static final Distance STARTING_PULLBACK = Meters.of(0.0);

  // ---- Control ----
  public static final double KP = 45.0;
  public static final double KD = 1.0;
  public static final double MAX_VOLTS = 10.0;
  public static final double PULLBACK_VOLTS = 8.0;
  public static final Distance TOLERANCE = Meters.of(0.015);
  public static final Distance TOLERANCE_METERS = TOLERANCE;
}
