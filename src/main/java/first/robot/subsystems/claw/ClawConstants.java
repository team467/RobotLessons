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
  public static final Angle CLOSED_ANGLE = Radians.of(0.0);
  public static final Angle OPEN_ANGLE = Degrees.of(60);
  public static final Angle STARTING_ANGLE = CLOSED_ANGLE;

  // ---- Control ----
  public static final double KP = 12.0; // volts per radian of error
  public static final double KD = 0.3;
  public static final double MAX_VOLTS = 6.0;
  public static final Angle TOLERANCE = Degrees.of(2);
  public static final Angle TOLERANCE_RADIANS = TOLERANCE;

  /** Volts used to keep squeezing a ball once we have it. Negative closes the claw. */
  public static final double GRIP_VOLTS = -2.0;
}
