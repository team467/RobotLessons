package first.robot.mechanisms.shooter;

import first.robot.mechanisms.elevator.ElevatorConstants;
import first.robot.mechanisms.extender.ExtenderConstants;

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

  // ---- Simulated hardware ----
  public static final double GEARING = 10.0;
  public static final double PLUNGER_MASS_KG = 1.5;
  public static final double DRUM_RADIUS_METERS = 0.015;

  // ---- Control ----
  public static final double KP = 45.0;
  public static final double KD = 1.0;
  public static final double MAX_VOLTS = 10.0;
  public static final double PULLBACK_VOLTS = 8.0;
  public static final double TOLERANCE_METERS = 0.015;

  /** Speed (m/s) at which the spring snaps the plunger forward when the solenoid releases the ratchet. */
  public static final double SPRING_RELEASE_SPEED_MPS = 5.0;

  // ---- Arm alignment & Launch physics ----
  /** Ideal elevator height (0.60 m) for holding the ball in front of the shooter. */
  public static final double SHOOT_HEIGHT = ElevatorConstants.SHOOT_HEIGHT;

  /** Ideal extender position (0.23 m) for holding the ball in front of the shooter. */
  public static final double SHOOT_EXTENSION = ExtenderConstants.SHOOT_EXTENSION;

  /**
   * Minimum and maximum extender reach (meters) at which the shooter plunger strikes the held ball.
   * If the ball is retracted closer than {@link #MIN_STRIKE_EXTENSION} to the robot, the shooter
   * misses it and does nothing.
   */
  public static final double MIN_STRIKE_EXTENSION = 0.17;
  public static final double MAX_STRIKE_EXTENSION = 0.29;

  /** Minimum and maximum wrist heights (meters) around 0.60 m where a held ball aligns with the shooter plunger. */
  public static final double MIN_STRIKE_WRIST_Y = 0.53;
  public static final double MAX_STRIKE_WRIST_Y = 0.67;

  /**
   * Horizontal and vertical ball launch velocities (m/s) when kicked at full {@link #LOADED_PULLBACK}
   * from {@code elevator = 0.60 m, extender = 0.23 m}, calibrated to arc over the table and drop
   * cleanly into the elevated basket when parked at the table.
   */
  public static final double LAUNCH_VX_MPS = 1.44;
  public static final double LAUNCH_VY_MPS = 5.10;
}
