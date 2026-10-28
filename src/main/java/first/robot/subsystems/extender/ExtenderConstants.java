package first.robot.subsystems.extender;

import static org.wpilib.units.Units.Meters;

import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import org.wpilib.units.measure.Distance;

/**
 * Constants for the {@link Extender}. Positions are how far the arm is extended
 * (0 = fully retracted). The wrist's x position is {@code WRIST_BASE_X + extension}.
 */
public final class ExtenderConstants {
  private ExtenderConstants() {}

  // ---- Physical limits ----
  /** Minimum horizontal arm reach: fully retracted inward against the carriage (0.0m). */
  public static final Distance MIN_EXTENSION = Meters.of(0.0);

  /** Maximum physical forward stroke of the telescoping slide (1.10m). */
  public static final Distance MAX_EXTENSION = Meters.of(1.10);

  /** Starting default extension position upon robot launch (0.33m). */
  public static final Distance STARTING_EXTENSION = Meters.of(0.33);

  // ---- Control ----
  /** Proportional gain: volts applied per meter of extension position error. */
  public static final double KP = 25.0;

  /** Derivative gain: volts applied per (meter/second) of velocity to damp slide motion. */
  public static final double KD = 1.0;

  /** Maximum voltage applied to the extender pulley motor during motion. */
  public static final double MAX_VOLTS = 10.0;

  /** Deadband tolerance within which the slide is considered arrived at its setpoint (2 cm). */
  public static final Distance TOLERANCE = Meters.of(0.02);
  public static final Distance TOLERANCE_METERS = TOLERANCE;

  /** Manual jogging speed when holding keyboard Left/Right arrows or controller D-pad (0.50 m/s). */
  public static final double MANUAL_SPEED_MPS = 0.50;

  // ---- Useful setpoints ----
  /** Extends arm so the claw is centered directly above Ball 1 resting on the table (~0.76m). */
  public static final Distance BALL_EXTENSION =
      FieldConstants.BALL_START_X.minus(RobotConstants.WRIST_BASE_X);

  /** Extends arm so the claw is centered directly over the green floor target zone (~0.58m). */
  public static final Distance TARGET_EXTENSION =
      FieldConstants.TARGET_CENTER_X.minus(RobotConstants.WRIST_BASE_X);

  /** Extends arm so the claw is centered directly over the orange table target box (~0.68m). */
  public static final Distance TABLE_TARGET_EXTENSION =
      FieldConstants.TABLE_TARGET_CENTER_X.minus(RobotConstants.WRIST_BASE_X);

  /** Retracts arm to 0.23m so a held ball sits directly in front of the spring-loaded kicker plunger. */
  public static final Distance SHOOT_EXTENSION = Meters.of(0.23);
  public static final Distance KICK_EXTENSION = SHOOT_EXTENSION;
}
