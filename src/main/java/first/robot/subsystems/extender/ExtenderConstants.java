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
  public static final Distance MIN_EXTENSION = Meters.of(0.0);
  public static final Distance MAX_EXTENSION = Meters.of(1.10);
  public static final Distance STARTING_EXTENSION = Meters.of(0.33);

  // ---- Control ----
  public static final double KP = 25.0; // volts per meter of error
  public static final double KD = 1.0;
  public static final double MAX_VOLTS = 10.0;
  public static final Distance TOLERANCE = Meters.of(0.02);
  public static final Distance TOLERANCE_METERS = TOLERANCE;
  public static final double MANUAL_SPEED_MPS = 0.50;

  // ---- Useful setpoints ----
  /** Puts the claw directly above the ball on the table. */
  public static final Distance BALL_EXTENSION =
      FieldConstants.BALL_START_X.minus(RobotConstants.WRIST_BASE_X);

  /** Puts the claw directly above the target zone on the floor. */
  public static final Distance TARGET_EXTENSION =
      FieldConstants.TARGET_CENTER_X.minus(RobotConstants.WRIST_BASE_X);

  /** Puts the claw directly above the orange target zone on the table. */
  public static final Distance TABLE_TARGET_EXTENSION =
      FieldConstants.TABLE_TARGET_CENTER_X.minus(RobotConstants.WRIST_BASE_X);

  /** Positions the arm so the held ball sits directly in front of the kicker. */
  public static final Distance SHOOT_EXTENSION = Meters.of(0.23);
  public static final Distance KICK_EXTENSION = SHOOT_EXTENSION;
}
