package first.robot.subsystems.extender;

import first.robot.constants.FieldConstants;

/**
 * Constants for the {@link ExtenderMechanism}. Positions are how far the arm is extended, in
 * meters (0 = fully retracted). The wrist's x position is {@code WRIST_BASE_X + extension}.
 */
public final class ExtenderConstants {
  private ExtenderConstants() {}

  // ---- Physical limits ----
  public static final double MIN_EXTENSION = 0.0;
  public static final double MAX_EXTENSION = 1.10;
  public static final double STARTING_EXTENSION = 0.0;

  // ---- Simulated hardware ----
  public static final double GEARING = 8.0;
  public static final double SLIDE_MASS_KG = 3.0;
  public static final double PULLEY_RADIUS_METERS = 0.02;

  // ---- Control ----
  public static final double KP = 25.0; // volts per meter of error
  public static final double KD = 1.0;
  public static final double MAX_VOLTS = 10.0;
  public static final double TOLERANCE_METERS = 0.02;
  public static final double MANUAL_SPEED_MPS = 0.50;

  // ---- Useful setpoints ----
  /** Puts the claw directly above the ball on the table. */
  public static final double BALL_EXTENSION =
      FieldConstants.BALL_START_X - FieldConstants.WRIST_BASE_X;

  /** Puts the claw directly above the target zone on the floor. */
  public static final double TARGET_EXTENSION =
      FieldConstants.TARGET_CENTER_X - FieldConstants.WRIST_BASE_X;

  /** Puts the claw directly above the orange target zone on the table. */
  public static final double TABLE_TARGET_EXTENSION =
      FieldConstants.TABLE_TARGET_CENTER_X - FieldConstants.WRIST_BASE_X;
}
