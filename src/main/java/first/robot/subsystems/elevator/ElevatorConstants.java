package first.robot.subsystems.elevator;

import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;

/** Constants for the {@link ElevatorMechanism}. Heights are wrist (claw pivot) heights in meters. */
public final class ElevatorConstants {
  private ElevatorConstants() {}

  // ---- Physical limits ----
  public static final double MIN_HEIGHT = 0.25;
  public static final double MAX_HEIGHT = 1.55;
  public static final double STARTING_HEIGHT = 0.50;

  // ---- Simulated hardware ----
  public static final double GEARING = 10.0; // 10:1 reduction
  public static final double CARRIAGE_MASS_KG = 6.0; // carriage + extender + claw
  public static final double DRUM_RADIUS_METERS = 0.025;

  // ---- Control ----
  /** Volts needed to hold the carriage still against gravity. */
  public static final double KG = 0.70;

  public static final double KP = 30.0; // volts per meter of error
  public static final double KD = 1.5;
  public static final double MAX_VOLTS = 10.0;
  public static final double TOLERANCE_METERS = 0.02;
  public static final double MANUAL_SPEED_MPS = 0.35;

  // ---- Useful setpoints ----
  /** Low enough that the claw grips the ball sitting on the table. */
  public static final double TABLE_GRAB_HEIGHT = FieldConstants.BALL_START_Y + RobotConstants.GRIP_DEPTH;
  public static final double GRAB_HEIGHT = TABLE_GRAB_HEIGHT;

  /** Low enough that the claw grips a ball sitting on the floor. */
  public static final double GROUND_GRAB_HEIGHT = FieldConstants.BALL_RADIUS + RobotConstants.GRIP_DEPTH;

  /** High enough to carry a ball over the table without hitting it. */
  public static final double ABOVE_TABLE_HEIGHT = 1.35;

  /** Low height over the floor target to release the ball gently without bouncing. */
  public static final double SCORE_HEIGHT = 0.45;

  /** Height over the table target to place the ball gently into the table box. */
  public static final double SCORE_TABLE_HEIGHT =
      FieldConstants.TABLE_HEIGHT + FieldConstants.TARGET_BOX_WALL_HEIGHT + RobotConstants.GRIP_DEPTH + 0.05;

  public static final double STOW_HEIGHT = 0.50;
}
