/**
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  DONT EDIT THIS FILE FOR THE 467 SIM BOT
 * 
 * 
 * 
 * 
 * 
 * 
 */
package first.robot.mechanisms.elevator;

import static org.wpilib.units.Units.Meters;

import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import org.wpilib.units.measure.Distance;

/** Constants for the {@link ElevatorMechanism}. Heights are wrist (claw pivot) heights. */
public final class ElevatorConstants {
  private ElevatorConstants() {}

  // ---- Physical limits ----
  public static final Distance MIN_HEIGHT = Meters.of(0.25);
  public static final Distance MAX_HEIGHT = Meters.of(1.20);
  public static final Distance STARTING_HEIGHT = Meters.of(0.50);

  // ---- Control ----
  /** Volts needed to hold the carriage still against gravity. */
  public static final double KG = 0.70;

  public static final double KP = 30.0; // volts per meter of error
  public static final double KD = 1.5;
  public static final double MAX_VOLTS = 10.0;
  public static final Distance TOLERANCE = Meters.of(0.02);
  public static final Distance TOLERANCE_METERS = TOLERANCE;
  public static final double MANUAL_SPEED_MPS = 0.35;

  // ---- Useful setpoints ----
  /** Low enough that the claw grips the ball sitting on the table. */
  public static final Distance TABLE_GRAB_HEIGHT = FieldConstants.BALL_START_Y.plus(RobotConstants.GRIP_DEPTH);
  public static final Distance GRAB_HEIGHT = TABLE_GRAB_HEIGHT;

  /** Low enough that the claw grips a ball sitting on the floor. */
  public static final Distance GROUND_GRAB_HEIGHT = FieldConstants.BALL_RADIUS.plus(RobotConstants.GRIP_DEPTH);

  /** High enough to carry a ball over the table without hitting it. */
  public static final Distance ABOVE_TABLE_HEIGHT = Meters.of(1.15);

  /** Low height over the floor target to release the ball gently without bouncing. */
  public static final Distance SCORE_HEIGHT = Meters.of(0.45);

  /** Height over the table target to place the ball gently into the table box. */
  public static final Distance SCORE_TABLE_HEIGHT =
      FieldConstants.TABLE_HEIGHT
          .plus(FieldConstants.TARGET_BOX_WALL_HEIGHT)
          .plus(RobotConstants.GRIP_DEPTH)
          .plus(Meters.of(0.05));

  public static final Distance STOW_HEIGHT = Meters.of(0.50);

  /** Height that positions a held ball directly in front of the spring-loaded shooter. */
  public static final Distance SHOOT_HEIGHT = Meters.of(0.60);
}
