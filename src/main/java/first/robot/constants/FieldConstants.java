package first.robot.constants;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * Shared geometry for the field and game elements (table, balls, targets).
 *
 * <p>x points along the driving axis from the start zone (x = 0) towards the table (x = 2.0),
 * and y points up from the floor.
 */
public final class FieldConstants {
  private FieldConstants() {}

  // ---------------- Field Layout & Drive Positions ----------------

  /** Total length of the field along the driving axis. */
  public static final Distance FIELD_LENGTH = Meters.of(3.6);
  public static final Distance FIELD_LENGTH_METERS = FIELD_LENGTH;

  /** Total width of the field. */
  public static final Distance FIELD_WIDTH = Meters.of(2.4);
  public static final Distance FIELD_WIDTH_METERS = FIELD_WIDTH;

  /** Field X position where the robot starts the match. */
  public static final Distance ROBOT_START_X = Meters.of(0.0);

  /** Field X position where the robot operates next to the table. */
  public static final Distance ROBOT_TABLE_X = Meters.of(2.0);

  // ---------------- Table Obstacle Geometry ----------------

  /** Table left edge X offset relative to the robot when parked at {@link #ROBOT_TABLE_X}. */
  public static final Distance TABLE_LEFT_X = Meters.of(0.70);

  /** Table right edge X offset relative to the robot when parked at {@link #ROBOT_TABLE_X}. */
  public static final Distance TABLE_RIGHT_X = Meters.of(1.30);

  /** Height of the table surface above the floor. */
  public static final Distance TABLE_HEIGHT = Meters.of(0.75);

  // ---------------- Balls ----------------

  public static final Distance BALL_RADIUS = Meters.of(0.12);
  public static final Distance BALL_DIAMETER = BALL_RADIUS.times(2.0);

  /** Ball 1 (Green) starts on top of the table. Relative to the robot when parked at the table. */
  public static final Distance BALL_START_X = Meters.of(1.00);
  public static final Distance BALL_START_Y = TABLE_HEIGHT.plus(BALL_RADIUS);
  public static final Distance BALL1_START_X = BALL_START_X;
  public static final Distance BALL1_START_Y = BALL_START_Y;

  /** Ball 2 (Orange) starts on the ground near the robot's starting location. */
  public static final Distance BALL2_START_X = Meters.of(1.00); // reachable when parked at start with BALL_EXTENSION
  public static final Distance BALL2_START_Y = BALL_RADIUS;

  // ---------------- Target boxes ----------------

  /** Height of the target box side walls (1/3 the height of the ball). */
  public static final Distance TARGET_BOX_WALL_HEIGHT = BALL_DIAMETER.div(3.0); // 0.08m

  /** Green target zone on the floor (relative to robot parked at table). */
  public static final Distance TARGET_LEFT_X = Meters.of(0.28);
  public static final Distance TARGET_RIGHT_X = Meters.of(0.56);
  public static final Distance TARGET_CENTER_X = TARGET_LEFT_X.plus(TARGET_RIGHT_X).div(2.0);

  /** Orange target zone on top of the table (relative to robot parked at table). */
  public static final Distance TABLE_TARGET_LEFT_X = Meters.of(0.86);
  public static final Distance TABLE_TARGET_RIGHT_X = Meters.of(1.14);
  public static final Distance TABLE_TARGET_CENTER_X = TABLE_TARGET_LEFT_X.plus(TABLE_TARGET_RIGHT_X).div(2.0);

  /** Elevated basket target in the air to the right of the table (relative to robot parked at table). */
  public static final Distance BASKET_LEFT_X = Meters.of(1.40);
  public static final Distance BASKET_RIGHT_X = Meters.of(1.72);
  public static final Distance BASKET_CENTER_X = BASKET_LEFT_X.plus(BASKET_RIGHT_X).div(2.0);
  public static final Distance BASKET_HEIGHT = Meters.of(1.25);

  /**
   * The claw angle where the fingers first touch the ball. Closing past
   * this angle is impossible while a ball is between the fingers.
   */
  public static final Angle BALL_CONTACT_ANGLE =
      Radians.of(
          Math.asin(
              (BALL_RADIUS.in(Meters) - RobotConstants.FINGER_PIVOT_HALF_SPACING.in(Meters))
                  / RobotConstants.GRIP_DEPTH.in(Meters)));
}
