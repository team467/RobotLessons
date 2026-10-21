package first.robot.constants;

/**
 * Shared geometry for the field and game elements (table, balls, targets).
 *
 * <p>Everything is in meters, viewed from the side. x points along the driving axis from the start
 * zone (x = 0) towards the table (x = 2.0), and y points up from the floor.
 */
public final class FieldConstants {
  private FieldConstants() {}

  // ---------------- Field Layout & Drive Positions ----------------

  /** Total length of the field along the driving axis (meters). */
  public static final double FIELD_LENGTH_METERS = 3.6;

  /** Total width of the field (meters). */
  public static final double FIELD_WIDTH_METERS = 2.4;

  /** Field X position where the robot starts the match (meters). */
  public static final double ROBOT_START_X = 0.0;

  /** Field X position where the robot operates next to the table (meters). */
  public static final double ROBOT_TABLE_X = 2.0;

  // ---------------- Table Obstacle Geometry ----------------

  /** Table left edge X offset relative to the robot when parked at {@link #ROBOT_TABLE_X} (meters). */
  public static final double TABLE_LEFT_X = 0.70;

  /** Table right edge X offset relative to the robot when parked at {@link #ROBOT_TABLE_X} (meters). */
  public static final double TABLE_RIGHT_X = 1.30;

  /** Height of the table surface above the floor (meters). */
  public static final double TABLE_HEIGHT = 0.75;

  // ---------------- Balls ----------------

  public static final double BALL_RADIUS = 0.12;
  public static final double BALL_DIAMETER = 2.0 * BALL_RADIUS;

  /** Ball 1 (Green) starts on top of the table. Relative to the robot when parked at the table. */
  public static final double BALL_START_X = 1.00;
  public static final double BALL_START_Y = TABLE_HEIGHT + BALL_RADIUS;
  public static final double BALL1_START_X = BALL_START_X;
  public static final double BALL1_START_Y = BALL_START_Y;

  /** Ball 2 (Orange) starts on the ground near the robot's starting location. */
  public static final double BALL2_START_X = 1.00; // reachable when parked at start with BALL_EXTENSION
  public static final double BALL2_START_Y = BALL_RADIUS;

  // ---------------- Target boxes ----------------

  /** Height of the target box side walls (1/3 the height of the ball). */
  public static final double TARGET_BOX_WALL_HEIGHT = BALL_DIAMETER / 3.0; // 0.08m

  /** Green target zone on the floor (relative to robot parked at table). */
  public static final double TARGET_LEFT_X = 0.28;
  public static final double TARGET_RIGHT_X = 0.56;
  public static final double TARGET_CENTER_X = (TARGET_LEFT_X + TARGET_RIGHT_X) / 2.0;

  /** Orange target zone on top of the table (relative to robot parked at table). */
  public static final double TABLE_TARGET_LEFT_X = 0.86;
  public static final double TABLE_TARGET_RIGHT_X = 1.14;
  public static final double TABLE_TARGET_CENTER_X = (TABLE_TARGET_LEFT_X + TABLE_TARGET_RIGHT_X) / 2.0;

  /** Elevated basket target in the air to the right of the table (relative to robot parked at table). */
  public static final double BASKET_LEFT_X = 1.40;
  public static final double BASKET_RIGHT_X = 1.72;
  public static final double BASKET_CENTER_X = (BASKET_LEFT_X + BASKET_RIGHT_X) / 2.0;
  public static final double BASKET_HEIGHT = 1.25;

  /**
   * The claw angle (radians from vertical) where the fingers first touch the ball. Closing past
   * this angle is impossible while a ball is between the fingers.
   */
  public static final double BALL_CONTACT_ANGLE =
      Math.asin((BALL_RADIUS - RobotConstants.FINGER_PIVOT_HALF_SPACING) / RobotConstants.GRIP_DEPTH);
}
