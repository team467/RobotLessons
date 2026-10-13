package first.robot.constants;

/**
 * Shared geometry for the robot arm and the world around it (table, ball, target).
 *
 * <p>Everything is in meters, viewed from the side. x points away from the tower (to the right)
 * and y points up from the floor. The tower stands at x = 0.
 *
 * <p>The mechanism constants use these numbers to work out useful setpoints, and the simulator
 * uses them to draw the scene and decide when the claw has grabbed the ball.
 */
public final class FieldConstants {
  private FieldConstants() {}

  // ---------------- Robot arm geometry ----------------

  /** Height of the elevator tower. */
  public static final double TOWER_HEIGHT = 1.75;

  /** x position of the wrist (claw pivot) when the extender is fully retracted. */
  public static final double WRIST_BASE_X = 0.12;

  /** Length of each claw finger, measured from the wrist downward. */
  public static final double FINGER_LENGTH = 0.22;

  /** Distance below the wrist where the fingers grip the ball. */
  public static final double GRIP_DEPTH = 0.15;

  /** Half the distance between the two finger pivots. */
  public static final double FINGER_PIVOT_HALF_SPACING = 0.03;

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

  /**
   * The claw angle (radians from vertical) where the fingers first touch the ball. Closing past
   * this angle is impossible while a ball is between the fingers.
   */
  public static final double BALL_CONTACT_ANGLE =
      Math.asin((BALL_RADIUS - FINGER_PIVOT_HALF_SPACING) / GRIP_DEPTH);
}
