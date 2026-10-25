package first.robot.constants;

/**
 * Physical geometry and dimensions for the robot arm mechanisms (elevator tower, extender slide, and claw).
 *
 * <p>Everything is in meters, viewed from the side elevation. x points away from the tower
 * (to the right) and y points up from the floor. The tower stands at x = 0.
 */
public final class RobotConstants {
  private RobotConstants() {}

  // ---------------- Robot arm geometry ----------------

  /** Height of the elevator tower (meters). */
  public static final double TOWER_HEIGHT = 1.30;

  /** x position of the wrist (claw pivot) when the extender is fully retracted (meters). */
  public static final double WRIST_BASE_X = 0.12;

  /** Length of each claw finger, measured from the wrist downward (meters). */
  public static final double FINGER_LENGTH = 0.22;

  /** Distance below the wrist where the fingers grip the ball (meters). */
  public static final double GRIP_DEPTH = 0.15;

  /** Half the distance between the two finger pivots (meters). */
  public static final double FINGER_PIVOT_HALF_SPACING = 0.03;
}
