package first.robot.constants;

import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Distance;

/**
 * Physical geometry and dimensions for the robot arm mechanisms (elevator tower, extender slide, and claw).
 *
 * <p>Everything is viewed from the side elevation. x points away from the tower
 * (to the right) and y points up from the floor. The tower stands at x = 0.
 */
public final class RobotConstants {
  private RobotConstants() {}

  // ---------------- Robot arm geometry ----------------

  /** Height of the elevator tower. */
  public static final Distance TOWER_HEIGHT = Meters.of(1.30);

  /** x position of the wrist (claw pivot) when the extender is fully retracted. */
  public static final Distance WRIST_BASE_X = Meters.of(0.12);

  /** Length of each claw finger, measured from the wrist downward. */
  public static final Distance FINGER_LENGTH = Meters.of(0.22);

  /** Distance below the wrist where the fingers grip the ball. */
  public static final Distance GRIP_DEPTH = Meters.of(0.15);

  /** Half the distance between the two finger pivots. */
  public static final Distance FINGER_PIVOT_HALF_SPACING = Meters.of(0.03);
}
