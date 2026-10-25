package first.robot.constants;

import java.awt.event.KeyEvent;

/** Constants for driver station hardware and simulator keyboard bindings. */
public final class DriverConstants {
  private DriverConstants() {}

  /** Driver gamepad USB index on the Driver Station. */
  public static final int DRIVER_CONTROLLER_PORT = 1;

  // Simulator keyboard bindings (active when the simulation window has focus):
  /** Hold to jog elevator carriage upward. */
  public static final int KEY_ELEVATOR_UP = KeyEvent.VK_UP;
  /** Hold to jog elevator carriage downward. */
  public static final int KEY_ELEVATOR_DOWN = KeyEvent.VK_DOWN;
  /** Hold to extend horizontal arm outward. */
  public static final int KEY_EXTENDER_OUT = KeyEvent.VK_RIGHT;
  /** Hold to retract horizontal arm inward. */
  public static final int KEY_EXTENDER_IN = KeyEvent.VK_LEFT;
  /** Press to open claw fingers. */
  public static final int KEY_CLAW_OPEN = KeyEvent.VK_Z;
  /** Press to close claw fingers. */
  public static final int KEY_CLAW_CLOSE = KeyEvent.VK_X;
  /** Shortcut to reset simulation state, balls, and mechanisms (R key). */
  public static final int KEY_RESET_STATE = KeyEvent.VK_R;
  public static final int KEY_RESET_BALL = KEY_RESET_STATE;
  /** Shortcut to activate Teleop mode (T key). */
  public static final int KEY_TELEOP = KeyEvent.VK_T;
  /** Hold to drive forward towards the table (D key). */
  public static final int KEY_DRIVE_FORWARD = KeyEvent.VK_D;
  /** Hold to drive backward towards the start zone (A key). */
  public static final int KEY_DRIVE_BACKWARD = KeyEvent.VK_A;
  /** Press to pull back / cock the spring-loaded shooter against its ratchet (C key). */
  public static final int KEY_SHOOTER_PULL = KeyEvent.VK_C;
  /** Press to release the shooter solenoid and kick the ball toward the basket (Space key). */
  public static final int KEY_SHOOTER_KICK = KeyEvent.VK_SPACE;
  /** Press to move the arm (elevator and extender) to the shooting position (V key). */
  public static final int KEY_SHOOT_POSITION = KeyEvent.VK_V;
}
