package first.robot.sim;

import static first.robot.constants.FieldConstants.*;
import static first.robot.constants.RobotConstants.*;

/**
 * Simple 2D physics for the ball: it sits on the table or floor, falls under gravity, and can be
 * picked up by the claw.
 *
 * <p>This class has no WPILib dependencies so it is easy to unit test.
 */
public class BallSim {
  public enum State {
    ON_TABLE,
    ON_FLOOR,
    FALLING,
    HELD
  }

  private static final double GRAVITY = 9.81;

  /** How close (horizontally) the ball must be to the claw's center to be grabbed. */
  static final double CAPTURE_X = 0.09;

  /** How close (vertically) the ball must be to the claw's grip point to be grabbed. */
  static final double CAPTURE_Y = 0.08;

  /** The claw must open this far past the contact angle to let go of the ball. */
  static final double RELEASE_MARGIN = Math.toRadians(2.5);

  /** The claw must open this far past contact angle to be wide enough to admit the ball between fingers. */
  static final double OPEN_FOR_CAPTURE_MARGIN = Math.toRadians(3.0);

  private final double startX;
  private final double startY;
  private final State initialState;
  private final double tableLeftX;
  private final double tableRightX;
  private final double tableHeight;
  private final double targetLeftX;
  private final double targetRightX;
  private final double targetSurfaceY;
  private double basketLeftX = BASKET_LEFT_X;
  private double basketRightX = BASKET_RIGHT_X;
  private double basketHeight = BASKET_HEIGHT;

  private double x;
  private double y;
  private double vx;
  private double vy;
  private State state;
  private boolean scored;
  private boolean scoredInBasket;
  private boolean readyToGrab;
  private double kickCooldown = 0.0;
  private double previousClawAngle = Double.NaN;

  public BallSim() {
    this(
        BALL_START_X,
        BALL_START_Y,
        State.ON_TABLE,
        TABLE_LEFT_X,
        TABLE_RIGHT_X,
        TABLE_HEIGHT,
        TARGET_LEFT_X,
        TARGET_RIGHT_X,
        0.0);
  }

  public BallSim(double startX, double startY, State initialState) {
    this(
        startX,
        startY,
        initialState,
        TABLE_LEFT_X,
        TABLE_RIGHT_X,
        TABLE_HEIGHT,
        TARGET_LEFT_X,
        TARGET_RIGHT_X,
        0.0);
  }

  public BallSim(
      double startX,
      double startY,
      State initialState,
      double tableLeftX,
      double tableRightX,
      double tableHeight,
      double targetLeftX,
      double targetRightX) {
    this(
        startX,
        startY,
        initialState,
        tableLeftX,
        tableRightX,
        tableHeight,
        targetLeftX,
        targetRightX,
        0.0);
  }

  public BallSim(
      double startX,
      double startY,
      State initialState,
      double tableLeftX,
      double tableRightX,
      double tableHeight,
      double targetLeftX,
      double targetRightX,
      double targetSurfaceY) {
    this.startX = startX;
    this.startY = startY;
    this.initialState = initialState;
    this.tableLeftX = tableLeftX;
    this.tableRightX = tableRightX;
    this.tableHeight = tableHeight;
    this.targetLeftX = targetLeftX;
    this.targetRightX = targetRightX;
    this.targetSurfaceY = targetSurfaceY;
    reset();
  }

  /** Configures the world X bounds and height of the elevated basket target. */
  public void setBasketBounds(double basketLeftX, double basketRightX, double basketHeight) {
    this.basketLeftX = basketLeftX;
    this.basketRightX = basketRightX;
    this.basketHeight = basketHeight;
  }

  /** Puts the ball back at its starting spot. */
  public void reset() {
    x = startX;
    y = startY;
    vx = 0;
    vy = 0;
    state = initialState;
    scored = false;
    scoredInBasket = false;
    readyToGrab = false;
    kickCooldown = 0.0;
    previousClawAngle = Double.NaN;
  }

  /**
   * Kicks the ball into the air with the given initial velocity, knocking it free from the claw.
   *
   * @param launchVx horizontal launch velocity (meters/sec)
   * @param launchVy vertical launch velocity (meters/sec)
   */
  public void kick(double launchVx, double launchVy) {
    state = State.FALLING;
    vx = launchVx;
    vy = launchVy;
    scored = false;
    scoredInBasket = false;
    readyToGrab = false;
    kickCooldown = 0.25;
  }

  /**
   * Advances the ball by one time step.
   *
   * @param dt time step in seconds
   * @param wristX x position of the claw pivot
   * @param wristY y position of the claw pivot
   * @param clawAngle how far open the claw is, in radians (0 = closed)
   * @return the angle the claw fingers can't close past (because the ball is in the way), or NaN
   *     if nothing is blocking them
   */
  public double update(double dt, double wristX, double wristY, double clawAngle) {
    double gripX = wristX;
    double gripY = wristY - GRIP_DEPTH;
    double lastAngle = Double.isNaN(previousClawAngle) ? clawAngle : previousClawAngle;
    previousClawAngle = clawAngle;

    if (kickCooldown > 0.0) {
      kickCooldown = Math.max(0.0, kickCooldown - dt);
    }

    if (state == State.HELD) {
      readyToGrab = false;
      if (clawAngle > BALL_CONTACT_ANGLE + RELEASE_MARGIN) {
        // Let go! The ball keeps whatever velocity the claw gave it.
        state = State.FALLING;
      } else {
        vx = (gripX - x) / dt;
        vy = (gripY - y) / dt;
        x = gripX;
        y = gripY;
        return BALL_CONTACT_ANGLE;
      }
    }

    // Try to grab: the ball must be in the capture zone, the claw must have opened
    // wide enough to fit over the ball, and then close onto the ball.
    boolean inCaptureZone =
        kickCooldown <= 0.0
            && Math.abs(x - gripX) < CAPTURE_X
            && Math.abs(y - gripY) < CAPTURE_Y;
    if (!inCaptureZone) {
      readyToGrab = false;
    } else if (clawAngle >= BALL_CONTACT_ANGLE + OPEN_FOR_CAPTURE_MARGIN) {
      readyToGrab = true;
    }

    boolean closingOntoBall =
        clawAngle <= BALL_CONTACT_ANGLE + 0.03
            && (lastAngle >= BALL_CONTACT_ANGLE || clawAngle <= lastAngle + 1e-4);
    if (inCaptureZone && readyToGrab && closingOntoBall) {
      state = State.HELD;
      readyToGrab = false;
      scored = false;
      scoredInBasket = false;
      vx = 0;
      vy = 0;
      x = gripX;
      y = gripY;
      return BALL_CONTACT_ANGLE;
    }

    if (state == State.FALLING) {
      double previousBottom = y - BALL_RADIUS;
      vy -= GRAVITY * dt;
      x += vx * dt;
      y += vy * dt;
      double bottom = y - BALL_RADIUS;

      boolean overBasket = x >= basketLeftX && x <= basketRightX;
      boolean overTable = x >= tableLeftX && x <= tableRightX;
      if (overBasket && vy <= 0.0 && bottom <= basketHeight && previousBottom >= basketHeight - 0.02) {
        land(basketHeight + BALL_RADIUS, State.ON_TABLE);
        scored = true;
        scoredInBasket = true;
      } else if (overTable && bottom <= tableHeight && previousBottom >= tableHeight - 0.01) {
        if (Math.abs(vy) > 2.4) {
          y = tableHeight + BALL_RADIUS;
          vy = -vy * 0.55;
          vx = (Math.abs(vx) < 0.2) ? 0.8 : vx * 0.8;
          scored = false;
          scoredInBasket = false;
        } else {
          land(tableHeight + BALL_RADIUS, State.ON_TABLE);
          if (Math.abs(targetSurfaceY - tableHeight) < 0.1) {
            scored = x >= targetLeftX && x <= targetRightX;
          }
        }
      } else if (bottom <= 0.0) {
        if (Math.abs(vy) > 2.4 || (Math.abs(vx) > 0.3 && Math.abs(vy) > 0.6)) {
          // Dropped from too high or still bouncing: bounce away!
          y = BALL_RADIUS;
          vy = -vy * 0.60;
          if (Math.abs(vx) < 0.2) {
            vx = 1.6; // kick away horizontally past the target zone
          } else {
            vx *= 0.8;
          }
          scored = false;
          scoredInBasket = false;
        } else {
          land(BALL_RADIUS, State.ON_FLOOR);
          if (Math.abs(targetSurfaceY - 0.0) < 0.1) {
            scored = x >= targetLeftX && x <= targetRightX;
          }
        }
      }
    }
    return Double.NaN;
  }

  private void land(double restingY, State newState) {
    y = restingY;
    vx = 0;
    vy = 0;
    state = newState;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public State getState() {
    return state;
  }

  public boolean isHeld() {
    return state == State.HELD;
  }

  /** True when the ball is resting inside its target zone or in the elevated basket. */
  public boolean isScored() {
    return scored;
  }

  /** True when the ball is resting inside the elevated basket target. */
  public boolean isScoredInBasket() {
    return scoredInBasket;
  }
}
