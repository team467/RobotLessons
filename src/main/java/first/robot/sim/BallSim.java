package first.robot.sim;

import static first.robot.constants.FieldConstants.*;
import static first.robot.constants.RobotConstants.*;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

/**
 * Simple 2D physics for the ball: it sits on the table or floor, falls under gravity, and can be
 * picked up by the claw.
 */
public class BallSim {
  public enum State {
    ON_TABLE,
    ON_FLOOR,
    FALLING,
    HELD
  }

  private static final double GRAVITY = 9.81;

  private static final double BALL_RADIUS_METERS = BALL_RADIUS.in(Meters);
  private static final double GRIP_DEPTH_METERS = GRIP_DEPTH.in(Meters);
  private static final double BALL_CONTACT_ANGLE_RAD = BALL_CONTACT_ANGLE.in(Radians);

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
  private double basketLeftX = BASKET_LEFT_X.in(Meters);
  private double basketRightX = BASKET_RIGHT_X.in(Meters);
  private double basketHeight = BASKET_HEIGHT.in(Meters);
  private double floorTargetLeftX = TARGET_LEFT_X.in(Meters);
  private double floorTargetRightX = TARGET_RIGHT_X.in(Meters);

  private double x;
  private double y;
  private double vx;
  private double vy;
  private State state;
  private boolean scored;
  private boolean scoredInBasket;
  private boolean readyToGrab;
  private boolean tableOccupied;
  private boolean floorTargetOccupied;
  private double kickCooldown = 0.0;
  private double previousClawAngle = Double.NaN;

  public BallSim() {
    this(
        BALL_START_X.in(Meters),
        BALL_START_Y.in(Meters),
        State.ON_TABLE,
        TABLE_LEFT_X.in(Meters),
        TABLE_RIGHT_X.in(Meters),
        TABLE_HEIGHT.in(Meters),
        TARGET_LEFT_X.in(Meters),
        TARGET_RIGHT_X.in(Meters),
        0.0);
  }

  public BallSim(double startX, double startY, State initialState) {
    this(
        startX,
        startY,
        initialState,
        TABLE_LEFT_X.in(Meters),
        TABLE_RIGHT_X.in(Meters),
        TABLE_HEIGHT.in(Meters),
        TARGET_LEFT_X.in(Meters),
        TARGET_RIGHT_X.in(Meters),
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
    this.floorTargetLeftX = targetLeftX;
    this.floorTargetRightX = targetRightX;
    reset();
  }

  /** Configures the world X bounds and height of the elevated basket target. */
  public void setBasketBounds(double basketLeftX, double basketRightX, double basketHeight) {
    this.basketLeftX = basketLeftX;
    this.basketRightX = basketRightX;
    this.basketHeight = basketHeight;
  }

  /** Configures the world X bounds of the green floor target box. */
  public void setFloorTargetBounds(double floorTargetLeftX, double floorTargetRightX) {
    this.floorTargetLeftX = floorTargetLeftX;
    this.floorTargetRightX = floorTargetRightX;
  }

  /** Sets whether another ball is currently occupying the table target. */
  public void setTableOccupied(boolean tableOccupied) {
    this.tableOccupied = tableOccupied;
  }

  /** Sets whether another ball is currently occupying the green floor target. */
  public void setFloorTargetOccupied(boolean floorTargetOccupied) {
    this.floorTargetOccupied = floorTargetOccupied;
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
    tableOccupied = false;
    floorTargetOccupied = false;
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
    double gripY = wristY - GRIP_DEPTH_METERS;
    double lastAngle = Double.isNaN(previousClawAngle) ? clawAngle : previousClawAngle;
    previousClawAngle = clawAngle;

    if (kickCooldown > 0.0) {
      kickCooldown = Math.max(0.0, kickCooldown - dt);
    }

    if (state == State.HELD) {
      readyToGrab = false;
      if (clawAngle > BALL_CONTACT_ANGLE_RAD + RELEASE_MARGIN) {
        // Let go! The ball keeps whatever velocity the claw gave it.
        state = State.FALLING;
      } else {
        vx = (gripX - x) / dt;
        vy = (gripY - y) / dt;
        x = gripX;
        y = gripY;
        return BALL_CONTACT_ANGLE_RAD;
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
    } else if (clawAngle >= BALL_CONTACT_ANGLE_RAD + OPEN_FOR_CAPTURE_MARGIN) {
      readyToGrab = true;
    }

    boolean closingOntoBall =
        clawAngle <= BALL_CONTACT_ANGLE_RAD + 0.03
            && (lastAngle >= BALL_CONTACT_ANGLE_RAD || clawAngle <= lastAngle + 1e-4);
    if (inCaptureZone && readyToGrab && closingOntoBall) {
      state = State.HELD;
      readyToGrab = false;
      scored = false;
      scoredInBasket = false;
      vx = 0;
      vy = 0;
      x = gripX;
      y = gripY;
      return BALL_CONTACT_ANGLE_RAD;
    }

    if (state == State.ON_FLOOR && Math.abs(vx) > 0.01) {
      x += vx * dt;
      // Apply rolling friction on the floor
      double decel = 1.8 * dt;
      if (Math.abs(vx) <= decel) {
        vx = 0.0;
      } else {
        vx -= Math.signum(vx) * decel;
      }
      if (Math.abs(targetSurfaceY - 0.0) < 0.1) {
        scored = Math.abs(vx) < 0.05 && x >= targetLeftX && x <= targetRightX;
      }
    }

    if (state == State.FALLING) {
      double previousBottom = y - BALL_RADIUS_METERS;
      vy -= GRAVITY * dt;
      x += vx * dt;
      y += vy * dt;
      double bottom = y - BALL_RADIUS_METERS;

      boolean overBasket = x >= basketLeftX && x <= basketRightX;
      boolean overTable = x >= tableLeftX && x <= tableRightX;
      if (overBasket && vy <= 0.0 && bottom <= basketHeight && previousBottom >= basketHeight - 0.02) {
        land(basketHeight + BALL_RADIUS_METERS, State.ON_TABLE);
        scored = true;
        scoredInBasket = true;
      } else if (overTable && bottom <= tableHeight && previousBottom >= tableHeight - 0.03) {
        if (Math.abs(vy) > 2.4 || tableOccupied) {
          y = tableHeight + BALL_RADIUS_METERS;
          vy = Math.max(Math.abs(vy) * 0.55, 1.5);
          vx = (Math.abs(vx) < 0.2) ? 0.8 : vx * 0.8;
          scored = false;
          scoredInBasket = false;
        } else {
          land(tableHeight + BALL_RADIUS_METERS, State.ON_TABLE);
          if (Math.abs(targetSurfaceY - tableHeight) < 0.1) {
            scored = x >= targetLeftX && x <= targetRightX;
          }
        }
      } else if (bottom <= 0.0) {
        boolean overFloorTarget = x >= floorTargetLeftX && x <= floorTargetRightX;
        if (Math.abs(vy) > 2.4
            || (Math.abs(vx) > 0.3 && Math.abs(vy) > 0.6)
            || (overFloorTarget && (floorTargetOccupied || targetSurfaceY > 0.1))) {
          // Dropped from too high, still bouncing, or dropped onto wrong/occupied floor target: bounce away!
          y = BALL_RADIUS_METERS;
          vy = Math.max(-vy * 0.60, 1.4);
          if (Math.abs(vx) < 0.2) {
            vx = 1.6; // kick away horizontally past the target zone
          } else {
            vx *= 0.8;
          }
          scored = false;
          scoredInBasket = false;
        } else {
          y = BALL_RADIUS_METERS;
          vy = 0;
          vx *= 0.7;
          if (Math.abs(vx) < 0.08) {
            vx = 0;
          }
          state = State.ON_FLOOR;
          if (Math.abs(targetSurfaceY - 0.0) < 0.1) {
            scored = Math.abs(vx) < 0.05 && x >= targetLeftX && x <= targetRightX;
          }
        }
      }
    }
    return Double.NaN;
  }

  /**
   * Checks if the robot's chassis bumper (extending from {@code robotX - 0.35} to {@code robotX +
   * 0.35} and up to height {@code 0.165m}) collides with this ball. If so, kicks the ball down the
   * line away from the bumper. The robot arm does not affect this collision.
   */
  public void checkBumperContact(double robotX, double robotVelX) {
    if (state == State.HELD || Math.abs(robotVelX) < 0.05) {
      return;
    }
    double bumperHalfLength = 0.35;
    double bumperTopY = 0.165;
    // Only applies when the ball is low enough to be hit by the bumper
    if (y - BALL_RADIUS_METERS > bumperTopY) {
      return;
    }
    double minSeparation = bumperHalfLength + BALL_RADIUS_METERS;
    double dx = x - robotX;
    if (Math.abs(dx) < minSeparation) {
      double dir = (dx >= 0.0) ? 1.0 : -1.0;
      if (Math.abs(dx) < 1e-4 && Math.abs(robotVelX) > 1e-3) {
        dir = Math.signum(robotVelX);
      }
      // Push ball outside the bumper perimeter and kick it down the line
      x = robotX + dir * (minSeparation + 0.01);
      double kickSpeed = Math.max(1.2, Math.abs(robotVelX) * 1.35 + 0.6);
      vx = dir * kickSpeed;
      vy = Math.max(vy, 0.35);
      state = State.FALLING;
      scored = false;
      scoredInBasket = false;
      readyToGrab = false;
    }
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
