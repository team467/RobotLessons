package first.robot.sim;

import static first.robot.constants.FieldConstants.*;
import static first.robot.constants.RobotConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class BallSimTest {
  private BallSim ball;

  @BeforeEach
  void setUp() {
    ball = new BallSim();
  }

  @Test
  void testInitialStateOnTable() {
    assertEquals(BallSim.State.ON_TABLE, ball.getState());
    assertEquals(BALL_START_X, ball.getX(), 1e-4);
    assertEquals(BALL_START_Y, ball.getY(), 1e-4);
    assertFalse(ball.isHeld());
    assertFalse(ball.isScored());
  }

  @Test
  void testBallGrabWhenClawClosesAtPosition() {
    double gripX = BALL_START_X;
    double gripY = BALL_START_Y;
    double wristX = gripX;
    double wristY = gripY + GRIP_DEPTH;

    // Approaching with open claw (angle > BALL_CONTACT_ANGLE)
    double stop = ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + 0.1);
    assertTrue(Double.isNaN(stop));
    assertFalse(ball.isHeld());

    // Claw closes onto the ball (crossing contact angle)
    stop = ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);
    assertEquals(BALL_CONTACT_ANGLE, stop, 1e-4);
    assertTrue(ball.isHeld());

    // Ball should move along with wrist
    ball.update(0.02, wristX + 0.05, wristY + 0.10, BALL_CONTACT_ANGLE);
    assertEquals(gripX + 0.05, ball.getX(), 1e-4);
    assertEquals(gripY + 0.10, ball.getY(), 1e-4);
  }

  @Test
  void testBallReleaseAndFallToTargetScore() {
    // Manually place claw at target location above floor at gentle drop height
    double targetGripX = TARGET_CENTER_X;
    double targetGripY = 0.35; // gentle drop height (SCORE_HEIGHT 0.45 - GRIP_DEPTH 0.10)
    double wristX = targetGripX;
    double wristY = targetGripY + GRIP_DEPTH;

    // Approaching and grabbing at table first
    ball.update(0.02, BALL_START_X, BALL_START_Y + GRIP_DEPTH, BALL_CONTACT_ANGLE + 0.1);
    ball.update(0.02, BALL_START_X, BALL_START_Y + GRIP_DEPTH, BALL_CONTACT_ANGLE);
    assertTrue(ball.isHeld());

    // Transport held ball over target zone and let it come to a stop
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);
    assertEquals(targetGripX, ball.getX(), 1e-4);
    assertEquals(targetGripY, ball.getY(), 1e-4);

    // Release claw by opening past margin
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + BallSim.RELEASE_MARGIN + 0.05);
    assertEquals(BallSim.State.FALLING, ball.getState());
    assertFalse(ball.isHeld());

    // Step physics until ball hits the floor and settles
    for (int i = 0; i < 50; i++) {
      ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + 0.2);
      if (ball.getState() == BallSim.State.ON_FLOOR) {
        break;
      }
    }

    assertEquals(BallSim.State.ON_FLOOR, ball.getState());
    assertEquals(BALL_RADIUS, ball.getY(), 1e-4);
    assertTrue(ball.isScored(), "Gentle drop in target zone should register as scored");
  }

  @Test
  void testHighDropBouncesAwayAndFailsToScore() {
    double targetGripX = TARGET_CENTER_X;
    double highGripY = 1.25; // high drop (ABOVE_TABLE_HEIGHT 1.35 - GRIP_DEPTH 0.10)
    double wristX = targetGripX;
    double wristY = highGripY + GRIP_DEPTH;

    ball.update(0.02, BALL_START_X, BALL_START_Y + GRIP_DEPTH, BALL_CONTACT_ANGLE + 0.1);
    ball.update(0.02, BALL_START_X, BALL_START_Y + GRIP_DEPTH, BALL_CONTACT_ANGLE);
    assertTrue(ball.isHeld());

    // Move to high position above target
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);

    // Release from high position
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + BallSim.RELEASE_MARGIN + 0.05);
    assertEquals(BallSim.State.FALLING, ball.getState());

    // Step physics through bounces until it settles
    for (int i = 0; i < 200; i++) {
      ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + 0.2);
    }

    assertFalse(ball.isScored(), "High drop should bounce away and NOT score");
    assertTrue(ball.getX() > TARGET_RIGHT_X, "Ball should have bounced horizontally past target");
  }

  @Test
  void testResetBall() {
    ball.update(0.02, BALL_START_X, BALL_START_Y + GRIP_DEPTH, BALL_CONTACT_ANGLE + 0.1);
    ball.update(0.02, BALL_START_X, BALL_START_Y + GRIP_DEPTH, BALL_CONTACT_ANGLE);
    assertTrue(ball.isHeld());

    ball.reset();
    assertEquals(BallSim.State.ON_TABLE, ball.getState());
    assertEquals(BALL_START_X, ball.getX(), 1e-4);
    assertEquals(BALL_START_Y, ball.getY(), 1e-4);
    assertFalse(ball.isHeld());
  }

  @Test
  void testClosedClawMovingOverBallDoesNotGrab() {
    double gripX = BALL_START_X;
    double gripY = BALL_START_Y;
    double wristX = gripX;
    double wristY = gripY + GRIP_DEPTH;

    // Moving down over the ball while claw is closed (0.0 rad)
    for (int i = 0; i < 20; i++) {
      double stop = ball.update(0.02, wristX, wristY, 0.0);
      assertTrue(Double.isNaN(stop), "Closed claw should not have grip stop angle applied");
      assertFalse(ball.isHeld(), "Closed claw must not grab the ball");
      assertEquals(BallSim.State.ON_TABLE, ball.getState());
    }

    // Now open the claw while in position
    ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + 0.1);
    assertFalse(ball.isHeld(), "Opening claw over ball should not grab yet");

    // Now close the claw onto the ball
    double stop = ball.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);
    assertEquals(BALL_CONTACT_ANGLE, stop, 1e-4);
    assertTrue(ball.isHeld(), "Closing claw onto ball after opening must grab it");
  }
}
