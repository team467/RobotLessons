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

  @Test
  void testBumperHitsFloorBallAndKicksItDownTheLine() {
    BallSim floorBall = new BallSim(1.0, BALL_RADIUS, BallSim.State.ON_FLOOR);
    assertEquals(1.0, floorBall.getX(), 1e-4);

    // Robot bumper is far away (robotX = 0.0 -> front bumper at 0.35), even if arm is extended to 1.0
    floorBall.checkBumperContact(0.0, 0.0);
    floorBall.update(0.02, 1.0, BALL_RADIUS + GRIP_DEPTH, 0.0);
    assertEquals(1.0, floorBall.getX(), 1e-4, "Arm alone without bumper contact should not move the ball");

    // Robot drives forward into the ball so front bumper (robotX + 0.35) hits ball at x = 1.0 (robotX = 0.60, vel = 2.0 m/s)
    floorBall.checkBumperContact(0.60, 2.0);
    for (int i = 0; i < 40; i++) {
      floorBall.update(0.02, 0.0, 0.5, 0.0);
    }

    assertTrue(
        floorBall.getX() > 1.35,
        "Ball hit by the bumper should be kicked forward down the line (actual x=" + floorBall.getX() + ")");
  }

  @Test
  void testOrangeBallBouncesAwayIfTableOccupied() {
    BallSim orangeBall =
        new BallSim(
            1.0,
            BALL_RADIUS,
            BallSim.State.ON_FLOOR,
            TABLE_LEFT_X,
            TABLE_RIGHT_X,
            TABLE_HEIGHT,
            TABLE_TARGET_LEFT_X,
            TABLE_TARGET_RIGHT_X,
            TABLE_HEIGHT);

    // Grab the orange ball from the floor
    orangeBall.update(0.02, 1.0, BALL_RADIUS + GRIP_DEPTH, BALL_CONTACT_ANGLE + 0.1);
    orangeBall.update(0.02, 1.0, BALL_RADIUS + GRIP_DEPTH, BALL_CONTACT_ANGLE);
    assertTrue(orangeBall.isHeld());

    // Carry it over the table target at gentle score height while the green ball is still on the table
    orangeBall.setTableOccupied(true);
    double wristX = TABLE_TARGET_CENTER_X;
    double wristY = TABLE_HEIGHT + TARGET_BOX_WALL_HEIGHT + GRIP_DEPTH + 0.05;
    orangeBall.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);
    orangeBall.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE);

    // Release the claw
    orangeBall.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + BallSim.RELEASE_MARGIN + 0.05);
    assertEquals(BallSim.State.FALLING, orangeBall.getState());

    for (int i = 0; i < 120; i++) {
      orangeBall.update(0.02, wristX, wristY, BALL_CONTACT_ANGLE + 0.2);
    }

    assertFalse(
        orangeBall.isScored(),
        "Orange ball must bounce away and not score if placed on the table while green ball is still on it");
    assertNotEquals(
        BallSim.State.ON_TABLE,
        orangeBall.getState(),
        "Orange ball should have bounced off the occupied table");

    // Reset and verify placing the orange ball onto the green floor target also bounces away
    orangeBall.reset();
    orangeBall.setFloorTargetBounds(TARGET_LEFT_X, TARGET_RIGHT_X);
    orangeBall.update(0.02, 1.0, BALL_RADIUS + GRIP_DEPTH, BALL_CONTACT_ANGLE + 0.1);
    orangeBall.update(0.02, 1.0, BALL_RADIUS + GRIP_DEPTH, BALL_CONTACT_ANGLE);
    assertTrue(orangeBall.isHeld());

    double floorTargetWristX = TARGET_CENTER_X;
    double floorScoreWristY = 0.45;
    orangeBall.update(0.02, floorTargetWristX, floorScoreWristY, BALL_CONTACT_ANGLE);
    orangeBall.update(0.02, floorTargetWristX, floorScoreWristY, BALL_CONTACT_ANGLE);
    orangeBall.update(0.02, floorTargetWristX, floorScoreWristY, BALL_CONTACT_ANGLE + BallSim.RELEASE_MARGIN + 0.05);

    for (int i = 0; i < 120; i++) {
      orangeBall.update(0.02, floorTargetWristX, floorScoreWristY, BALL_CONTACT_ANGLE + 0.2);
    }

    assertTrue(
        orangeBall.getX() > TARGET_RIGHT_X,
        "Orange ball placed onto the green floor target should bounce away past the target");
  }
}
