package first.robot.orchestration;

import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderConstants;
import first.robot.subsystems.extender.ExtenderMechanism;
import org.wpilib.command3.Command;

/**
 * Reusable multi-mechanism command orchestration sequences for manipulating game balls.
 *
 * <p>These compositions use {@link Command#noRequirements} with coroutines ({@code coroutine.await(...)})
 * so that child commands dynamically lock only the mechanisms they need while running, allowing
 * default holding commands to run when individual mechanisms are idle.
 */
public final class BallCommands {
  private BallCommands() {}

  /**
   * Raises the elevator above the table and opens the claw in parallel, extends over Ball 1,
   * lowers onto the ball, and closes the claw fingers around it.
   */
  public static Command grabBallFromTable(
      ElevatorMechanism elevator, ExtenderMechanism extender, ClawMechanism claw) {
    return Command.noRequirements(coroutine -> {
      // 1. Concurrently raise elevator above table obstacle and open claw fingers
      coroutine.awaitAll(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT), claw.open());
      // 2. Extend slide outward directly above the ball on the table
      coroutine.await(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
      // 3. Lower elevator onto the ball
      coroutine.await(elevator.moveTo(ElevatorConstants.GRAB_HEIGHT));
      // 4. Close claw fingers to grip the ball
      coroutine.await(claw.close());
    }).named("Grab Ball From Table");
  }

  /**
   * Scores a held ball on the ground: lifts it over the obstacle, extends over the green
   * floor target, lowers close to the floor, and gently releases the ball to prevent bouncing.
   *
   * <p>Assumes the robot is already holding a ball. Call {@link #grabBallFromTable} prior to
   * this command if the ball is still on the table.
   */
  public static Command scoreBallOnGround(
      ElevatorMechanism elevator, ExtenderMechanism extender, ClawMechanism claw) {
    return Command.noRequirements(coroutine -> {
      coroutine.await(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
      coroutine.await(extender.moveTo(ExtenderConstants.TARGET_EXTENSION));
      coroutine.await(elevator.moveTo(ElevatorConstants.SCORE_HEIGHT));
      coroutine.await(claw.open());
    }).named("Score Ball On Ground");
  }

  /**
   * Picks up Ball 2 from the ground near the starting zone:
   * pre-aligns claw open, extension out, and elevator at stow height in parallel,
   * lowers to ground grab height, closes claw, and returns to stow height.
   */
  public static Command grabBallFromGround(
      ElevatorMechanism elevator, ExtenderMechanism extender, ClawMechanism claw) {
    return Command.noRequirements(coroutine -> {
      // 1. Concurrently prepare claw open, extend to ball radius, and position elevator
      coroutine.awaitAll(
          claw.open(),
          extender.moveTo(ExtenderConstants.BALL_EXTENSION),
          elevator.moveTo(ElevatorConstants.STOW_HEIGHT));
      // 2. Lower to ground grab level
      coroutine.await(elevator.moveTo(ElevatorConstants.GROUND_GRAB_HEIGHT));
      // 3. Close claw to secure the ball
      coroutine.await(claw.close());
      // 4. Lift back up to safe travel stow height
      coroutine.await(elevator.moveTo(ElevatorConstants.STOW_HEIGHT));
    }).named("Grab Ball From Ground");
  }

  /**
   * Places a held ball gently into the orange table target box:
   * lifts over table height, extends over the target box center, lowers to box rim height,
   * and opens the claw.
   */
  public static Command scoreTableBall(
      ElevatorMechanism elevator, ExtenderMechanism extender, ClawMechanism claw) {
    return Command.noRequirements(coroutine -> {
      coroutine.await(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
      coroutine.await(extender.moveTo(ExtenderConstants.TABLE_TARGET_EXTENSION));
      coroutine.await(elevator.moveTo(ElevatorConstants.SCORE_TABLE_HEIGHT));
      coroutine.await(claw.open());
    }).named("Score Ball On Table");
  }

  /** Places a held ball gently into the table target box (alias). */
  public static Command scoreBallOnTable(
      ElevatorMechanism elevator, ExtenderMechanism extender, ClawMechanism claw) {
    return scoreTableBall(elevator, extender, claw);
  }
}
