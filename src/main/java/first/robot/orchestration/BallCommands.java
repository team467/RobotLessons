package first.robot.orchestration;

import static org.wpilib.units.Units.Meters;

import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.extender.ExtenderConstants;
import first.robot.subsystems.kicker.Kicker;
import org.wpilib.command3.Command;

/**
 * Reusable multi-subsystem command orchestration sequences for manipulating game balls.
 *
 * <p>These compositions use {@link Command#noRequirements} with coroutines ({@code coroutine.await(...)})
 * so that child commands dynamically lock only the subsystems they need while running, allowing
 * default holding commands to run when individual subsystems are idle.
 */
public final class BallCommands {
  private BallCommands() {}

  /**
   * Raises the elevator above the table and opens the claw in parallel, extends over Ball 1,
   * lowers onto the ball, and closes the claw fingers around it.
   *
   * <p>Subsystem motion breakdown:
   * <ol>
   *   <li>Concurrent prep: Elevates to {@code ABOVE_TABLE_HEIGHT} (1.15m) to clear the table lip while
   *       simultaneously opening the claw fingers to 60 deg. Running these in parallel saves cycle time.
   *   <li>Horizontal extension: Extends the arm to {@code BALL_EXTENSION} (~0.76m) so the open claw is
   *       positioned directly above Ball 1 resting on the table.
   *   <li>Vertical descent: Lowers the elevator to {@code TABLE_GRAB_HEIGHT} (~0.79m), placing the open
   *       pinchers around the ball circumference without jamming into the tabletop.
   *   <li>Grip acquisition: Closes the claw fingers until the optical ball sensor triggers and the
   *       mechanism switches into active gripping mode.
   * </ol>
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @return a sequential coroutine command orchestrating the full table grab
   */
  public static Command grabBallFromTable(
      Elevator elevator, Extender extender, Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Concurrently raise elevator above table obstacle and open claw fingers.
      // Running both motions simultaneously minimizes total cycle time.
      coroutine.awaitAll(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT), claw.open());

      // Step 2: Extend slide outward directly above the ball on the table.
      // Must occur while elevator is above table height to prevent hitting the table edge.
      coroutine.await(extender.moveTo(ExtenderConstants.BALL_EXTENSION));

      // Step 3: Lower elevator onto the ball.
      // The claw fingers are open, so they straddle the ball without pushing it away.
      coroutine.await(elevator.moveTo(ElevatorConstants.TABLE_GRAB_HEIGHT));

      // Step 4: Close claw fingers to grip the ball.
      // Finishes as soon as the optical sensor confirms ball contact or fingers fully close.
      coroutine.await(claw.close());
    }).named("Grab Ball From Table");
  }

  /**
   * Scores a held ball on the ground: lifts it over the obstacle, extends over the green
   * floor target, lowers close to the floor, and gently releases the ball to prevent bouncing.
   *
   * <p>Assumes the robot is already holding a ball. Call {@link #grabBallFromTable} prior to
   * this command if the ball is still on the table.
   *
   * <p>Subsystem motion breakdown:
   * <ol>
   *   <li>Clearance: Raises the elevator to {@code ABOVE_TABLE_HEIGHT} to avoid collision.
   *   <li>Target reach: Extends to {@code TARGET_EXTENSION} (~0.58m), centering over the floor target.
   *   <li>Gently lower: Lowers to {@code SCORE_HEIGHT} (0.45m), just above floor level.
   *   <li>Release: Opens the claw fingers so the ball drops smoothly into the scoring zone without bounce.
   * </ol>
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @return a sequential coroutine command executing the floor score sequence
   */
  public static Command scoreBallOnGround(
      Elevator elevator, Extender extender, Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Ensure arm and held ball clear the table surface before horizontal motion
      coroutine.await(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));

      // Step 2: Extend arm so claw centers directly over the green floor target zone
      coroutine.await(extender.moveTo(ExtenderConstants.TARGET_EXTENSION));

      // Step 3: Descend close to the floor (0.45m) so the release drop distance is minimal.
      // Dropping from high above causes balls to bounce out of the scoring area.
      coroutine.await(elevator.moveTo(ElevatorConstants.SCORE_HEIGHT));

      // Step 4: Open claw fingers to deposit the ball gently onto the target
      coroutine.await(claw.open());
    }).named("Score Ball On Ground");
  }

  /**
   * Picks up Ball 2 from the ground near the starting zone:
   * pre-aligns claw open, extension out, and elevator at stow height in parallel,
   * lowers to ground grab height, closes claw, and returns to stow height.
   *
   * <p>Subsystem motion breakdown:
   * <ol>
   *   <li>Parallel prep: Opens claw, extends to {@code BALL_EXTENSION}, and holds elevator at stow height.
   *   <li>Ground drop: Lowers elevator to {@code GROUND_GRAB_HEIGHT} (~0.29m) to straddle the ground ball.
   *   <li>Grip: Closes claw fingers to grasp the ball.
   *   <li>Stow lift: Raises elevator back to {@code STOW_HEIGHT} (0.50m) to safely transport the ball.
   * </ol>
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @return a sequential coroutine command executing the ground pickup sequence
   */
  public static Command grabBallFromGround(
      Elevator elevator, Extender extender, Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Concurrently open claw, extend to ball radius, and position elevator at travel stow height.
      // All three subsystems move concurrently to prepare the grab in minimal time.
      coroutine.awaitAll(
          claw.open(),
          extender.moveTo(ExtenderConstants.BALL_EXTENSION),
          elevator.moveTo(ElevatorConstants.STOW_HEIGHT));

      // Step 2: Lower down to ground grab level (claw fingers surround ball on carpet)
      coroutine.await(elevator.moveTo(ElevatorConstants.GROUND_GRAB_HEIGHT));

      // Step 3: Close claw to secure the ball (finishes when optical beam is broken)
      coroutine.await(claw.close());

      // Step 4: Lift back up to safe travel stow height (0.50m) so ball doesn't drag during chassis translation
      coroutine.await(elevator.moveTo(ElevatorConstants.STOW_HEIGHT));
    }).named("Grab Ball From Ground");
  }

  /**
   * Places a held ball gently on the ground to the left of the green floor target ({@code extender
   * = 0.0m}) while the robot is parked at the table.
   *
   * <p>Used in multi-ball routines to stage a ball on the floor before picking up and scoring
   * another ball, preventing collisions and target area overcrowding.
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @return a sequential command that safely stages the ball left of target
   */
  public static Command placeBallLeftOfTarget(
      Elevator elevator, Extender extender, Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Safety check: if currently extended over the table and below safe table height,
      // raise first to prevent the arm or ball from colliding with the table edge during retraction.
      if (extender.getPosition().gt(Meters.of(0.45))
          && elevator.getPosition().lt(ElevatorConstants.ABOVE_TABLE_HEIGHT.minus(Meters.of(0.05)))) {
        coroutine.await(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
      }

      // Retract arm fully inwards (0.0m) to position over the staging area left of the target
      coroutine.await(extender.moveTo(Meters.of(0.0)));

      // Lower down close to the floor to prevent bouncing upon release
      coroutine.await(elevator.moveTo(ElevatorConstants.SCORE_HEIGHT));

      // Release ball onto the floor
      coroutine.await(claw.open());
    }).named("Place Ball Left Of Target");
  }

  /**
   * Picks up a ball resting on the ground to the left of the green floor target ({@code extender =
   * 0.0m}) while the robot is parked at the table.
   *
   * <p>Pairs with {@link #placeBallLeftOfTarget} to retrieve the staged ball during complex
   * autonomous routines.
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @return a command sequence retrieving the staged ball from the floor
   */
  public static Command grabBallLeftOfTarget(
      Elevator elevator, Extender extender, Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Concurrently open claw, fully retract arm (0.0m), and set elevator to stow height
      coroutine.awaitAll(
          claw.open(),
          extender.moveTo(Meters.of(0.0)),
          elevator.moveTo(ElevatorConstants.STOW_HEIGHT));

      // Step 2: Lower to ground level directly over the staged ball
      coroutine.await(elevator.moveTo(ElevatorConstants.GROUND_GRAB_HEIGHT));

      // Step 3: Close claw fingers to grip the staged ball
      coroutine.await(claw.close());

      // Step 4: Lift back up to stow height for subsequent manipulation
      coroutine.await(elevator.moveTo(ElevatorConstants.STOW_HEIGHT));
    }).named("Grab Ball Left Of Target");
  }

  /**
   * Places a held ball gently into the orange table target box:
   * lifts over table height, extends over the target box center, lowers to box rim height,
   * and opens the claw.
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @return a command sequence scoring the ball into the table target box
   */
  public static Command scoreTableBall(
      Elevator elevator, Extender extender, Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Lift elevator above table height to clear the box outer perimeter walls
      coroutine.await(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));

      // Step 2: Extend horizontally to center over the table target box
      coroutine.await(extender.moveTo(ExtenderConstants.TABLE_TARGET_EXTENSION));

      // Step 3: Descend slightly to table box rim height so the ball deposits cleanly inside
      coroutine.await(elevator.moveTo(ElevatorConstants.SCORE_TABLE_HEIGHT));

      // Step 4: Open claw fingers to release the ball into the box
      coroutine.await(claw.open());
    }).named("Score Ball On Table");
  }

  /** Places a held ball gently into the table target box (alias). */
  public static Command scoreBallOnTable(
      Elevator elevator, Extender extender, Claw claw) {
    return scoreTableBall(elevator, extender, claw);
  }

  /**
   * Moves the arm (elevator and extender) to the shooting position ({@code elevator = 0.60m,
   * extender = 0.23m}) where a held ball sits directly in front of the spring-loaded kicker.
   *
   * <p>If the arm is currently extended over the table, it raises above the table first before
   * retracting to avoid colliding with table obstacles.
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @return a command that safely positions the arm into the kicker strike zone
   */
  public static Command moveToShootPosition(
      Elevator elevator, Extender extender) {
    return Command.noRequirements(
            coroutine -> {
              // Safety collision check: if extended beyond 0.45m and below table clearance height,
              // elevate over the table before retracting inwards
              if (extender.getPosition().gt(Meters.of(0.45))
                  && elevator.getPosition().lt(ElevatorConstants.ABOVE_TABLE_HEIGHT.minus(Meters.of(0.05)))) {
                coroutine.await(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
              }

              // Retract arm into kicker strike window (0.23m extension)
              coroutine.await(extender.moveTo(ExtenderConstants.SHOOT_EXTENSION));

              // Position elevator at kicker strike height (0.60m elevation)
              coroutine.await(elevator.moveTo(ElevatorConstants.SHOOT_HEIGHT));
            })
        .named("Move To Shoot Position");
  }

  /**
   * Positions a held ball close to the front of the robot in front of the spring-loaded kicker,
   * pulls the kicker back against its ratchet, and releases the solenoid to kick the ball into
   * the elevated basket.
   *
   * <p>Subsystem motion breakdown:
   * <ol>
   *   <li>Parallel alignment & winch: Simultaneously positions the arm into the kicker strike zone
   *       while winding the kicker motor to pull the plunger back against the one-way ratchet.
   *   <li>Release kick: Energizes the solenoid to trip the ratchet release pawl, causing the heavy
   *       spring to launch the plunger forward into the held ball and propel it toward the basket.
   * </ol>
   *
   * @param elevator the vertical elevator mechanism
   * @param extender the horizontal arm extension mechanism
   * @param claw the claw pincher mechanism
   * @param kicker the spring-loaded kicker subsystem
   * @return a command sequence launching the ball into the basket
   */
  public static Command kickBallIntoBasket(
      Elevator elevator,
      Extender extender,
      Claw claw,
      Kicker kicker) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Concurrently position arm in strike zone and cock the spring-loaded kicker
      coroutine.awaitAll(
          moveToShootPosition(elevator, extender),
          kicker.pullBack());

      // Step 2: Release the solenoid to disengage ratchet pawl and launch the ball toward the basket
      coroutine.await(kicker.kick());
    }).named("Kick Ball Into Basket");
  }

  /** Alias for {@link #kickBallIntoBasket}. */
  public static Command shootBallIntoBasket(
      Elevator elevator,
      Extender extender,
      Claw claw,
      Kicker kicker) {
    return kickBallIntoBasket(elevator, extender, claw, kicker);
  }
}
