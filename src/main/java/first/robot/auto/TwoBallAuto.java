package first.robot.auto;

import first.robot.Robot;
import first.robot.orchestration.BallCommands;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.extender.ExtenderConstants;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Two-ball autonomous opmode:
 * <ol>
 *   <li>Picks up Ball 2 (Orange) from the floor near the starting zone.</li>
 *   <li>Stows the arm and drives across the field to the table.</li>
 *   <li>Places Ball 2 on the ground to the left of the green floor target.</li>
 *   <li>Grabs Ball 1 (Green) off the table.</li>
 *   <li>Scores Ball 1 into the green floor target zone.</li>
 *   <li>Picks up Ball 2 (Orange) from the ground to the left of the green target.</li>
 *   <li>Places Ball 2 into the orange target box on top of the table.</li>
 * </ol>
 */
@Autonomous(name = "Two Ball Routine")
public class TwoBallAuto implements OpMode {
  private final Command auto;

  public TwoBallAuto(Robot robot) {
    auto = twoBallRoutine(robot.drive, robot.elevator, robot.extender, robot.claw);
  }

  /**
   * Constructs the complete two-ball autonomous command routine.
   *
   * @param drive Drivetrain subsystem for field driving.
   * @param elevator Elevator subsystem for vertical positioning.
   * @param extender Extender subsystem for horizontal arm extension.
   * @param claw Claw subsystem for ball grip and release.
   * @return A command executing the full two-ball sequence via coroutines.
   */
  public static Command twoBallRoutine(
      Drive drive,
      Elevator elevator,
      Extender extender,
      Claw claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Pick up Ball 2 (Orange) resting on the ground in front of the robot
      coroutine.await(BallCommands.grabBallFromGround(elevator, extender, claw));

      // Step 2: Retract horizontal arm, stow elevator, and drive to the table in parallel
      coroutine.awaitAll(
          extender.moveTo(ExtenderConstants.MIN_EXTENSION),
          elevator.moveTo(ElevatorConstants.STOW_HEIGHT),
          drive.driveToTable());

      // Step 3: Place Ball 2 on the ground to the left of the green target
      coroutine.await(BallCommands.placeBallLeftOfTarget(elevator, extender, claw));

      // Step 4: Grab Ball 1 (Green) from the table
      coroutine.await(BallCommands.grabBallFromTable(elevator, extender, claw));

      // Step 5: Place Ball 1 in green floor target zone
      coroutine.await(BallCommands.scoreBallOnGround(elevator, extender, claw));

      // Step 6: Pick up Ball 2 (Orange) from the ground to the left of the green target
      coroutine.await(BallCommands.grabBallLeftOfTarget(elevator, extender, claw));

      // Step 7: Score Ball 2 gently into the orange target box on top of the table
      coroutine.await(BallCommands.scoreBallOnTable(elevator, extender, claw));
    }).named("Two Ball Routine");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(auto);
  }
}
