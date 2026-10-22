package first.robot.auto;

import first.robot.Robot;
import first.robot.orchestration.BallCommands;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.mechanisms.elevator.ElevatorConstants;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Two-ball autonomous opmode:
 * <ol>
 *   <li>Drives across the field to the table.</li>
 *   <li>Grabs Ball 1 (Green) off the table.</li>
 *   <li>Scores Ball 1 into the floor target zone.</li>
 *   <li>Stows the elevator and arm in parallel ({@code coroutine.awaitAll}).</li>
 *   <li>Drives back across the field to the starting zone.</li>
 *   <li>Picks up Ball 2 (Orange) from the floor.</li>
 *   <li>Drives back to the table carrying Ball 2.</li>
 *   <li>Places Ball 2 into the target box on top of the table.</li>
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
   * @param drive Drivetrain mechanism for field driving.
   * @param elevator Elevator mechanism for vertical positioning.
   * @param extender Extender mechanism for horizontal arm extension.
   * @param claw Claw mechanism for ball grip and release.
   * @return A command executing the full two-ball sequence via coroutines.
   */
  public static Command twoBallRoutine(
      DriveMechanism drive,
      ElevatorMechanism elevator,
      ExtenderMechanism extender,
      ClawMechanism claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Drive to table
      coroutine.await(drive.driveToTable());

      // Step 2: Grab Ball 1 from table
      coroutine.await(BallCommands.grabBallFromTable(elevator, extender, claw));

      // Step 3: Place Ball 1 in green floor target zone
      coroutine.await(BallCommands.scoreBallOnGround(elevator, extender, claw));

      // Step 4: Retract horizontal arm and stow elevator to safe transit height in parallel
      coroutine.awaitAll(
          extender.moveTo(0.0),
          elevator.moveTo(ElevatorConstants.STOW_HEIGHT));

      // Step 5: Drive back across the field to start location
      coroutine.await(drive.driveToStart());

      // Step 6: Pick up Ball 2 (Orange) resting on the ground
      coroutine.await(BallCommands.grabBallFromGround(elevator, extender, claw));

      // Step 7: Drive back to the table carrying Ball 2
      coroutine.await(drive.driveToTable());

      // Step 8: Score Ball 2 gently into the orange target box on top of the table
      coroutine.await(BallCommands.scoreBallOnTable(elevator, extender, claw));
    }).named("Two Ball Routine");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(auto);
  }
}
