package first.robot.auto;

import first.robot.Robot;
import first.robot.orchestration.BallCommands;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Autonomous routine: drives to the table and scores Ball 1 into the floor target zone.
 *
 * <p>Uses Command3 coroutines ({@code coroutine.await(...)}) to sequentially drive the robot
 * across the field to the table and then orchestrate the arm mechanisms to pick up and place the ball.
 */
@Autonomous(name = "Score Ball")
public class ScoreBallAuto implements OpMode {
  private final Command auto;

  public ScoreBallAuto(Robot robot) {
    auto = driveAndScoreBall(robot.drive, robot.elevator, robot.extender, robot.claw);
  }

  /**
   * Drives to the table and scores Ball 1 into the floor target zone.
   *
   * @param drive Drivetrain mechanism to drive to the table.
   * @param elevator Elevator mechanism to lift the arm.
   * @param extender Extender mechanism to position the claw over the ball and target.
   * @param claw Claw mechanism to grip and release the ball.
   * @return A command orchestrating the full drive and score sequence.
   */
  public static Command driveAndScoreBall(
      DriveMechanism drive,
      ElevatorMechanism elevator,
      ExtenderMechanism extender,
      ClawMechanism claw) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Drive from field start position (x = 0.0m) to table (x = 2.0m)
      coroutine.await(drive.driveToTable());

      // Step 2: Grab Ball 1 from the table surface
      coroutine.await(BallCommands.grabBallFromTable(elevator, extender, claw));

      // Step 3: Place it in the green floor target zone
      coroutine.await(BallCommands.scoreBallOnGround(elevator, extender, claw));
    }).named("Drive And Score Ball");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(auto);
  }
}
