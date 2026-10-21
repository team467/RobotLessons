package first.robot.auto;

import first.robot.Robot;
import first.robot.orchestration.BallCommands;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveMechanism;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderMechanism;
import first.robot.subsystems.shooter.ShooterConstants;
import first.robot.subsystems.shooter.ShooterMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Autonomous opmode that shoots both balls into the elevated basket to the right of the table:
 * <ol>
 *   <li>Drives to the table while winding the spring-loaded shooter back against its ratchet.</li>
 *   <li>Grabs Ball 1 (Green) off the table.</li>
 *   <li>Retracts the arm close to the robot and releases the solenoid to kick Ball 1 into the basket.</li>
 *   <li>Drives back to the starting zone while re-cocking the shooter in parallel.</li>
 *   <li>Picks up Ball 2 (Orange) from the floor.</li>
 *   <li>Retracts the arm and drives back to the table in parallel.</li>
 *   <li>Releases the solenoid to kick Ball 2 into the basket.</li>
 * </ol>
 */
@Autonomous(name = "Shoot Both Balls")
public class ShootBothBallsAuto implements OpMode {
  private final Command auto;

  public ShootBothBallsAuto(Robot robot) {
    auto =
        shootBothBallsRoutine(
            robot.drive, robot.elevator, robot.extender, robot.claw, robot.shooter);
  }

  /**
   * Constructs the two-ball basket shooting autonomous routine.
   *
   * @param drive Drivetrain mechanism for field translation.
   * @param elevator Elevator mechanism for vertical positioning.
   * @param extender Extender mechanism for horizontal arm extension.
   * @param claw Claw mechanism for ball grip and release.
   * @param shooter Spring-loaded shooter mechanism for kicking balls into the basket.
   * @return A command executing the full two-ball basket shooting sequence.
   */
  public static Command shootBothBallsRoutine(
      DriveMechanism drive,
      ElevatorMechanism elevator,
      ExtenderMechanism extender,
      ClawMechanism claw,
      ShooterMechanism shooter) {
    return Command.noRequirements(coroutine -> {
      // Step 1: Drive to the table and pull back the shooter in parallel
      coroutine.awaitAll(drive.driveToTable(), shooter.pullBack());

      // Step 2: Grab Ball 1 from the table
      coroutine.await(BallCommands.grabBallFromTable(elevator, extender, claw));

      // Step 3: Bring Ball 1 close to the robot and kick it into the basket
      coroutine.await(BallCommands.shootBallIntoBasket(elevator, extender, claw, shooter));

      // Step 4: Drive back to the start location while re-cocking the shooter in parallel
      coroutine.awaitAll(drive.driveToStart(), shooter.pullBack());

      // Step 5: Pick up Ball 2 (Orange) from the ground
      coroutine.await(BallCommands.grabBallFromGround(elevator, extender, claw));

      // Step 6: Retract the arm close to the robot while driving back to the table
      coroutine.awaitAll(
          extender.moveTo(ShooterConstants.SHOOT_EXTENSION),
          drive.driveToTable());

      // Step 7: Kick Ball 2 into the basket
      coroutine.await(BallCommands.shootBallIntoBasket(elevator, extender, claw, shooter));
    }).named("Shoot Both Balls");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(auto);
  }
}
