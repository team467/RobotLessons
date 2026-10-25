package first.robot.auto;

import first.robot.Robot;
import first.robot.orchestration.BallCommands;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderMechanism;
import first.robot.mechanisms.shooter.ShooterConstants;
import first.robot.mechanisms.shooter.ShooterMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Autonomous opmode that shoots both balls into the elevated basket to the right of the table:
 * <ol>
 *   <li>Picks up Ball 2 (Orange) from the floor while winding the spring-loaded shooter back against its ratchet.</li>
 *   <li>Retracts the arm close to the robot and drives to the table in parallel.</li>
 *   <li>Releases the solenoid to kick Ball 2 into the basket.</li>
 *   <li>Grabs Ball 1 (Green) off the table while re-cocking the shooter in parallel.</li>
 *   <li>Retracts the arm close to the robot and releases the solenoid to kick Ball 1 into the basket.</li>
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
      // Step 1: Pick up Ball 2 (Orange) from the ground while pulling back the shooter in parallel
      coroutine.awaitAll(
          BallCommands.grabBallFromGround(elevator, extender, claw),
          shooter.pullBack());

      // Step 2: Retract the arm close to the robot while driving to the table
      coroutine.awaitAll(
          extender.moveTo(ShooterConstants.SHOOT_EXTENSION),
          drive.driveToTable());

      // Step 3: Kick Ball 2 into the basket
      coroutine.await(BallCommands.shootBallIntoBasket(elevator, extender, claw, shooter));

      // Step 4: Grab Ball 1 (Green) from the table while re-cocking the shooter in parallel
      coroutine.awaitAll(
          BallCommands.grabBallFromTable(elevator, extender, claw),
          shooter.pullBack());

      // Step 5: Bring Ball 1 close to the robot and kick it into the basket
      coroutine.await(BallCommands.shootBallIntoBasket(elevator, extender, claw, shooter));
    }).named("Shoot Both Balls");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(auto);
  }
}
