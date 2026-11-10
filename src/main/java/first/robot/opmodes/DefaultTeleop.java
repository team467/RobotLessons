package first.robot.opmodes;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import java.awt.event.KeyEvent;

/**
 * Driver control. Each action works from both the gamepad and the simulator keyboard.
 *
 * <ul>
 *   <li>Up / Down (D-pad or arrow keys): hold to raise / lower the elevator
 *   <li>Right / Left (D-pad or arrow keys): hold to extend / retract the arm
 *   <li>Right bumper or Z: open the claw
 *   <li>Left bumper or X: close the claw
 *   <li>D / A (or Y / A face buttons): hold to drive forward / backward
 *   <li>C / Space (or X / B face buttons): pull back kicker / release solenoid to kick
 *   <li>V: move the arm (elevator and extender) to the kicking position
 * </ul>
 *
 * <p>When you let go of a movement key, the command is canceled and the mechanism's default
 * {@code hold()} command takes over, keeping it where it stopped.
 */
@Teleop(name = "Teleop")
public class DefaultTeleop implements OpMode {
  public DefaultTeleop(Robot robot) {
    var driverController = robot.driverController;
    var keys = robot.keyboard;

    // Extender jogging:
    // Extends/retracts the horizontal slide while the key/button is held.
    driverController.dpadRight().or(keys.key(KeyEvent.VK_RIGHT)).whileTrue(robot.extender.out());
    driverController.dpadLeft().or(keys.key(KeyEvent.VK_LEFT)).whileTrue(robot.extender.in());

  }
}
