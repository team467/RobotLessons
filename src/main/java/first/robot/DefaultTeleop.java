package first.robot;

import static first.robot.constants.DriverConstants.*;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Driver control. Each action works from both the gamepad and the simulator keyboard.
 *
 * <ul>
 *   <li>Up / Down (D-pad or arrow keys): hold to raise / lower the elevator
 *   <li>Right / Left (D-pad or arrow keys): hold to extend / retract the arm
 *   <li>Right bumper or Z: open the claw
 *   <li>Left bumper or X: close the claw
 *   <li>D / A (or Y / A face buttons): hold to drive forward / backward
 * </ul>
 *
 * <p>When you let go of a movement key, the command is canceled and the mechanism's default
 * {@code hold()} command takes over, keeping it where it stopped.
 */
@Teleop(name = "Teleop")
public class DefaultTeleop implements OpMode {
  public DefaultTeleop(Robot robot) {
    var pad = robot.controller;
    var keys = robot.keyboard;

    // Elevator jogging:
    // whileTrue runs the upward/downward jogging command while either gamepad D-pad or keyboard
    // arrow key is held. On release, the command cancels and the elevator's hold() command preserves position.
    pad.dpadUp().or(keys.key(KEY_ELEVATOR_UP)).whileTrue(robot.elevator.up());
    pad.dpadDown().or(keys.key(KEY_ELEVATOR_DOWN)).whileTrue(robot.elevator.down());

    // Extender jogging:
    // Extends/retracts the horizontal slide while the key/button is held.
    pad.dpadRight().or(keys.key(KEY_EXTENDER_OUT)).whileTrue(robot.extender.out());
    pad.dpadLeft().or(keys.key(KEY_EXTENDER_IN)).whileTrue(robot.extender.in());

    // Claw pinchers:
    // onTrue triggers a single-shot command that drives to full open (60 deg) or full grip.
    pad.rightBumper().or(keys.key(KEY_CLAW_OPEN)).onTrue(robot.claw.open());
    pad.leftBumper().or(keys.key(KEY_CLAW_CLOSE)).onTrue(robot.claw.close());

    // Drivetrain translation:
    // Drives forward (towards table) or backward (towards starting position) at teleop speed (5.5 m/s).
    pad.faceUp().or(keys.key(KEY_DRIVE_FORWARD)).whileTrue(robot.drive.forward());
    pad.faceDown().or(keys.key(KEY_DRIVE_BACKWARD)).whileTrue(robot.drive.backward());
  }
}
