package first.robot.opmodes;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import java.awt.event.KeyEvent;

/**
 * Driver control.
 * 
 * This class binds Controller and Keyboard presses to commands.
 */
@Teleop(name = "Teleop")
public class DefaultTeleop implements OpMode {
  public DefaultTeleop(Robot robot) {
    var keys = robot.keyboard;

    // Drive the robot forward and backward with the D and A keys.
    keys.key(KeyEvent.VK_D).whileTrue(robot.drive.forward());
    keys.key(KeyEvent.VK_A).whileTrue(robot.drive.backward());
  }
}
