package first.robot.subsystems.extender;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;

/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Extender implements Mechanism {

  public Extender() {
  }

  public Command out() {
    return run(coroutine -> {
      while (true) {
        // Add code here to move the extender out.
        coroutine.yield();
      }
    }).named("Extender Out");
  }

  public Command in() {
    return run(coroutine -> {
      while (true) {
        // Add code here to move the extender in.
        coroutine.yield();
      }
    }).named("Extender In");
  }

}
