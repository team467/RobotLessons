package first.robot.subsystems.extender;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
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
        coroutine.yield();
      }
    })
        .named("Extender Out");
  }

  public Command in() {
    return run(coroutine -> {
      while (true) {
        coroutine.yield();
      }
    })
        .named("Extender In");
  }

}
