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
  private final SparkMax motor = Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless);

  public Extender() {
    setDefaultCommand(hold());
  }

  public Command out() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(6.0);
        coroutine.yield();
      }
    })
        .named("Extender Out");
  }

  public Command in() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(-1.0);
        coroutine.yield();
      }
    })
        .named("Extender In");
  }

  public Command hold() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(0);
        coroutine.yield();
      }
    })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Extender Hold");
  }
}
