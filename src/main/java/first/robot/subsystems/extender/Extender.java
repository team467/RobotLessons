package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

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
        motor.setVoltage(5.0);
        coroutine.yield();
      }
    })
        .named("Extender Out");
  }

  public Command in() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(-5.0);
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
