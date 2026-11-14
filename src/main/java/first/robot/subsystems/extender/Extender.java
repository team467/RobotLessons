package first.robot.subsystems.extender;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import first.robot.mechanical.Mechanical;

/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Extender implements Mechanism {
  private final SparkMax motor = Mechanical.createSparkMax(3, MotorType.kBrushless);

  public Extender() {
     setDefaultCommand(hold());
  }

  public Command out() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(12); // Example code to move the extender out.
        coroutine.yield();
      }
    }).named("Extender Out");
  }

  public Command in() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(-12);
        coroutine.yield();
      }
    }).named("Extender In");
  }
  public Command hold() {
    return run(coroutine -> {
      motor.setVoltage(0);
      coroutine.yield();
    }).named("Extender Hold");
  }

}
