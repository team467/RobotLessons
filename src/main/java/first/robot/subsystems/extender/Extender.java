package first.robot.subsystems.extender;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;

import com.revrobotics.spark.SparkMax;

import first.robot.Schematic;
import first.robot.mechanical.Mechanical;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Extender implements Mechanism {

  public Extender() {
    setDefaultCommand(hold());
  }

  private final SparkMax motor = Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

    public Command out() {
        return run(coroutine -> {
            while (encoder.getPosition().get() < 1.08) {
                motor.setVoltage(6.0);
                coroutine.yield();
            }
        }).named("Extender Out");
    }

    public Command in() {
        return run(coroutine -> {
            while (encoder.getPosition().get() > 0.1) {
                motor.setVoltage(-6.0);
                coroutine.yield();
            }
        }).named("Extender In");
    }
    
  public Command hold() {
    return run(coroutine -> {
        while (true) {
            motor.setVoltage(0.0);
            coroutine.yield();
        }
    }).named("Extender Hold");
  }
}
