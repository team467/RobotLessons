package first.robot.subsystems.elevator;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import first.robot.Schematic;
import first.robot.mechanical.Mechanical;

/** The elevator moves the body up and down (the y-axis). */
@Logged
public class Elevator implements Mechanism {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public Elevator() {
    
    this.motor = Mechanical.createSparkMax(Schematic.ELEVATOR_SPARK_MAX_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    setDefaultCommand(hold()); 
  }


  public Command up() {
    return run(coroutine -> {
      while (encoder.getPosition().get() < 1.15) {
        motor.setVoltage(4.0); // Add code here to move the extender out.
        coroutine.yield();
      }
      motor.setVoltage(0);
    }).named("Elevator Up");
  }

  public Command down() {
    return run(coroutine -> {
      while (encoder.getPosition().get() > 0.30) {
        motor.setVoltage(-4.0); // Add code here to move the extender in.
        coroutine.yield();
      }
      motor.setVoltage(0);
    }).named("Elevator Down");
  }

  public Command hold() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(0.0); // Add code here to stop the extender from moving.
        coroutine.yield();
      }
    }).named("Elevator Hold");
  }
}

