package first.robot.subsystems.extender;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import first.robot.Schematic;
import first.robot.mechanical.Mechanical;

import static org.wpilib.units.Units.Meters;

import java.awt.event.KeyEvent;

/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Extender implements Mechanism {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(15, 0, 1);

  public Extender() 
  {
    this.motor = Mechanical.createSparkMax(Schematic.EXTENDER_SPARK_MAX_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    setDefaultCommand(hold()); 
  }

  public Command moveToSetpoint(double targetPosition) {
    return run(coroutine -> {
      pid.reset();
      pid.setSetpoint(targetPosition);

      // while (true) 
      // {
      //   double currentPosition = encoder.getPosition().get();
      //   double newVoltage = pid.calculate(currentPosition);

      //   motor.setVoltage(newVoltage);
      //   coroutine.yield();
      // }

      double currentPosition;
      while ((currentPosition = encoder.getPosition().get()) != targetPosition) 
      {
        // double currentPosition = encoder.getPosition().get();
        double newVoltage = pid.calculate(currentPosition);

        motor.setVoltage(newVoltage);
        coroutine.yield();
      }
    }).named("Extender Move To Setpoint");
  }

  public Command out() {
    return run(coroutine -> {
      while (encoder.getPosition().get() < 1.10) {
        motor.setVoltage(4.0); // Add code here to move the extender out.
        coroutine.yield();
      }
      motor.setVoltage(0);
    }).named("Extender Out");
  }

  public Command in() {
    return run(coroutine -> {
      while (encoder.getPosition().get() > 0.01) {
        motor.setVoltage(-4.0); // Add code here to move the extender in.
        coroutine.yield();
      }
      motor.setVoltage(0);
    }).named("Extender In");
  }

  public Command hold() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(0.0); // Add code here to stop the extender from moving.
        coroutine.yield();
      }
    }).named("Extender Hold");
  }
}
