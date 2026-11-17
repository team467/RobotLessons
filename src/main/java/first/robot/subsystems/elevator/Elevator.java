package first.robot.subsystems.elevator;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.epilogue.Logged;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;
import static first.robot.subsystems.drive.DriveConstants.*;
import static org.wpilib.units.Units.Meters;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
//import java.util.List;
//import org.wpilib.framework.RobotBase;
//import org.wpilib.command3.Command;


/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Elevator implements Mechanism {

  private final SparkMax motor;

  public Elevator() {
    motor = Mechanical.createSparkMax(Schematic.ELEVATOR_NEO_CAN_ID, MotorType.kBrushless);
    setDefaultCommand(hold());  
  }
  

  

  public Command out() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(12);
        coroutine.yield();
      }
    }).named("Elevator Out");
  }

  public Command hold() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(0);
        coroutine.yield();
      }
    }).named("Hold");
  }


  public Command in() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(-12);
        coroutine.yield();
      }
    }).named("Elevator In");
  }
  

}
