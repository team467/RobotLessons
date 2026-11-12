package first.robot.subsystems.extender;

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
public class Extender implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private Distance setpoint = START_POSITION;

  public Extender() {
    this.motor = Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    //setDefaultCommand(hold());  
  }

  

  public Command out() {
    return run(coroutine -> {
      while (true) {
        motor.setVoltage(12);
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
  

}
