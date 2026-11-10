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
  private final SparkMax motor =
      Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private Distance setpoint = STARTING_EXTENSION;

  public Extender() {
    setDefaultCommand(hold());
  }

  public Command moveTo(Distance extension) {
    setpoint = extension;
    return run(coroutine -> {
          setpoint = extension;
          pid.reset();
          while (!atSetpoint()) {
            driveTowardSetpoint();
            coroutine.yield();
          }
          driveTowardSetpoint();
        })
        .named(String.format("Extender To %.2fm", extension.in(Meters)));
  }

  public Command moveTo(double extensionMeters) {
    return moveTo(Meters.of(extensionMeters));
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
          if (!atSetpoint()) {
            setpoint = getPosition();
            pid.reset();
          }
          while (true) {
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Extender Hold");
  }

  private void driveTowardSetpoint() {
    double volts = pid.calculate(getPosition().in(Meters), setpoint.in(Meters));
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, TOLERANCE);
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    return Meters.of(pos != null ? pos : STARTING_EXTENSION.in(Meters));
  }

  public Distance getSetpoint() {
    return setpoint;
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }
}
