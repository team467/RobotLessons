package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import first.robot.mechanisms.ElevatorMechanism;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Distance;

/** The elevator moves the whole arm up and down (the y-axis). */
public class Elevator {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final ElevatorMechanism elevatorMechanism;

  /** True while the elevator is within tolerance of its setpoint. */
  public final Trigger atSetpoint;

  public Elevator() {
    this.motor = Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();

    this.elevatorMechanism = new ElevatorMechanism(motor::setVoltage, this::getPosition, this::getVelocity);

    this.atSetpoint = elevatorMechanism.atSetpoint;
    // When no other command is using the elevator, hold it where it is.
    elevatorMechanism.setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command moveTo(Distance height) {
    return elevatorMechanism.moveTo(height);
  }

  public Command moveTo(double heightMeters) {
    return moveTo(Meters.of(heightMeters));
  }

  public Command up() {
    return elevatorMechanism.jog("Elevator Up", MANUAL_SPEED_MPS * 0.02);
  }

  public Command down() {
    return elevatorMechanism.jog("Elevator Down", -MANUAL_SPEED_MPS * 0.02);
  }

  public Command hold() {
    return elevatorMechanism.hold();
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean atSetpoint() {
    return elevatorMechanism.atSetpoint();
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_HEIGHT;
    }
    return Meters.of(pos);
  }

  public Distance getSetpoint() {
    return elevatorMechanism.getSetpoint();
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    if (val == null) {
      return 0.0;
    }
    return val * 12.0;
  }

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    if (vel == null) {
      return 0.0;
    }
    return vel;
  }

  /** The name of the most specific command currently using the elevator. */
  public String getCurrentCommandName() {
    List<Command> running = elevatorMechanism.getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    elevatorMechanism.resetSetpoint();
  }
}
