package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.LinearPositionMechanism;
import first.robot.mechanical.Mechanical;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Distance;

/** The elevator moves the whole arm up and down (the y-axis). */
public class Elevator implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final LinearPositionMechanism mechanism;

  /** True while the elevator is within tolerance of its setpoint. */
  public final Trigger atSetpoint;

  public Elevator() {
    this(Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless));
  }

  public Elevator(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);

    this.mechanism =
        new LinearPositionMechanism(
            "Elevator",
            motor::setVoltage,
            this::getPosition,
            this::getVelocity,
            KP,
            KD,
            KG,
            MAX_VOLTS,
            TOLERANCE,
            MIN_HEIGHT,
            MAX_HEIGHT,
            STARTING_HEIGHT);

    this.atSetpoint = mechanism.atSetpoint;
    // When no other command is using the elevator, hold it where it is.
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command moveTo(Distance height) {
    return mechanism.moveTo(this, height);
  }

  public Command moveTo(double heightMeters) {
    return moveTo(Meters.of(heightMeters));
  }

  public Command up() {
    return mechanism.jog(this, "Elevator Up", MANUAL_SPEED_MPS * 0.02);
  }

  public Command down() {
    return mechanism.jog(this, "Elevator Down", -MANUAL_SPEED_MPS * 0.02);
  }

  public Command hold() {
    return mechanism.hold(this);
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean atSetpoint() {
    return mechanism.atSetpoint();
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_HEIGHT;
    }
    return Meters.of(pos);
  }

  public Distance getSetpoint() {
    return mechanism.getSetpoint();
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
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    mechanism.resetSetpoint();
  }

  public LinearPositionMechanism getMechanism() {
    return mechanism;
  }
}
