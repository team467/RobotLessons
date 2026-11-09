package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.lib.mechanism.LinearPositionMechanism;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import org.wpilib.command3.Command;
import org.wpilib.epilogue.Logged;
import org.wpilib.units.measure.Distance;

/** The elevator moves the whole arm up and down (the y-axis). */
@Logged
public class Elevator extends LinearPositionMechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public Elevator() {
    super(
        "Elevator",
        KP,
        KD,
        KG,
        MAX_VOLTS,
        TOLERANCE,
        MIN_HEIGHT,
        MAX_HEIGHT,
        STARTING_HEIGHT);

    this.motor = Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();

    // When no other command is using the elevator, hold it where it is.
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command moveTo(double heightMeters) {
    return moveTo(Meters.of(heightMeters));
  }

  public Command up() {
    return jog("Elevator Up", MANUAL_SPEED_MPS * 0.02);
  }

  public Command down() {
    return jog("Elevator Down", -MANUAL_SPEED_MPS * 0.02);
  }

  // ------------------------------------------------------------------------------------------
  // Actuator and sensor overrides
  // ------------------------------------------------------------------------------------------

  @Override
  protected void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public Distance getPosition() {
    if (encoder == null) {
      return STARTING_HEIGHT;
    }
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_HEIGHT;
    }
    return Meters.of(pos);
  }

  @Override
  public double getVelocity() {
    if (encoder == null) {
      return 0.0;
    }
    Double vel = encoder.getVelocity().get();
    if (vel == null) {
      return 0.0;
    }
    return vel;
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    if (val == null) {
      return 0.0;
    }
    return val * 12.0;
  }

  public Distance getSetpoint() {
    return super.getSetpoint();
  }

  public String getCurrentCommandName() {
    return super.getCurrentCommandName();
  }
}
