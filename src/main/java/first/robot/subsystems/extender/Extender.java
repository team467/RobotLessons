package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;
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

/** The extender slides the claw in and out on a pulley (the x-axis). */
public class Extender implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final LinearPositionMechanism mechanism;

  /** True while the extender is within tolerance of its setpoint. */
  public final Trigger atSetpoint;

  public Extender() {
    this(Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless));
  }

  public Extender(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);

    this.mechanism =
        new LinearPositionMechanism(
            "Extender",
            motor::setVoltage,
            this::getPosition,
            this::getVelocity,
            KP,
            KD,
            MAX_VOLTS,
            TOLERANCE,
            MIN_EXTENSION,
            MAX_EXTENSION,
            STARTING_EXTENSION);

    this.atSetpoint = mechanism.atSetpoint;
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command moveTo(Distance extension) {
    return mechanism.moveTo(this, extension);
  }

  public Command moveTo(double extensionMeters) {
    return moveTo(Meters.of(extensionMeters));
  }

  public Command out() {
    return mechanism.jog(this, "Extender Out", MANUAL_SPEED_MPS * 0.02);
  }

  public Command in() {
    return mechanism.jog(this, "Extender In", -MANUAL_SPEED_MPS * 0.02);
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
      return STARTING_EXTENSION;
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

  /** The name of the most specific command currently using the extender. */
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
