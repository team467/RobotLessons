package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;
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

/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Extender extends LinearPositionMechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public Extender() {
    super(
        "Extender",
        KP,
        KD,
        MAX_VOLTS,
        TOLERANCE,
        MIN_EXTENSION,
        MAX_EXTENSION,
        STARTING_EXTENSION);

    this.motor = Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();

    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command moveTo(double extensionMeters) {
    return moveTo(Meters.of(extensionMeters));
  }

  public Command out() {
    return jog("Extender Out", MANUAL_SPEED_MPS * 0.02);
  }

  public Command in() {
    return jog("Extender In", -MANUAL_SPEED_MPS * 0.02);
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
      return STARTING_EXTENSION;
    }
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_EXTENSION;
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
