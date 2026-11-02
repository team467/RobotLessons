package first.robot.mechanical;

import static first.robot.subsystems.claw.ClawConstants.CLOSED_ANGLE;
import static first.robot.subsystems.claw.ClawConstants.KD;
import static first.robot.subsystems.claw.ClawConstants.KP;
import static first.robot.subsystems.claw.ClawConstants.MAX_VOLTS;
import static first.robot.subsystems.claw.ClawConstants.OPEN_ANGLE;
import static first.robot.subsystems.claw.ClawConstants.STARTING_ANGLE;
import static first.robot.subsystems.claw.ClawConstants.TOLERANCE;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.units.measure.Angle;

/**
 * Angular position mechanism tailored for the claw, preconfigured with claw
 * PID gains, travel limits, and voltage limits.
 */
public class ClawMechanism extends AngularPositionMechanism {
  public ClawMechanism(
      Consumer<Double> setVoltage,
      Supplier<Angle> getAngle,
      DoubleSupplier getVelocity) {
    super(
        "Claw",
        setVoltage,
        getAngle,
        getVelocity,
        KP,
        KD,
        MAX_VOLTS,
        TOLERANCE,
        CLOSED_ANGLE,
        OPEN_ANGLE,
        STARTING_ANGLE);
  }
}
