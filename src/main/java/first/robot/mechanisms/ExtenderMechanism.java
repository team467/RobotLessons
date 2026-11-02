package first.robot.mechanisms;

import static first.robot.subsystems.extender.ExtenderConstants.KD;
import static first.robot.subsystems.extender.ExtenderConstants.KP;
import static first.robot.subsystems.extender.ExtenderConstants.MAX_EXTENSION;
import static first.robot.subsystems.extender.ExtenderConstants.MAX_VOLTS;
import static first.robot.subsystems.extender.ExtenderConstants.MIN_EXTENSION;
import static first.robot.subsystems.extender.ExtenderConstants.STARTING_EXTENSION;
import static first.robot.subsystems.extender.ExtenderConstants.TOLERANCE;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.units.measure.Distance;

/**
 * Linear position mechanism tailored for the extender, preconfigured with extender
 * PID gains, travel limits, and voltage limits.
 */
public class ExtenderMechanism extends LinearPositionMechanism {
  public ExtenderMechanism(
      Consumer<Double> setVoltage,
      Supplier<Distance> getPosition,
      DoubleSupplier getVelocity) {
    super(
        "Extender",
        setVoltage,
        getPosition,
        getVelocity,
        KP,
        KD,
        MAX_VOLTS,
        TOLERANCE,
        MIN_EXTENSION,
        MAX_EXTENSION,
        STARTING_EXTENSION);
  }
}
