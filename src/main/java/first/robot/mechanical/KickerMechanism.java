package first.robot.mechanical;

import static first.robot.subsystems.kicker.KickerConstants.KD;
import static first.robot.subsystems.kicker.KickerConstants.KP;
import static first.robot.subsystems.kicker.KickerConstants.LOADED_PULLBACK;
import static first.robot.subsystems.kicker.KickerConstants.MAX_PULLBACK;
import static first.robot.subsystems.kicker.KickerConstants.MAX_VOLTS;
import static first.robot.subsystems.kicker.KickerConstants.MIN_PULLBACK;
import static first.robot.subsystems.kicker.KickerConstants.PULLBACK_VOLTS;
import static first.robot.subsystems.kicker.KickerConstants.STARTING_PULLBACK;
import static first.robot.subsystems.kicker.KickerConstants.TOLERANCE;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.units.measure.Distance;

/**
 * Spring-loaded kicker mechanism tailored for the kicker subsystem, preconfigured with
 * pullback winch PID gains, travel limits, pullback voltages, and ratchet timing.
 */
public class KickerMechanism extends SpringKickerMechanism {
  public KickerMechanism(
      Consumer<Double> setMotorVoltage,
      Consumer<Boolean> setSolenoid,
      BooleanSupplier isSolenoidReleased,
      Supplier<Distance> getPosition,
      DoubleSupplier getVelocity) {
    super(
        "Kicker",
        setMotorVoltage,
        setSolenoid,
        isSolenoidReleased,
        getPosition,
        getVelocity,
        KP,
        KD,
        MAX_VOLTS,
        PULLBACK_VOLTS,
        TOLERANCE,
        MIN_PULLBACK,
        MAX_PULLBACK,
        LOADED_PULLBACK,
        STARTING_PULLBACK);
  }
}
