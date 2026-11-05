package first.robot.mechanisms;

import static org.wpilib.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

/**
 * Reusable generic mechanism representing a spring-loaded, ratchet-latched mechanism.
 *
 * <p>Encapsulates:
 * <ul>
 *   <li>Pullback winch motor control with heavy spring feedforward.</li>
 *   <li>One-way ratchet locking where holding draws 0V.</li>
 *   <li>Solenoid release status and loaded/setpoint triggers.</li>
 * </ul>
 */
public class LatchedSpringMechanism implements Mechanism {
  private final String name;
  private final Consumer<Double> setMotorVoltage;
  private final Consumer<Boolean> setSolenoid;
  private final BooleanSupplier isSolenoidReleased;
  private final Supplier<Distance> getPosition;
  private final PIDController pid;
  private final double maxVolts;
  private final double pullbackVolts;
  private final Distance tolerance;
  private final Distance minPullback;
  private final Distance maxPullback;
  private final Distance loadedPullback;
  private final Distance startingPullback;

  private Distance setpoint;

  /** True when the spring is pulled back and locked by the latch/ratchet, ready to fire. */
  public final Trigger isLoaded = new Trigger(this::isLoaded);

  /** True when the mechanism is within tolerance of its target pullback position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public LatchedSpringMechanism(
      String name,
      Consumer<Double> setMotorVoltage,
      Consumer<Boolean> setSolenoid,
      BooleanSupplier isSolenoidReleased,
      Supplier<Distance> getPosition,
      DoubleSupplier getVelocity,
      double kP,
      double kD,
      double maxVolts,
      double pullbackVolts,
      Distance tolerance,
      Distance minPullback,
      Distance maxPullback,
      Distance loadedPullback,
      Distance startingPullback) {
    this.name = name;
    this.setMotorVoltage = setMotorVoltage;
    this.setSolenoid = setSolenoid;
    this.isSolenoidReleased = isSolenoidReleased;
    this.getPosition = getPosition;
    this.maxVolts = maxVolts;
    this.pullbackVolts = pullbackVolts;
    this.tolerance = tolerance;
    this.minPullback = minPullback;
    this.maxPullback = maxPullback;
    this.loadedPullback = loadedPullback;
    this.startingPullback = startingPullback;
    this.setpoint = startingPullback;

    this.pid = new PIDController(kP, 0.0, kD);
  }

  /**
   * Drives the pullback winch motor toward the current setpoint with latch engaged.
   */
  public void driveTowardSetpoint() {
    if (RobotBase.isDisabled()) {
      setMotorVoltage.accept(0.0);
      return;
    }
    setSolenoid.accept(false);
    double volts = Math.max(pullbackVolts, pid.calculate(getPosition().in(Meters), setpoint.in(Meters)));
    setMotorVoltage.accept(Math.clamp(volts, 0.0, maxVolts));
  }

  public Distance clampPullback(Distance pullback) {
    if (pullback.lt(minPullback)) {
      return minPullback;
    } else if (pullback.gt(maxPullback)) {
      return maxPullback;
    } else {
      return pullback;
    }
  }

  public boolean isLoaded() {
    return getPosition().gte(loadedPullback.minus(tolerance));
  }

  public boolean isRatchetLocked() {
    return !isSolenoidReleased.getAsBoolean();
  }

  public boolean isSolenoidReleased() {
    return isSolenoidReleased.getAsBoolean();
  }

  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, tolerance);
  }

  public Distance getPosition() {
    return getPosition.get();
  }

  public Distance getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(Distance setpoint) {
    this.setpoint = clampPullback(setpoint);
  }

  public void resetSetpoint() {
    setpoint = startingPullback;
    pid.reset();
  }
}
