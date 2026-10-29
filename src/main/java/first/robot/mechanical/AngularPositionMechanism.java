package first.robot.mechanical;

import static org.wpilib.units.Units.Radians;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Angle;

/**
 * Reusable generic mechanism representing a single-degree-of-freedom angular position axis
 * controlled via closed-loop PID.
 *
 * <p>Encapsulates:
 * <ul>
 *   <li>Closed-loop PID calculation and voltage clamping.</li>
 *   <li>Physical angular travel boundary clamping (min / max angle).</li>
 *   <li>Safe setpoint initialization and PID controller reset.</li>
 *   <li>Arrival detection within tolerance.</li>
 *   <li>Continuous default {@link #hold()}.</li>
 * </ul>
 */
public class AngularPositionMechanism implements Mechanism {
  private final String name;
  private final Consumer<Double> setVoltage;
  private final Supplier<Angle> getAngle;
  private final DoubleSupplier getVelocity;

  private final PIDController pid;
  private final Angle minAngle;
  private final Angle maxAngle;
  private final Angle startingAngle;
  private final Angle tolerance;
  private final double maxVolts;

  private Angle setpoint;

  /** True while the mechanism is within tolerance of its target setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public AngularPositionMechanism(
      String name,
      Consumer<Double> setVoltage,
      Supplier<Angle> getAngle,
      DoubleSupplier getVelocity,
      double kP,
      double kD,
      double maxVolts,
      Angle tolerance,
      Angle minAngle,
      Angle maxAngle,
      Angle startingAngle) {
    this.name = name;
    this.setVoltage = setVoltage;
    this.getAngle = getAngle;
    this.getVelocity = getVelocity;
    this.maxVolts = maxVolts;
    this.tolerance = tolerance;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.startingAngle = startingAngle;
    this.setpoint = startingAngle;

    this.pid = new PIDController(kP, 0.0, kD);
  }

  /**
   * Clamps a requested angle within the physical angular travel limits.
   *
   * @param angle raw target angle
   * @return clamped angle within [minAngle, maxAngle]
   */
  public Angle clampAngle(Angle angle) {
    if (angle.lt(minAngle)) {
      return minAngle;
    } else if (angle.gt(maxAngle)) {
      return maxAngle;
    } else {
      return angle;
    }
  }

  /**
   * Calculates closed-loop voltage: {@code volts = PID(angle, setpoint)},
   * clamped between {@code [-maxVolts, +maxVolts]}.
   */
  public void driveTowardSetpoint() {
    driveTowardSetpoint(maxVolts);
  }

  /**
   * Calculates closed-loop voltage with a custom maximum voltage clamp.
   *
   * @param voltageLimit maximum allowed voltage magnitude
   */
  public void driveTowardSetpoint(double voltageLimit) {
    if (RobotBase.isDisabled()) {
      setVoltage.accept(0.0);
      return;
    }
    double currentRad = getAngle().in(Radians);
    double targetRad = setpoint.in(Radians);
    double volts = pid.calculate(currentRad, targetRad);
    setVoltage.accept(Math.clamp(volts, -voltageLimit, voltageLimit));
  }

  /**
   * Returns true if current angle is within {@link #tolerance} of {@link #setpoint}.
   */
  public boolean atSetpoint() {
    return getAngle().isNear(setpoint, tolerance);
  }

  /**
   * Continuous default holding command. Maintains the current setpoint, syncing setpoint
   * to actual angle when disabled to prevent jump on re-enable.
   *
   * @return default holding command running at LOWEST_PRIORITY
   */
  public Command hold() {
    StateMachine sm = new StateMachine(name + " Hold [FSM]");
    var holding = sm.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getAngle();
          }
          driveTowardSetpoint();
        }).named(name + " Hold"));
    holding.onEnter(() -> {
      if (RobotBase.isDisabled()) {
        setpoint = getAngle();
      }
    });
    sm.setInitialState(holding);
    return run(coroutine -> coroutine.await(sm))
        .withPriority(Command.LOWEST_PRIORITY)
        .named(name + " Hold");
  }

  public Angle getAngle() {
    return getAngle.get();
  }

  public Angle getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(Angle setpoint) {
    this.setpoint = clampAngle(setpoint);
  }

  public void resetSetpoint() {
    setpoint = startingAngle;
    pid.reset();
  }

  public void resetPid() {
    pid.reset();
  }

  public PIDController getPid() {
    return pid;
  }
}
