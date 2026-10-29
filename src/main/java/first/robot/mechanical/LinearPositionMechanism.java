package first.robot.mechanical;

import static org.wpilib.units.Units.Meters;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

/**
 * Reusable generic mechanism representing a single-degree-of-freedom linear position axis
 * controlled via closed-loop PID and optional feedforward (e.g. gravity compensation).
 *
 * <p>Encapsulates:
 * <ul>
 *   <li>Closed-loop PID calculation and voltage clamping.</li>
 *   <li>Optional feedforward voltage (such as {@code kG} for gravity compensation).</li>
 *   <li>Physical travel boundary clamping (min / max distance).</li>
 *   <li>Safe setpoint initialization and PID controller reset.</li>
 *   <li>Settled arrival detection combining position tolerance with velocity settling.</li>
 *   <li>Point-to-point {@link #moveTo(Mechanism, Distance)} state machines and continuous default {@link #hold(Mechanism)}.</li>
 * </ul>
 */
public class LinearPositionMechanism implements Mechanism {
  private final String name;
  private final Consumer<Double> setVoltage;
  private final Supplier<Distance> getPosition;
  private final DoubleSupplier getVelocity;

  private final PIDController pid;
  private final double kG;
  private final Distance minPosition;
  private final Distance maxPosition;
  private final Distance startingPosition;
  private final Distance tolerance;
  private final double maxVolts;

  private Distance setpoint;

  /** True while the mechanism is within tolerance of its target setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  /**
   * Constructs a new LinearPositionMechanism without gravity feedforward (kG = 0.0).
   */
  public LinearPositionMechanism(
      String name,
      Consumer<Double> setVoltage,
      Supplier<Distance> getPosition,
      DoubleSupplier getVelocity,
      double kP,
      double kD,
      double maxVolts,
      Distance tolerance,
      Distance minPosition,
      Distance maxPosition,
      Distance startingPosition) {
    this(
        name,
        setVoltage,
        getPosition,
        getVelocity,
        kP,
        kD,
        0.0,
        maxVolts,
        tolerance,
        minPosition,
        maxPosition,
        startingPosition);
  }

  /**
   * Constructs a new LinearPositionMechanism with gravity feedforward (kG).
   */
  public LinearPositionMechanism(
      String name,
      Consumer<Double> setVoltage,
      Supplier<Distance> getPosition,
      DoubleSupplier getVelocity,
      double kP,
      double kD,
      double kG,
      double maxVolts,
      Distance tolerance,
      Distance minPosition,
      Distance maxPosition,
      Distance startingPosition) {
    this.name = name;
    this.setVoltage = setVoltage;
    this.getPosition = getPosition;
    this.getVelocity = getVelocity;
    this.kG = kG;
    this.maxVolts = maxVolts;
    this.tolerance = tolerance;
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
    this.startingPosition = startingPosition;
    this.setpoint = startingPosition;

    this.pid = new PIDController(kP, 0.0, kD);
  }

  /**
   * Clamps a requested position within the physical travel limits of the mechanism.
   *
   * @param pos raw target distance
   * @return clamped distance within [minPosition, maxPosition]
   */
  public Distance clampPosition(Distance pos) {
    if (pos.lt(minPosition)) {
      return minPosition;
    } else if (pos.gt(maxPosition)) {
      return maxPosition;
    } else {
      return pos;
    }
  }

  /**
   * Calculates closed-loop voltage: {@code volts = kG + PID(position, setpoint)},
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
    double currentMeters = getPosition().in(Meters);
    double targetMeters = setpoint.in(Meters);
    double volts = kG + pid.calculate(currentMeters, targetMeters);
    setVoltage.accept(Math.clamp(volts, -voltageLimit, voltageLimit));
  }

  /**
   * Returns true if current position is within {@link #tolerance} of {@link #setpoint}.
   */
  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, tolerance);
  }

  /**
   * Returns true if current position is within {@link #tolerance} and linear speed is &le; 0.05 m/s.
   */
  public boolean isSettledAtSetpoint() {
    return atSetpoint() && Math.abs(getVelocity.getAsDouble()) <= 0.05;
  }

  /**
   * Moves the mechanism to a target position and completes once arrived and settled.
   *
   * @param parent the subsystem requiring this command
   * @param targetPosition raw target distance (will be clamped)
   * @return command driving to the target
   */
  public Command moveTo(Mechanism parent, Distance targetPosition) {
    Distance target = clampPosition(targetPosition);
    setpoint = target;
    String cmdName = String.format("%s To %.2fm", name, target.in(Meters));
    StateMachine sm = new StateMachine(cmdName);
    var moving = sm.addState(parent.runRepeatedly(this::driveTowardSetpoint).named(cmdName));
    moving.onEnter(() -> {
      setpoint = target;
      pid.reset();
    });
    sm.setInitialState(moving);
    moving.exitStateMachine().when(this::isSettledAtSetpoint);
    return parent.run(coroutine -> coroutine.await(sm)).named(cmdName);
  }

  /**
   * Moves the mechanism to a target position with a custom command display name.
   */
  public Command moveTo(Mechanism parent, Distance targetPosition, String cmdName) {
    Distance target = clampPosition(targetPosition);
    setpoint = target;
    StateMachine sm = new StateMachine(cmdName);
    var moving = sm.addState(parent.runRepeatedly(this::driveTowardSetpoint).named(cmdName));
    moving.onEnter(() -> {
      setpoint = target;
      pid.reset();
    });
    sm.setInitialState(moving);
    moving.exitStateMachine().when(this::isSettledAtSetpoint);
    return parent.run(coroutine -> coroutine.await(sm)).named(cmdName);
  }

  /**
   * Incremental jogging command that steps setpoint by {@code deltaMetersPerTick} each 20ms cycle.
   * On exit, latches setpoint to current actual position and resets the PID controller.
   *
   * @param parent the subsystem requiring this command
   * @param cmdName command name
   * @param deltaMetersPerTick change in setpoint per 20ms tick
   * @return jogging command
   */
  public Command jog(Mechanism parent, String cmdName, double deltaMetersPerTick) {
    StateMachine sm = new StateMachine(cmdName);
    var moving = sm.addState(
        parent.runRepeatedly(() -> {
          double nextMeters = setpoint.in(Meters) + deltaMetersPerTick;
          setpoint = clampPosition(Meters.of(nextMeters));
          driveTowardSetpoint();
        }).named(cmdName));
    moving.onEnter(() -> setpoint = getPosition());
    moving.onExit(() -> {
      setpoint = getPosition();
      pid.reset();
    });
    sm.setInitialState(moving);
    return parent.run(coroutine -> coroutine.await(sm)).named(cmdName);
  }

  /**
   * Continuous default holding command requiring the parent mechanism.
   *
   * @param parent the subsystem requiring this command
   * @return default holding command running at LOWEST_PRIORITY
   */
  public Command hold(Mechanism parent) {
    StateMachine sm = new StateMachine(name + " Hold [FSM]");
    var holding = sm.addState(
        parent.runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          driveTowardSetpoint();
        }).named(name + " Hold"));
    holding.onEnter(() -> {
      if (RobotBase.isDisabled()) {
        setpoint = getPosition();
      }
    });
    sm.setInitialState(holding);
    return parent.run(coroutine -> coroutine.await(sm))
        .withPriority(Command.LOWEST_PRIORITY)
        .named(name + " Hold");
  }

  public Distance getPosition() {
    return getPosition.get();
  }

  public Distance getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(Distance setpoint) {
    this.setpoint = clampPosition(setpoint);
  }

  public void resetSetpoint() {
    setpoint = startingPosition;
    pid.reset();
  }

  public void resetPid() {
    pid.reset();
  }

  public PIDController getPid() {
    return pid;
  }
}
