package first.lib.mechanism;

import static org.wpilib.units.Units.Meters;

import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
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
 *   <li>Point-to-point {@link #moveTo(Distance)} commands and continuous default {@link #hold()}.</li>
 * </ul>
 */
public abstract class LinearPositionMechanism implements Mechanism {
  private final String name;
  private final PIDController pid;
  private final double kG;
  private final Distance minPosition;
  private final Distance maxPosition;
  private final Distance startingPosition;
  private final Distance tolerance;
  private final double maxVolts;

  private Distance setpoint;

  /**
   * Constructs a new LinearPositionMechanism without gravity feedforward (kG = 0.0).
   */
  public LinearPositionMechanism(
      String name,
      double kP,
      double kD,
      double maxVolts,
      Distance tolerance,
      Distance minPosition,
      Distance maxPosition,
      Distance startingPosition) {
    this(
        name,
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
      double kP,
      double kD,
      double kG,
      double maxVolts,
      Distance tolerance,
      Distance minPosition,
      Distance maxPosition,
      Distance startingPosition) {
    this.name = name;
    this.kG = kG;
    this.maxVolts = maxVolts;
    this.tolerance = tolerance;
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
    this.startingPosition = startingPosition;
    this.setpoint = startingPosition;

    this.pid = new PIDController(kP, 0.0, kD);
  }

  /** Gets the current physical position of the mechanism. */
  public abstract Distance getPosition();

  /** Gets the current physical linear velocity in meters per second. */
  public abstract double getVelocity();

  /** Applies output voltage to the mechanism's motor actuator. */
  protected abstract void setVoltage(double volts);

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
    if (RobotBase.isDisabled()) {
      setVoltage(0.0);
      return;
    }
    double currentMeters = getPosition().in(Meters);
    double targetMeters = setpoint.in(Meters);
    double volts = kG + pid.calculate(currentMeters, targetMeters);
    setVoltage(Math.clamp(volts, -maxVolts, maxVolts));
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
    return atSetpoint() && Math.abs(getVelocity()) <= 0.05;
  }

  /**
   * Moves the mechanism to a target position and completes once arrived and settled.
   *
   * @param targetPosition raw target distance (will be clamped)
   * @return command driving to the target
   */
  public Command moveTo(Distance targetPosition) {
    Distance target = clampPosition(targetPosition);
    String cmdName = String.format("%s To %.2fm", name, target.in(Meters));
    return moveTo(target, cmdName);
  }

  /**
   * Moves the mechanism to a target position with a custom command display name.
   */
  public Command moveTo(Distance targetPosition, String cmdName) {
    Distance target = clampPosition(targetPosition);
    setpoint = target;
    return run(coroutine -> {
      setpoint = target;
      pid.reset();
      while (!isSettledAtSetpoint()) {
        driveTowardSetpoint();
        coroutine.yield();
      }
      driveTowardSetpoint();
    }).named(cmdName);
  }

  /**
   * Incremental jogging command that steps setpoint by {@code deltaMetersPerTick} each 20ms cycle.
   * On exit, latches setpoint to current actual position and resets the PID controller.
   *
   * @param cmdName command name
   * @param deltaMetersPerTick change in setpoint per 20ms tick
   * @return jogging command
   */
  public Command jog(String cmdName, double deltaMetersPerTick) {
    return run(coroutine -> {
      setpoint = getPosition();
      while (true) {
        double nextMeters = setpoint.in(Meters) + deltaMetersPerTick;
        setpoint = clampPosition(Meters.of(nextMeters));
        driveTowardSetpoint();
        coroutine.yield();
      }
    }).whenCanceled(() -> {
      setpoint = getPosition();
      pid.reset();
    }).named(cmdName);
  }

  /**
   * Continuous default holding command.
   *
   * @return default holding command running at LOWEST_PRIORITY
   */
  public Command hold() {
    return run(coroutine -> {
          if (!atSetpoint()) {
            Distance pos = getPosition();
            setpoint = pos.isNear(startingPosition, tolerance) ? startingPosition : pos;
            pid.reset();
          }
          while (true) {
            if (RobotBase.isDisabled()) {
              setpoint = getPosition();
            }
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named(name + " Hold");
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

  /** The name of the most specific command currently using this mechanism. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }
}
