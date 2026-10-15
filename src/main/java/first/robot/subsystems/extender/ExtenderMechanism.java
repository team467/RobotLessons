package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;

import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;

/** The extender slides the claw in and out on a pulley (the x-axis). */
public class ExtenderMechanism implements Mechanism {
  private final ExtenderIO io;
  private final ExtenderIO.ExtenderInputs inputs = new ExtenderIO.ExtenderInputs();
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = STARTING_EXTENSION;

  /** True while the extender is within tolerance of its setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public ExtenderMechanism(ExtenderIO io) {
    this.io = io;
    setDefaultCommand(hold());
  }

  /** Call once per robot loop, before the scheduler runs, to refresh sensor readings. */
  public void periodic() {
    io.updateInputs(inputs);
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Moves the extender to a position and finishes once it gets there.
   *
   * @param extensionMeters how far to extend, clamped to the slide's travel
   */
  public Command moveTo(double extensionMeters) {
    double target = Math.clamp(extensionMeters, MIN_EXTENSION, MAX_EXTENSION);
    return run(coroutine -> {
          setpoint = target;
          while (!atSetpoint()) {
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named(String.format("Extender To %.2fm", target));
  }

  /** Extends outward smoothly while held, stopping at MAX_EXTENSION. */
  public Command out() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.min(MAX_EXTENSION, setpoint + MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Extender Out");
  }

  /** Retracts inward smoothly while held, stopping at MIN_EXTENSION. */
  public Command in() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.max(MIN_EXTENSION, setpoint - MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Extender In");
  }

  /** Holds the current position. This is the default command. */
  public Command hold() {
    return run(coroutine -> {
          if (Math.abs(getPosition() - setpoint) > 0.08) {
            setpoint = getPosition();
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
        .named("Extender Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    double volts = pid.calculate(getPosition(), setpoint);
    io.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  /** The name of the most specific command currently using the extender. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_EXTENSION;
  }
}
