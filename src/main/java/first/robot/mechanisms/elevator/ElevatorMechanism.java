package first.robot.mechanisms.elevator;

import static first.robot.mechanisms.elevator.ElevatorConstants.*;

import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;

/** The elevator moves the whole arm up and down (the y-axis). */
public class ElevatorMechanism implements Mechanism {
  private final ElevatorIO io;
  private final ElevatorIO.ElevatorInputs inputs = new ElevatorIO.ElevatorInputs();
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  /** The height we are currently trying to reach. */
  private double setpoint = STARTING_HEIGHT;

  /** True while the elevator is within tolerance of its setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public ElevatorMechanism(ElevatorIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(io::updateSim);
    Scheduler.getDefault().addPeriodic(() -> io.updateInputs(inputs));
    io.updateInputs(inputs);
    // When no other command is using the elevator, hold it where it is.
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Moves the elevator to a height and finishes once it gets there.
   *
   * @param heightMeters the target wrist height, clamped to the elevator's travel
   */
  public Command moveTo(double heightMeters) {
    double target = Math.clamp(heightMeters, MIN_HEIGHT, MAX_HEIGHT);
    return run(coroutine -> {
          setpoint = target;
          while (!atSetpoint()) {
            driveTowardSetpoint();
            coroutine.yield(); // wait for the next robot loop
          }
        })
        .named(String.format("Elevator To %.2fm", target));
  }

  /** Drives up smoothly while held, stopping at MAX_HEIGHT. */
  public Command up() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.min(MAX_HEIGHT, setpoint + MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Elevator Up");
  }

  /** Drives down smoothly while held, stopping at MIN_HEIGHT. */
  public Command down() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.max(MIN_HEIGHT, setpoint - MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Elevator Down");
  }

  /**
   * Holds the elevator at the current height. This is the default command, so it runs whenever
   * nothing else needs the elevator.
   */
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
        .named("Elevator Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    double volts = KG + pid.calculate(getPosition(), setpoint);
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

  /** The name of the most specific command currently using the elevator. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_HEIGHT;
  }
}
