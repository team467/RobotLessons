package first.robot.mechanisms.shooter;

import static first.robot.mechanisms.shooter.ShooterConstants.*;

import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;

/**
 * Spring-loaded shooter subsystem mounted on the front of the robot.
 *
 * <p>A pullback motor spins to pull the spring-loaded plunger back while a mechanical ratchet locks
 * it in place. Releasing the solenoid disengages the ratchet to kick a ball held close to the robot
 * in an upward arc toward the elevated basket.
 */
public class ShooterMechanism implements Mechanism {
  private final ShooterIO io;
  private final ShooterIO.ShooterInputs inputs = new ShooterIO.ShooterInputs();
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = STARTING_PULLBACK;

  /** True when the shooter spring is pulled back and locked by the ratchet, ready to fire. */
  public final Trigger isLoaded = new Trigger(this::isLoaded);

  /** True when the plunger is within tolerance of its target pullback position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public ShooterMechanism() {
    this(new ShooterIOSim());
  }

  public ShooterMechanism(ShooterIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(() -> io.updateInputs(inputs));
    io.updateInputs(inputs);
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Spins the pullback motor to wind the spring back to {@link ShooterConstants#LOADED_PULLBACK}
   * with the ratchet engaged, then stops the motor so the ratchet holds it cocked.
   */
  public Command pullBack() {
    return run(coroutine -> {
          setpoint = LOADED_PULLBACK;
          io.setSolenoidReleased(false);
          while (!isLoaded()) {
            double volts = Math.max(PULLBACK_VOLTS, pid.calculate(getPosition(), setpoint));
            io.setMotorVoltage(Math.clamp(volts, 0.0, MAX_VOLTS));
            coroutine.yield();
          }
          io.setMotorVoltage(0.0);
        })
        .named("Shooter Pull Back");
  }

  /**
   * Releases the solenoid to disengage the ratchet and kick the ball into the air, then re-engages
   * the ratchet once the spring has fully extended.
   */
  public Command kick() {
    return run(coroutine -> {
          setpoint = MIN_PULLBACK;
          io.setMotorVoltage(0.0);
          io.setSolenoidReleased(true);
          while (getPosition() > MIN_PULLBACK + TOLERANCE_METERS) {
            io.setMotorVoltage(0.0);
            io.setSolenoidReleased(true);
            coroutine.yield();
          }
          // Brief follow-through before re-latching the solenoid
          for (int i = 0; i < 3; i++) {
            coroutine.yield();
          }
          io.setSolenoidReleased(false);
        })
        .named("Shooter Kick");
  }

  /**
   * Manual command that winds the pullback motor while held (will stall at {@link
   * ShooterConstants#MAX_PULLBACK} if held past the limit).
   */
  public Command wind() {
    return run(coroutine -> {
          io.setSolenoidReleased(false);
          while (true) {
            setpoint = MAX_PULLBACK;
            io.setMotorVoltage(PULLBACK_VOLTS);
            coroutine.yield();
          }
        })
        .named("Shooter Wind");
  }

  /** Manual command that holds the release solenoid open while active. */
  public Command releaseSolenoid() {
    return run(coroutine -> {
          while (true) {
            setpoint = MIN_PULLBACK;
            io.setMotorVoltage(0.0);
            io.setSolenoidReleased(true);
            coroutine.yield();
          }
        })
        .named("Shooter Release");
  }

  /**
   * Default command: keeps the pullback motor idle (0 V) and the ratchet locked so any cocked
   * spring tension is mechanically held in place without drawing motor current.
   */
  public Command hold() {
    return run(coroutine -> {
          while (true) {
            if (RobotBase.isDisabled()) {
              setpoint = getPosition();
            }
            io.setMotorVoltage(0.0);
            io.setSolenoidReleased(false);
            coroutine.yield();
          }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Shooter Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean isLoaded() {
    return inputs.isLoaded;
  }

  public boolean isRatchetLocked() {
    return inputs.ratchetLocked;
  }

  public boolean isSolenoidReleased() {
    return inputs.solenoidReleased;
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    return inputs.pullbackMeters;
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_PULLBACK;
  }
}
