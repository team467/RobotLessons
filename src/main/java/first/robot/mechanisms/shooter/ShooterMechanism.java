package first.robot.mechanisms.shooter;

import static first.robot.mechanisms.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.math.controller.PIDController;

/**
 * Spring-loaded shooter subsystem mounted on the front of the robot.
 *
 * <p>A pullback motor spins to pull the spring-loaded plunger back while a mechanical ratchet locks
 * it in place. Releasing the solenoid disengages the ratchet to kick a ball held close to the robot
 * in an upward arc toward the elevated basket.
 */
public class ShooterMechanism implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalOutput solenoid;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = STARTING_PULLBACK;

  private boolean requestPullBack = false;
  private boolean requestKick = false;
  private boolean requestWind = false;
  private boolean requestReleaseSolenoid = false;

  /** True when the shooter spring is pulled back and locked by the ratchet, ready to fire. */
  public final Trigger isLoaded = new Trigger(this::isLoaded);

  /** True when the plunger is within tolerance of its target pullback position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public ShooterMechanism() {
    this(
        Mechanical.createSparkMax(Schematic.SHOOTER_CAN_ID, MotorType.kBrushless),
        Mechanical.createDigitalOutput(Schematic.SHOOTER_SOLENOID_CHANNEL));
  }

  public ShooterMechanism(SparkMax motor, DigitalOutput solenoid) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    this.solenoid = solenoid;
    Mechanical.register(motor);
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------------------------------

  /**
   * The overarching state machine managing the shooter's lifecycle: idle, winding/pullback,
   * ratchet lock (loaded), solenoid release (kick), follow-through, and manual jogging.
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Shooter");

    StateMachine.State idle = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          motor.setVoltage(0.0);
          solenoid.set(false);
        }).named("Shooter Idle"));

    StateMachine.State pullingBack = machine.addState(
        runRepeatedly(() -> {
          solenoid.set(false);
          double volts = Math.max(PULLBACK_VOLTS, pid.calculate(getPosition(), setpoint));
          motor.setVoltage(Math.clamp(volts, 0.0, MAX_VOLTS));
        }).named("Shooter Pulling Back"));
    pullingBack.onEnter(() -> {
      setpoint = LOADED_PULLBACK;
      solenoid.set(false);
    });
    pullingBack.onExit(() -> {
      motor.setVoltage(0.0);
      requestPullBack = false;
    });

    StateMachine.State loaded = machine.addState(
        runRepeatedly(() -> {
          motor.setVoltage(0.0);
          solenoid.set(false);
        }).named("Shooter Loaded"));

    StateMachine.State releasing = machine.addState(
        runRepeatedly(() -> {
          motor.setVoltage(0.0);
          solenoid.set(true);
        }).named("Shooter Releasing"));
    releasing.onEnter(() -> {
      setpoint = MIN_PULLBACK;
      motor.setVoltage(0.0);
      solenoid.set(true);
    });
    releasing.onExit(() -> requestKick = false);

    StateMachine.State followThrough = machine.addState(
        run(coroutine -> {
          setpoint = MIN_PULLBACK;
          motor.setVoltage(0.0);
          solenoid.set(true);
          for (int i = 0; i < 3; i++) {
            coroutine.yield();
          }
        }).named("Shooter Follow-Through"));
    followThrough.onExit(() -> solenoid.set(false));

    StateMachine.State manualWind = machine.addState(
        runRepeatedly(() -> {
          solenoid.set(false);
          setpoint = MAX_PULLBACK;
          motor.setVoltage(PULLBACK_VOLTS);
        }).named("Shooter Wind"));
    manualWind.onExit(() -> motor.setVoltage(0.0));

    StateMachine.State manualRelease = machine.addState(
        runRepeatedly(() -> {
          setpoint = MIN_PULLBACK;
          motor.setVoltage(0.0);
          solenoid.set(true);
        }).named("Shooter Release"));
    manualRelease.onExit(() -> solenoid.set(false));

    // Transitions:
    idle.switchTo(pullingBack).when(() -> requestPullBack);
    idle.switchTo(manualWind).when(() -> requestWind);
    idle.switchTo(manualRelease).when(() -> requestReleaseSolenoid);
    idle.switchTo(releasing).when(() -> requestKick && isLoaded());

    pullingBack.switchTo(loaded).when(this::isLoaded);
    pullingBack.switchTo(idle).when(() -> !requestPullBack);

    loaded.switchTo(releasing).when(() -> requestKick);
    loaded.switchTo(pullingBack).when(() -> requestPullBack && !isLoaded());
    loaded.switchTo(manualRelease).when(() -> requestReleaseSolenoid);

    releasing.switchTo(followThrough).when(() -> getPosition() <= MIN_PULLBACK + TOLERANCE_METERS);
    followThrough.switchTo(idle).whenComplete();

    manualWind.switchTo(idle).when(() -> !requestWind);
    manualRelease.switchTo(idle).when(() -> !requestReleaseSolenoid);

    machine.setInitialState(isLoaded() ? loaded : idle);
    return machine;
  }

  // ------------------------------------------------------------------------------------------
  // Commands and Request Triggers
  // ------------------------------------------------------------------------------------------

  public void requestPullBack() {
    this.requestPullBack = true;
  }

  public void requestKick() {
    this.requestKick = true;
  }

  public void requestWind(boolean active) {
    this.requestWind = active;
  }

  public void requestReleaseSolenoid(boolean active) {
    this.requestReleaseSolenoid = active;
  }

  /**
   * Spins the pullback motor to wind the spring back to {@link ShooterConstants#LOADED_PULLBACK}
   * with the ratchet engaged, then stops the motor so the ratchet holds it cocked.
   */
  public Command pullBack() {
    return Command.noRequirements(coroutine -> {
      if (isLoaded()) {
        return;
      }
      requestPullBack = true;
      while (!isLoaded()) {
        coroutine.yield();
      }
    }).named("Shooter Pull Back");
  }

  /**
   * Releases the solenoid to disengage the ratchet and kick the ball into the air, then re-engages
   * the ratchet once the spring has fully extended.
   */
  public Command kick() {
    return Command.noRequirements(coroutine -> {
      if (!isLoaded()) {
        return;
      }
      requestKick = true;
      while (isRatchetLocked() && requestKick) {
        coroutine.yield();
      }
      while (isSolenoidReleased()) {
        coroutine.yield();
      }
    }).named("Shooter Kick");
  }

  /**
   * Manual command that winds the pullback motor while held (will stall at {@link
   * ShooterConstants#MAX_PULLBACK} if held past the limit).
   */
  public Command wind() {
    return Command.noRequirements(coroutine -> {
      requestWind = true;
      while (true) {
        coroutine.yield();
      }
    }).whenCanceled(() -> requestWind = false).named("Shooter Wind");
  }

  /** Manual command that holds the release solenoid open while active. */
  public Command releaseSolenoid() {
    return Command.noRequirements(coroutine -> {
      requestReleaseSolenoid = true;
      while (true) {
        coroutine.yield();
      }
    }).whenCanceled(() -> requestReleaseSolenoid = false).named("Shooter Release");
  }

  /**
   * Default command: runs the shooter state machine, maintaining idle or holding cocked spring
   * tension without drawing motor current.
   */
  public Command hold() {
    return run(coroutine -> coroutine.await(stateMachine()))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Shooter Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean isLoaded() {
    return getPosition() >= LOADED_PULLBACK - TOLERANCE_METERS;
  }

  public boolean isRatchetLocked() {
    return !isSolenoidReleased();
  }

  public boolean isSolenoidReleased() {
    return solenoid.get();
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    Double pos = encoder.getPosition().get();
    return pos != null ? pos : STARTING_PULLBACK;
  }

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    return vel != null ? vel : 0.0;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    return val != null ? val * 12.0 : 0.0;
  }

  public DigitalOutput getSolenoid() {
    return solenoid;
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_PULLBACK;
  }
}
