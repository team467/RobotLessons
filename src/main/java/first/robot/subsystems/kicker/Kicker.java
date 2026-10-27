package first.robot.subsystems.kicker;

import static first.robot.subsystems.kicker.KickerConstants.*;
import static org.wpilib.units.Units.Meters;

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
import org.wpilib.units.measure.Distance;

/**
 * Spring-loaded kicker subsystem mounted on the front of the robot.
 *
 * <p>A pullback motor spins to pull the spring-loaded plunger back while a mechanical ratchet locks
 * it in place. Releasing the solenoid disengages the ratchet to kick a ball held close to the robot
 * in an upward arc toward the elevated basket.
 */
public class Kicker implements Mechanism {
  /** The target goal/intent for the kicker state machine. */
  public enum Goal {
    IDLE,
    LOAD,
    KICK,
    WIND,
    RELEASE
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalOutput solenoid;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private Distance setpoint = STARTING_PULLBACK;
  private Goal goal = Goal.IDLE;

  /** True when the kicker spring is pulled back and locked by the ratchet, ready to fire. */
  public final Trigger isLoaded = new Trigger(this::isLoaded);

  /** True when the plunger is within tolerance of its target pullback position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public Kicker() {
    this(
        Mechanical.createSparkMax(Schematic.KICKER_CAN_ID, MotorType.kBrushless),
        Mechanical.createDigitalOutput(Schematic.KICKER_SOLENOID_CHANNEL));
  }

  public Kicker(SparkMax motor, DigitalOutput solenoid) {
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
   * The overarching state machine managing the kicker's lifecycle: idle, winding/pullback,
   * ratchet lock (loaded), solenoid release (kick), follow-through, and manual jogging.
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Kicker");

    StateMachine.State idle = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          motor.setVoltage(0.0);
          solenoid.set(false);
        }).named("Kicker Idle"));

    StateMachine.State pullingBack = machine.addState(
        runRepeatedly(() -> {
          solenoid.set(false);
          double volts = Math.max(PULLBACK_VOLTS, pid.calculate(getPosition().in(Meters), setpoint.in(Meters)));
          motor.setVoltage(Math.clamp(volts, 0.0, MAX_VOLTS));
        }).named("Kicker Pulling Back"));
    pullingBack.onEnter(() -> {
      setpoint = LOADED_PULLBACK;
      solenoid.set(false);
    });
    pullingBack.onExit(() -> motor.setVoltage(0.0));

    StateMachine.State loaded = machine.addState(
        runRepeatedly(() -> {
          motor.setVoltage(0.0);
          solenoid.set(false);
        }).named("Kicker Loaded"));

    StateMachine.State releasing = machine.addState(
        runRepeatedly(() -> {
          motor.setVoltage(0.0);
          solenoid.set(true);
        }).named("Kicker Releasing"));
    releasing.onEnter(() -> {
      setpoint = MIN_PULLBACK;
      motor.setVoltage(0.0);
      solenoid.set(true);
    });

    StateMachine.State followThrough = machine.addState(
        run(coroutine -> {
          setpoint = MIN_PULLBACK;
          motor.setVoltage(0.0);
          solenoid.set(true);
          for (int i = 0; i < 3; i++) {
            coroutine.yield();
          }
        }).named("Kicker Follow-Through"));
    followThrough.onExit(() -> {
      solenoid.set(false);
      if (goal == Goal.KICK) {
        goal = Goal.IDLE;
      }
    });

    StateMachine.State manualWind = machine.addState(
        runRepeatedly(() -> {
          solenoid.set(false);
          setpoint = MAX_PULLBACK;
          motor.setVoltage(PULLBACK_VOLTS);
        }).named("Kicker Wind"));
    manualWind.onExit(() -> motor.setVoltage(0.0));

    StateMachine.State manualRelease = machine.addState(
        runRepeatedly(() -> {
          setpoint = MIN_PULLBACK;
          motor.setVoltage(0.0);
          solenoid.set(true);
        }).named("Kicker Release"));
    manualRelease.onExit(() -> solenoid.set(false));

    // Transitions from idle:
    idle.switchTo(pullingBack).when(() -> goal == Goal.LOAD);
    idle.switchTo(manualWind).when(() -> goal == Goal.WIND);
    idle.switchTo(manualRelease).when(() -> goal == Goal.RELEASE);
    idle.switchTo(releasing).when(() -> goal == Goal.KICK && isLoaded());

    // Transitions from pullingBack:
    pullingBack.switchTo(loaded).when(this::isLoaded);
    pullingBack.switchTo(idle).when(() -> goal != Goal.LOAD);

    // Transitions from loaded:
    loaded.switchTo(releasing).when(() -> goal == Goal.KICK);
    loaded.switchTo(pullingBack).when(() -> goal == Goal.LOAD && !isLoaded());
    loaded.switchTo(manualRelease).when(() -> goal == Goal.RELEASE);
    loaded.switchTo(idle).when(() -> goal == Goal.IDLE && !isLoaded());

    // Transitions from releasing:
    releasing.switchTo(followThrough).when(() -> getPosition().lte(MIN_PULLBACK.plus(TOLERANCE)));

    // Transitions from followThrough:
    followThrough.switchTo(idle).whenComplete();

    // Transitions from manual states:
    manualWind.switchTo(idle).when(() -> goal != Goal.WIND);
    manualRelease.switchTo(idle).when(() -> goal != Goal.RELEASE);

    if (isLoaded()) {
      machine.setInitialState(loaded);
    } else {
      machine.setInitialState(idle);
    }
    return machine;
  }

  // ------------------------------------------------------------------------------------------
  // Goals and Commands
  // ------------------------------------------------------------------------------------------

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public void requestPullBack() {
    setGoal(Goal.LOAD);
  }

  public void requestKick() {
    setGoal(Goal.KICK);
  }

  public void requestWind(boolean active) {
    if (active) {
      setGoal(Goal.WIND);
    } else {
      setGoal(Goal.IDLE);
    }
  }

  public void requestReleaseSolenoid(boolean active) {
    if (active) {
      setGoal(Goal.RELEASE);
    } else {
      setGoal(Goal.IDLE);
    }
  }

  /**
   * Spins the pullback motor to wind the spring back to {@link KickerConstants#LOADED_PULLBACK}
   * with the ratchet engaged, then stops the motor so the ratchet holds it cocked.
   */
  public Command pullBack() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.LOAD);
      if (isLoaded()) {
        return;
      }
      coroutine.waitUntil(this::isLoaded);
    }).whenCanceled(() -> {
      if (goal == Goal.LOAD && !isLoaded()) {
        setGoal(Goal.IDLE);
      }
    }).named("Kicker Pull Back");
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
      setGoal(Goal.KICK);
      coroutine.waitUntil(() -> !isLoaded() || isSolenoidReleased());
      coroutine.waitUntil(() -> !isSolenoidReleased() && goal != Goal.KICK);
    }).whenCanceled(() -> {
      if (goal == Goal.KICK) {
        setGoal(Goal.IDLE);
      }
    }).named("Kicker Kick");
  }

  /**
   * Manual command that winds the pullback motor while held (will stall at {@link
   * KickerConstants#MAX_PULLBACK} if held past the limit).
   */
  public Command wind() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.WIND);
      coroutine.park();
    }).whenCanceled(() -> setGoal(Goal.IDLE)).named("Kicker Wind");
  }

  /** Manual command that holds the release solenoid open while active. */
  public Command releaseSolenoid() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.RELEASE);
      coroutine.park();
    }).whenCanceled(() -> setGoal(Goal.IDLE)).named("Kicker Release");
  }

  /**
   * Default command: runs the kicker state machine, maintaining idle or holding cocked spring
   * tension without drawing motor current.
   */
  public Command hold() {
    return run(coroutine -> coroutine.await(stateMachine()))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Kicker Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean isLoaded() {
    return getPosition().gte(LOADED_PULLBACK.minus(TOLERANCE));
  }

  public boolean isRatchetLocked() {
    return !isSolenoidReleased();
  }

  public boolean isSolenoidReleased() {
    return solenoid.get();
  }

  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, TOLERANCE);
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_PULLBACK;
    }
    return Meters.of(pos);
  }

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    if (vel == null) {
      return 0.0;
    }
    return vel;
  }

  public Distance getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    if (val == null) {
      return 0.0;
    }
    return val * 12.0;
  }

  public DigitalOutput getSolenoid() {
    return solenoid;
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_PULLBACK;
    goal = Goal.IDLE;
  }
}
