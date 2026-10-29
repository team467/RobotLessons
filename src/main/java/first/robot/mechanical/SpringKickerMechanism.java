package first.robot.mechanical;

import static org.wpilib.units.Units.Meters;

import java.util.function.BooleanSupplier;
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
 * Reusable generic mechanism representing a spring-loaded, ratchet-latched launcher / kicker.
 *
 * <p>Encapsulates:
 * <ul>
 *   <li>Pullback winch motor control with heavy spring feedforward.</li>
 *   <li>One-way ratchet locking where holding draws 0V.</li>
 *   <li>Solenoid release trigger and follow-through dwell timing.</li>
 *   <li>Full lifecycle state machine: Idle, Pulling Back, Loaded, Releasing, Follow-Through, Manual Wind/Release.</li>
 *   <li>High-level asynchronous commands: {@link #pullBack()} and {@link #kick()}.</li>
 * </ul>
 */
public class SpringKickerMechanism implements Mechanism {
  /** The target goal/intent for the state machine. */
  public enum Goal {
    IDLE,
    LOAD,
    KICK,
    WIND,
    RELEASE
  }

  private final String name;
  private final Consumer<Double> setMotorVoltage;
  private final Consumer<Boolean> setSolenoid;
  private final BooleanSupplier isSolenoidReleased;
  private final Supplier<Distance> getPosition;
  private final DoubleSupplier getVelocity;

  private final PIDController pid;
  private final double maxVolts;
  private final double pullbackVolts;
  private final Distance tolerance;
  private final Distance minPullback;
  private final Distance maxPullback;
  private final Distance loadedPullback;
  private final Distance startingPullback;

  private Distance setpoint;
  private Goal goal = Goal.IDLE;

  /** True when the spring is pulled back and locked by the ratchet, ready to fire. */
  public final Trigger isLoaded = new Trigger(this::isLoaded);

  /** True when the plunger is within tolerance of its target pullback position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public SpringKickerMechanism(
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
    this.getVelocity = getVelocity;
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
   * The overarching state machine managing the spring kicker's lifecycle.
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine(name);

    // State 1: Idle - Rest position, 0V applied, solenoid off (ratchet engaged)
    StateMachine.State idle = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          setMotorVoltage.accept(0.0);
          setSolenoid.accept(false);
        }).named(name + " Idle"));

    // State 2: Pulling Back - Motor powers the winch to draw spring to loaded pullback
    StateMachine.State pullingBack = machine.addState(
        runRepeatedly(() -> {
          setSolenoid.accept(false); // Ensure ratchet is engaged to catch each tooth
          double volts = Math.max(pullbackVolts, pid.calculate(getPosition().in(Meters), setpoint.in(Meters)));
          setMotorVoltage.accept(Math.clamp(volts, 0.0, maxVolts));
        }).named(name + " Pulling Back"));
    pullingBack.onEnter(() -> {
      setpoint = loadedPullback;
      setSolenoid.accept(false);
    });
    pullingBack.onExit(() -> setMotorVoltage.accept(0.0));

    // State 3: Loaded - Cocked and locked by the ratchet, 0V drawn
    StateMachine.State loaded = machine.addState(
        runRepeatedly(() -> {
          setMotorVoltage.accept(0.0);
          setSolenoid.accept(false);
        }).named(name + " Loaded"));

    // State 4: Releasing - Solenoid energizes to trip ratchet and unleash spring
    StateMachine.State releasing = machine.addState(
        runRepeatedly(() -> {
          setMotorVoltage.accept(0.0);
          setSolenoid.accept(true);
        }).named(name + " Releasing"));
    releasing.onEnter(() -> {
      setpoint = minPullback;
      setMotorVoltage.accept(0.0);
      setSolenoid.accept(true);
    });

    // State 5: Follow-Through - Dwell for 3 periodic ticks to guarantee full launch
    StateMachine.State followThrough = machine.addState(
        run(coroutine -> {
          setpoint = minPullback;
          setMotorVoltage.accept(0.0);
          setSolenoid.accept(true);
          for (int i = 0; i < 3; i++) {
            coroutine.yield();
          }
        }).named(name + " Follow-Through"));
    followThrough.onExit(() -> {
      setSolenoid.accept(false);
      if (goal == Goal.KICK) {
        goal = Goal.IDLE;
      }
    });

    // State 6: Manual Wind - Teleop override winding
    StateMachine.State manualWind = machine.addState(
        runRepeatedly(() -> {
          setSolenoid.accept(false);
          setpoint = maxPullback;
          setMotorVoltage.accept(pullbackVolts);
        }).named(name + " Wind"));
    manualWind.onExit(() -> setMotorVoltage.accept(0.0));

    // State 7: Manual Release - Teleop override solenoid open
    StateMachine.State manualRelease = machine.addState(
        runRepeatedly(() -> {
          setpoint = minPullback;
          setMotorVoltage.accept(0.0);
          setSolenoid.accept(true);
        }).named(name + " Release"));
    manualRelease.onExit(() -> setSolenoid.accept(false));

    // Transitions from idle
    idle.switchTo(pullingBack).when(() -> goal == Goal.LOAD);
    idle.switchTo(manualWind).when(() -> goal == Goal.WIND);
    idle.switchTo(manualRelease).when(() -> goal == Goal.RELEASE);
    idle.switchTo(releasing).when(() -> goal == Goal.KICK && isLoaded());

    // Transitions from pullingBack
    pullingBack.switchTo(loaded).when(this::isLoaded);
    pullingBack.switchTo(idle).when(() -> goal != Goal.LOAD);

    // Transitions from loaded
    loaded.switchTo(releasing).when(() -> goal == Goal.KICK);
    loaded.switchTo(pullingBack).when(() -> goal == Goal.LOAD && !isLoaded());
    loaded.switchTo(manualRelease).when(() -> goal == Goal.RELEASE);
    loaded.switchTo(idle).when(() -> goal == Goal.IDLE && !isLoaded());

    // Transitions from releasing
    releasing.switchTo(followThrough).when(() -> getPosition().lte(minPullback.plus(tolerance)));

    // Transitions from followThrough
    followThrough.switchTo(idle).whenComplete();

    // Transitions from manual states
    manualWind.switchTo(idle).when(() -> goal != Goal.WIND);
    manualRelease.switchTo(idle).when(() -> goal != Goal.RELEASE);

    if (isLoaded()) {
      machine.setInitialState(loaded);
    } else {
      machine.setInitialState(idle);
    }
    return machine;
  }

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
    }).named(name + " Pull Back");
  }

  public Command kick() {
    return Command.noRequirements(coroutine -> {
      if (!isLoaded()) {
        return;
      }
      setGoal(Goal.KICK);
      coroutine.waitUntil(() -> !isLoaded() || isSolenoidReleased.getAsBoolean());
      coroutine.waitUntil(() -> !isSolenoidReleased.getAsBoolean() && goal != Goal.KICK);
    }).whenCanceled(() -> {
      if (goal == Goal.KICK) {
        setGoal(Goal.IDLE);
      }
    }).named(name + " Kick");
  }

  public Command wind() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.WIND);
      coroutine.park();
    }).whenCanceled(() -> setGoal(Goal.IDLE)).named(name + " Wind");
  }

  public Command releaseSolenoid() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.RELEASE);
      coroutine.park();
    }).whenCanceled(() -> setGoal(Goal.IDLE)).named(name + " Release");
  }

  public Command hold() {
    return run(coroutine -> coroutine.await(stateMachine()))
        .withPriority(Command.LOWEST_PRIORITY)
        .named(name + " Hold");
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

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public void resetSetpoint() {
    setpoint = startingPullback;
    goal = Goal.IDLE;
    pid.reset();
  }

  public void resetPid() {
    pid.reset();
  }

  public PIDController getPid() {
    return pid;
  }
}
