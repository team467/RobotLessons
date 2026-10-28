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
   *
   * <p>State Overview:
   * <ul>
   *   <li><b>idle</b>: Plunger at rest or uncocked. Motor is unpowered (0V) and solenoid is de-energized
   *       (ratchet engaged).
   *   <li><b>pullingBack</b>: Motor applies pullback voltage (minimum {@link KickerConstants#PULLBACK_VOLTS} = 8.0V)
   *       to winch the plunger backward against the heavy launch spring.
   *   <li><b>loaded</b>: Plunger has reached {@link KickerConstants#LOADED_PULLBACK} (0.22m). The one-way
   *       ratchet holds spring tension mechanically, so motor voltage drops to 0.0V (zero electrical draw).
   *   <li><b>releasing</b>: Solenoid energizes (tripping the ratchet pawl), allowing the spring to snap
   *       the plunger forward at high velocity to launch the ball.
   *   <li><b>followThrough</b>: Brief 3-loop dwell ensuring kinetic energy is fully transferred to the ball
   *       before the solenoid de-energizes and the ratchet re-engages.
   *   <li><b>manualWind</b> / <b>manualRelease</b>: Direct jog states for teleop calibration.
   * </ul>
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Kicker");

    // State 1: Idle - Rest position, 0V applied, solenoid off (ratchet engaged)
    StateMachine.State idle = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          motor.setVoltage(0.0);
          solenoid.set(false);
        }).named("Kicker Idle"));

    // State 2: Pulling Back - Motor powers the winch to draw spring to 0.22m pullback
    StateMachine.State pullingBack = machine.addState(
        runRepeatedly(() -> {
          solenoid.set(false); // Ensure ratchet is engaged to catch each tooth
          double volts = Math.max(PULLBACK_VOLTS, pid.calculate(getPosition().in(Meters), setpoint.in(Meters)));
          motor.setVoltage(Math.clamp(volts, 0.0, MAX_VOLTS));
        }).named("Kicker Pulling Back"));
    pullingBack.onEnter(() -> {
      setpoint = LOADED_PULLBACK;
      solenoid.set(false);
    });
    pullingBack.onExit(() -> motor.setVoltage(0.0));

    // State 3: Loaded - Cocked and locked by the ratchet, 0V drawn
    StateMachine.State loaded = machine.addState(
        runRepeatedly(() -> {
          motor.setVoltage(0.0);
          solenoid.set(false);
        }).named("Kicker Loaded"));

    // State 4: Releasing - Solenoid energizes to trip ratchet and unleash spring
    StateMachine.State releasing = machine.addState(
        runRepeatedly(() -> {
          motor.setVoltage(0.0);
          solenoid.set(true); // Energize release solenoid
        }).named("Kicker Releasing"));
    releasing.onEnter(() -> {
      setpoint = MIN_PULLBACK;
      motor.setVoltage(0.0);
      solenoid.set(true);
    });

    // State 5: Follow-Through - Dwell for 3 periodic ticks to guarantee full ball launch
    StateMachine.State followThrough = machine.addState(
        run(coroutine -> {
          setpoint = MIN_PULLBACK;
          motor.setVoltage(0.0);
          solenoid.set(true);
          // Yield 3 ticks (60ms) while plunger drives through impact point
          for (int i = 0; i < 3; i++) {
            coroutine.yield();
          }
        }).named("Kicker Follow-Through"));
    followThrough.onExit(() -> {
      solenoid.set(false); // De-energize solenoid, resetting ratchet pawl
      if (goal == Goal.KICK) {
        goal = Goal.IDLE;
      }
    });

    // State 6: Manual Wind - Teleop override winding
    StateMachine.State manualWind = machine.addState(
        runRepeatedly(() -> {
          solenoid.set(false);
          setpoint = MAX_PULLBACK;
          motor.setVoltage(PULLBACK_VOLTS);
        }).named("Kicker Wind"));
    manualWind.onExit(() -> motor.setVoltage(0.0));

    // State 7: Manual Release - Teleop override solenoid open
    StateMachine.State manualRelease = machine.addState(
        runRepeatedly(() -> {
          setpoint = MIN_PULLBACK;
          motor.setVoltage(0.0);
          solenoid.set(true);
        }).named("Kicker Release"));
    manualRelease.onExit(() -> solenoid.set(false));

    // ---- Transitions from idle ----
    idle.switchTo(pullingBack).when(() -> goal == Goal.LOAD);
    idle.switchTo(manualWind).when(() -> goal == Goal.WIND);
    idle.switchTo(manualRelease).when(() -> goal == Goal.RELEASE);
    idle.switchTo(releasing).when(() -> goal == Goal.KICK && isLoaded());

    // ---- Transitions from pullingBack ----
    pullingBack.switchTo(loaded).when(this::isLoaded);
    pullingBack.switchTo(idle).when(() -> goal != Goal.LOAD);

    // ---- Transitions from loaded ----
    loaded.switchTo(releasing).when(() -> goal == Goal.KICK);
    loaded.switchTo(pullingBack).when(() -> goal == Goal.LOAD && !isLoaded());
    loaded.switchTo(manualRelease).when(() -> goal == Goal.RELEASE);
    loaded.switchTo(idle).when(() -> goal == Goal.IDLE && !isLoaded());

    // ---- Transitions from releasing ----
    releasing.switchTo(followThrough).when(() -> getPosition().lte(MIN_PULLBACK.plus(TOLERANCE)));

    // ---- Transitions from followThrough ----
    followThrough.switchTo(idle).whenComplete();

    // ---- Transitions from manual states ----
    manualWind.switchTo(idle).when(() -> goal != Goal.WIND);
    manualRelease.switchTo(idle).when(() -> goal != Goal.RELEASE);

    // Set initial state: if already locked at pullback height, resume loaded
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
   * Spins the pullback motor to wind the spring back to {@link KickerConstants#LOADED_PULLBACK} (0.22m)
   * with the ratchet engaged, then stops the motor so the ratchet holds it cocked.
   *
   * <p>Execution:
   * <ul>
   *   <li>Sets goal to {@link Goal#LOAD}.
   *   <li>If already cocked and loaded, returns immediately.
   *   <li>Waits until {@link #isLoaded} is true (plunger &ge; 0.205m).
   *   <li>If canceled before reaching loaded state, resets goal back to {@link Goal#IDLE}.
   * </ul>
   *
   * @return a command that cocks the spring-loaded kicker
   */
  public Command pullBack() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.LOAD);
      // Skip wait if already cocked and locked
      if (isLoaded()) {
        return;
      }
      // Await until winch pulls spring back to ratchet engagement threshold
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
   *
   * <p>Execution:
   * <ul>
   *   <li>Requires that the kicker is cocked ({@link #isLoaded}); no-ops if uncocked to protect mechanism.
   *   <li>Sets goal to {@link Goal#KICK}, which energizes the release solenoid.
   *   <li>Awaits spring acceleration through impact and follow-through dwell.
   *   <li>Completes when the solenoid has safely de-energized and goal resets to {@link Goal#IDLE}.
   * </ul>
   *
   * @return a command that fires the kicker and returns to idle
   */
  public Command kick() {
    return Command.noRequirements(coroutine -> {
      // Must be cocked to kick; prevent dry-firing or unlatched solenoid release
      if (!isLoaded()) {
        return;
      }
      setGoal(Goal.KICK);
      // Wait for release sequence to initiate
      coroutine.waitUntil(() -> !isLoaded() || isSolenoidReleased());
      // Wait for follow-through to finish and solenoid to reset
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
   *
   * @return manual winding command
   */
  public Command wind() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.WIND);
      coroutine.park();
    }).whenCanceled(() -> setGoal(Goal.IDLE)).named("Kicker Wind");
  }

  /**
   * Manual command that holds the release solenoid open while active.
   *
   * @return manual solenoid release command
   */
  public Command releaseSolenoid() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.RELEASE);
      coroutine.park();
    }).whenCanceled(() -> setGoal(Goal.IDLE)).named("Kicker Release");
  }

  /**
   * Default command: runs the kicker state machine, maintaining idle or holding cocked spring
   * tension without drawing motor current.
   *
   * <p>Runs at {@link Command#LOWEST_PRIORITY} so explicit kick or pullback actions take priority.
   *
   * @return continuous default holding command
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
    pid.reset();
  }
}
