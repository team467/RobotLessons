package first.robot.subsystems.kicker;

import static first.robot.subsystems.kicker.KickerConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.lib.state.TrackedStateMachine;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
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
@Logged
public class Kicker implements Mechanism {
  /** The states of the kicker state machine, which are also the goals commands can request. */
  private enum Mode {
    IDLE,
    LOAD,
    LOADED,
    KICK
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalOutput solenoid;
  private final PIDController pid = new PIDController(KP, 0.0, KD);
  private final TrackedStateMachine<Mode> stateMachine = new TrackedStateMachine<>("Kicker");

  private Distance setpoint = STARTING_PULLBACK;

  public Kicker() {
    this.motor = Mechanical.createSparkMax(Schematic.KICKER_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    this.solenoid = Mechanical.createDigitalOutput(Schematic.KICKER_SOLENOID_CHANNEL);

    buildStateMachine();
    setDefaultCommand(stateMachine.runOn(this));
  }

  // ------------------------------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------------------------------

  /**
   * Declares the kicker's states and transitions between idle, pulling back, loaded, and kicking.
   */
  private void buildStateMachine() {
    // Rest position, 0V applied, solenoid off (ratchet engaged)
    stateMachine.addState(
        Mode.IDLE,
        run(coroutine -> {
          while (true) {
            if (RobotBase.isDisabled()) {
              setpoint = getPosition();
            }
            motor.setVoltage(0.0);
            solenoid.set(false);
            coroutine.yield();
          }
        }).named("Kicker Idle"));

    // Motor powers the winch to draw spring to loaded pullback
    stateMachine
        .addState(
            Mode.LOAD,
            run(coroutine -> {
              setSetpoint(LOADED_PULLBACK);
              solenoid.set(false);
              while (!isLoaded()) {
                driveTowardSetpoint();
                coroutine.yield();
              }
            }).named("Kicker Pulling Back"))
        .onExit(() -> motor.setVoltage(0.0));

    // Cocked and locked by the ratchet, 0V drawn
    stateMachine.addState(
        Mode.LOADED,
        run(coroutine -> {
          while (true) {
            motor.setVoltage(0.0);
            solenoid.set(false);
            coroutine.yield();
          }
        }).named("Kicker Loaded"));

    // Solenoid energizes to trip ratchet and unleash spring, followed by 3 ticks of follow-through
    stateMachine
        .addState(
            Mode.KICK,
            run(coroutine -> {
              setSetpoint(MIN_PULLBACK);
              motor.setVoltage(0.0);
              solenoid.set(true);
              while (!getPosition().lte(MIN_PULLBACK.plus(TOLERANCE))) {
                motor.setVoltage(0.0);
                solenoid.set(true);
                coroutine.yield();
              }
              for (int i = 0; i < 3; i++) {
                motor.setVoltage(0.0);
                solenoid.set(true);
                coroutine.yield();
              }
            }).named("Kicker Releasing"))
        .onExit(() -> solenoid.set(false));

    // Transitions from IDLE
    stateMachine.transition(Mode.IDLE, Mode.LOADED, this::isLoaded);
    stateMachine.transition(Mode.IDLE, Mode.LOAD, stateMachine.isRequested(Mode.LOAD));
    stateMachine.transition(
        Mode.IDLE,
        Mode.KICK,
        () -> stateMachine.isRequested(Mode.KICK).getAsBoolean() && isLoaded());

    // Transitions from LOAD
    stateMachine.transitionWhenComplete(Mode.LOAD, Mode.LOADED);
    stateMachine.transition(Mode.LOAD, Mode.IDLE, stateMachine.isRequested(Mode.IDLE));

    // Transitions from LOADED
    stateMachine.transition(Mode.LOADED, Mode.KICK, stateMachine.isRequested(Mode.KICK));
    stateMachine.transition(Mode.LOADED, Mode.LOAD, stateMachine.isRequested(Mode.LOAD));
    stateMachine.transition(Mode.LOADED, Mode.IDLE, () -> !isLoaded());

    // Transitions from KICK
    stateMachine.transitionWhenComplete(Mode.KICK, Mode.LOAD);

    stateMachine.setInitialState(Mode.LOAD);
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command pullBack() {
    return stateMachine.setGoal(Mode.LOAD);
  }

  public Command kick() {
    return stateMachine.setGoal(Mode.KICK);
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public void driveTowardSetpoint() {
    if (RobotBase.isDisabled()) {
      motor.setVoltage(0.0);
      return;
    }
    solenoid.set(false);
    double volts = Math.max(PULLBACK_VOLTS, pid.calculate(getPosition().in(Meters), setpoint.in(Meters)));
    motor.setVoltage(Math.clamp(volts, 0.0, MAX_VOLTS));
  }

  public Distance clampPullback(Distance pullback) {
    if (pullback.lt(MIN_PULLBACK)) {
      return MIN_PULLBACK;
    } else if (pullback.gt(MAX_PULLBACK)) {
      return MAX_PULLBACK;
    } else {
      return pullback;
    }
  }

  public boolean isLoaded() {
    return getPosition().gte(LOADED_PULLBACK.minus(TOLERANCE));
  }

  public boolean isRatchetLocked() {
    return !solenoid.get();
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

  public void setSetpoint(Distance target) {
    this.setpoint = clampPullback(target);
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    if (val == null) {
      return 0.0;
    }
    return val * 12.0;
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
    pid.reset();
  }
}
