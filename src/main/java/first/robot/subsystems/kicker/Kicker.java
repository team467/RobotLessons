package first.robot.subsystems.kicker;

import static first.robot.subsystems.kicker.KickerConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import first.robot.mechanisms.KickerMechanism;
import first.robot.mechanisms.TrackedStateMachine;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.units.measure.Distance;

/**
 * Spring-loaded kicker subsystem mounted on the front of the robot.
 *
 * <p>A pullback motor spins to pull the spring-loaded plunger back while a mechanical ratchet locks
 * it in place. Releasing the solenoid disengages the ratchet to kick a ball held close to the robot
 * in an upward arc toward the elevated basket.
 */
public class Kicker {
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
  private final KickerMechanism kickerMechanism;
  private final TrackedStateMachine<Mode> stateMachine = new TrackedStateMachine<>("Kicker");

  /** True when the kicker spring is pulled back and locked by the ratchet, ready to fire. */
  public final Trigger isLoaded;

  /** True when the plunger is within tolerance of its target pullback position. */
  public final Trigger atSetpoint;

  public Kicker() {
    this.motor = Mechanical.createSparkMax(Schematic.KICKER_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    this.solenoid = Mechanical.createDigitalOutput(Schematic.KICKER_SOLENOID_CHANNEL);
    Mechanical.register(motor);

    this.kickerMechanism =
        new KickerMechanism(
            motor::setVoltage,
            solenoid::set,
            solenoid::get,
            this::getPosition,
            this::getVelocity);

    this.isLoaded = kickerMechanism.isLoaded;
    this.atSetpoint = kickerMechanism.atSetpoint;

    buildStateMachine();
    kickerMechanism.setDefaultCommand(stateMachine.runOn(kickerMechanism));
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
        kickerMechanism.run(coroutine -> {
          while (true) {
            if (RobotBase.isDisabled()) {
              kickerMechanism.setSetpoint(getPosition());
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
            kickerMechanism.run(coroutine -> {
              kickerMechanism.setSetpoint(LOADED_PULLBACK);
              solenoid.set(false);
              while (!isLoaded()) {
                kickerMechanism.driveTowardSetpoint();
                coroutine.yield();
              }
            }).named("Kicker Pulling Back"))
        .onExit(() -> motor.setVoltage(0.0));

    // Cocked and locked by the ratchet, 0V drawn
    stateMachine.addState(
        Mode.LOADED,
        kickerMechanism.run(coroutine -> {
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
            kickerMechanism.run(coroutine -> {
              kickerMechanism.setSetpoint(MIN_PULLBACK);
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

    stateMachine.transition(Mode.IDLE, Mode.LOADED, this::isLoaded);
    stateMachine.transition(Mode.IDLE, Mode.LOAD, stateMachine.isRequested(Mode.LOAD));
    stateMachine.transition(
        Mode.IDLE,
        Mode.KICK,
        () -> stateMachine.isRequested(Mode.KICK).getAsBoolean() && isLoaded());

    stateMachine.transitionWhenComplete(Mode.LOAD, Mode.LOADED);
    stateMachine.transition(Mode.LOAD, Mode.IDLE, stateMachine.isRequested(Mode.IDLE));

    stateMachine.transition(Mode.LOADED, Mode.KICK, stateMachine.isRequested(Mode.KICK));
    stateMachine.transition(Mode.LOADED, Mode.LOAD, stateMachine.isRequested(Mode.LOAD));
    stateMachine.transition(Mode.LOADED, Mode.IDLE, () -> !isLoaded());

    stateMachine.transitionWhenComplete(Mode.KICK, Mode.IDLE);

    stateMachine.setInitialState(Mode.IDLE);
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

  public boolean isLoaded() {
    return kickerMechanism.isLoaded();
  }

  public boolean isRatchetLocked() {
    return kickerMechanism.isRatchetLocked();
  }

  public boolean isSolenoidReleased() {
    return kickerMechanism.isSolenoidReleased();
  }

  public boolean atSetpoint() {
    return kickerMechanism.atSetpoint();
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
    return kickerMechanism.getSetpoint();
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    if (val == null) {
      return 0.0;
    }
    return val * 12.0;
  }

  public String getCurrentCommandName() {
    List<Command> running = kickerMechanism.getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    kickerMechanism.resetSetpoint();
  }
}
