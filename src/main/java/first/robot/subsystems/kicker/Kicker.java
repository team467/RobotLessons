package first.robot.subsystems.kicker;

import static first.robot.subsystems.kicker.KickerConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import first.robot.mechanical.SpringKickerMechanism;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.discrete.DigitalOutput;
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
  private final SpringKickerMechanism mechanism;

  /** True when the kicker spring is pulled back and locked by the ratchet, ready to fire. */
  public final Trigger isLoaded;

  /** True when the plunger is within tolerance of its target pullback position. */
  public final Trigger atSetpoint;

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

    this.mechanism =
        new SpringKickerMechanism(
            "Kicker",
            motor::setVoltage,
            solenoid::set,
            solenoid::get,
            this::getPosition,
            this::getVelocity,
            KP,
            KD,
            MAX_VOLTS,
            PULLBACK_VOLTS,
            TOLERANCE,
            MIN_PULLBACK,
            MAX_PULLBACK,
            LOADED_PULLBACK,
            STARTING_PULLBACK);

    this.isLoaded = mechanism.isLoaded;
    this.atSetpoint = mechanism.atSetpoint;
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------------------------------

  public StateMachine stateMachine() {
    return mechanism.stateMachine();
  }

  // ------------------------------------------------------------------------------------------
  // Goals and Commands
  // ------------------------------------------------------------------------------------------

  public void setGoal(Goal goal) {
    mechanism.setGoal(SpringKickerMechanism.Goal.valueOf(goal.name()));
  }

  public Goal getGoal() {
    return Goal.valueOf(mechanism.getGoal().name());
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

  public Command pullBack() {
    return mechanism.pullBack();
  }

  public Command kick() {
    return mechanism.kick();
  }

  public Command wind() {
    return mechanism.wind();
  }

  public Command releaseSolenoid() {
    return mechanism.releaseSolenoid();
  }

  public Command hold() {
    return run(coroutine -> coroutine.await(stateMachine()))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Kicker Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean isLoaded() {
    return mechanism.isLoaded();
  }

  public boolean isRatchetLocked() {
    return mechanism.isRatchetLocked();
  }

  public boolean isSolenoidReleased() {
    return mechanism.isSolenoidReleased();
  }

  public boolean atSetpoint() {
    return mechanism.atSetpoint();
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
    return mechanism.getSetpoint();
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
    mechanism.resetSetpoint();
  }

  public SpringKickerMechanism getMechanism() {
    return mechanism;
  }
}
