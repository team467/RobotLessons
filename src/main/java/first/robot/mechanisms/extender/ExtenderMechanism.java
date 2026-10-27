package first.robot.mechanisms.extender;

import static first.robot.mechanisms.extender.ExtenderConstants.*;

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
import org.wpilib.math.controller.PIDController;

/** The extender slides the claw in and out on a pulley (the x-axis). */
public class ExtenderMechanism implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = STARTING_EXTENSION;

  /** True while the extender is within tolerance of its setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public ExtenderMechanism() {
    this(Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless));
  }

  public ExtenderMechanism(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);
    setDefaultCommand(hold());
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
    setpoint = target;
    String name = String.format("Extender To %.2fm", target);
    StateMachine sm = new StateMachine(name);
    var moving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    moving.onEnter(() -> setpoint = target);
    sm.setInitialState(moving);
    moving.exitStateMachine().when(this::atSetpoint);
    return run(coroutine -> coroutine.await(sm)).named(name);
  }

  /** Extends outward smoothly while held, stopping at MAX_EXTENSION. */
  public Command out() {
    StateMachine sm = new StateMachine("Extender Out");
    var moving = sm.addState(
        runRepeatedly(() -> {
          setpoint = Math.min(MAX_EXTENSION, setpoint + MANUAL_SPEED_MPS * 0.02);
          driveTowardSetpoint();
        }).named("Extender Out"));
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Extender Out");
  }

  /** Retracts inward smoothly while held, stopping at MIN_EXTENSION. */
  public Command in() {
    StateMachine sm = new StateMachine("Extender In");
    var moving = sm.addState(
        runRepeatedly(() -> {
          setpoint = Math.max(MIN_EXTENSION, setpoint - MANUAL_SPEED_MPS * 0.02);
          driveTowardSetpoint();
        }).named("Extender In"));
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Extender In");
  }

  /** Holds the current position. This is the default command. */
  public Command hold() {
    StateMachine sm = new StateMachine("Extender Hold [FSM]");
    var holding = sm.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          driveTowardSetpoint();
        }).named("Extender Hold"));
    holding.onEnter(() -> {
      if (Math.abs(getPosition() - setpoint) > 0.08) {
        setpoint = getPosition();
      }
    });
    sm.setInitialState(holding);
    return run(coroutine -> coroutine.await(sm))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Extender Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    double volts = pid.calculate(getPosition(), setpoint);
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    Double pos = encoder.getPosition().get();
    return pos != null ? pos : STARTING_EXTENSION;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    return val != null ? val * 12.0 : 0.0;
  }

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    return vel != null ? vel : 0.0;
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
