package first.robot.mechanisms.elevator;

import static first.robot.mechanisms.elevator.ElevatorConstants.*;

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

/** The elevator moves the whole arm up and down (the y-axis). */
public class ElevatorMechanism implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  /** The height we are currently trying to reach. */
  private double setpoint = STARTING_HEIGHT;

  /** True while the elevator is within tolerance of its setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public ElevatorMechanism() {
    this(Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless));
  }

  public ElevatorMechanism(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);
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
    setpoint = target;
    String name = String.format("Elevator To %.2fm", target);
    StateMachine sm = new StateMachine(name);
    var moving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    moving.onEnter(() -> setpoint = target);
    sm.setInitialState(moving);
    moving.exitStateMachine().when(this::atSetpoint);
    return run(coroutine -> coroutine.await(sm)).named(name);
  }

  /** Drives up smoothly while held, stopping at MAX_HEIGHT. */
  public Command up() {
    StateMachine sm = new StateMachine("Elevator Up");
    var moving = sm.addState(
        runRepeatedly(() -> {
          setpoint = Math.min(MAX_HEIGHT, setpoint + MANUAL_SPEED_MPS * 0.02);
          driveTowardSetpoint();
        }).named("Elevator Up"));
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Elevator Up");
  }

  /** Drives down smoothly while held, stopping at MIN_HEIGHT. */
  public Command down() {
    StateMachine sm = new StateMachine("Elevator Down");
    var moving = sm.addState(
        runRepeatedly(() -> {
          setpoint = Math.max(MIN_HEIGHT, setpoint - MANUAL_SPEED_MPS * 0.02);
          driveTowardSetpoint();
        }).named("Elevator Down"));
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Elevator Down");
  }

  /**
   * Holds the elevator at the current height. This is the default command, so it runs whenever
   * nothing else needs the elevator.
   */
  public Command hold() {
    StateMachine sm = new StateMachine("Elevator Hold [FSM]");
    var holding = sm.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          driveTowardSetpoint();
        }).named("Elevator Hold"));
    holding.onEnter(() -> {
      if (Math.abs(getPosition() - setpoint) > 0.08) {
        setpoint = getPosition();
      }
    });
    sm.setInitialState(holding);
    return run(coroutine -> coroutine.await(sm))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Elevator Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    double volts = KG + pid.calculate(getPosition(), setpoint);
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    Double pos = encoder.getPosition().get();
    return pos != null ? pos : STARTING_HEIGHT;
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

  /** The name of the most specific command currently using the elevator. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_HEIGHT;
  }
}
