package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;
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
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

/** The elevator moves the whole arm up and down (the y-axis). */
public class Elevator implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  /** The height we are currently trying to reach. */
  private Distance setpoint = STARTING_HEIGHT;

  /** True while the elevator is within tolerance of its setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public Elevator() {
    this(Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless));
  }

  public Elevator(SparkMax motor) {
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
   * Clamps a requested wrist height within the physical travel limits of the elevator
   * ({@link ElevatorConstants#MIN_HEIGHT} to {@link ElevatorConstants#MAX_HEIGHT}).
   *
   * @param height the raw target distance
   * @return the distance clamped within safe mechanical limits
   */
  private Distance clampHeight(Distance height) {
    if (height.lt(MIN_HEIGHT)) {
      return MIN_HEIGHT;
    } else if (height.gt(MAX_HEIGHT)) {
      return MAX_HEIGHT;
    } else {
      return height;
    }
  }

  /**
   * Moves the elevator to a height and finishes once it gets there.
   *
   * <p>Features:
   * <ul>
   *   <li><b>Safety clamping</b>: Automatically limits targets between 0.25m and 1.20m.
   *   <li><b>Gravity feedforward</b>: Automatically adds {@link ElevatorConstants#KG} (0.70V) in addition
   *       to proportional-derivative closed-loop error correction.
   *   <li><b>Velocity-settled exit</b>: Verifies both that height is within tolerance (&plusmn;0.02m)
   *       AND vertical velocity has slowed to &le; 0.05 m/s before completing.
   * </ul>
   *
   * @param height the target wrist height, clamped to the elevator's travel
   * @return a command that drives the elevator to the target height
   */
  public Command moveTo(Distance height) {
    Distance target = clampHeight(height);
    setpoint = target;
    String name = String.format("Elevator To %.2fm", target.in(Meters));
    StateMachine sm = new StateMachine(name);
    var moving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    moving.onEnter(() -> {
      setpoint = target;
      pid.reset(); // Clear accumulated integral/derivative state
    });
    sm.setInitialState(moving);
    // Complete only when at setpoint and carriage has settled to avoid inertial bounce
    moving.exitStateMachine().when(() -> atSetpoint() && Math.abs(getVelocity()) <= 0.05);
    return run(coroutine -> coroutine.await(sm)).named(name);
  }

  /**
   * Overload for moving the elevator to a height specified in meters as a double.
   *
   * @param heightMeters the target wrist height in meters
   * @return a command that drives the elevator to the target height
   */
  public Command moveTo(double heightMeters) {
    return moveTo(Meters.of(heightMeters));
  }

  /**
   * Drives up smoothly while held, stopping at {@link ElevatorConstants#MAX_HEIGHT}.
   *
   * <p>Increments the setpoint by {@code MANUAL_SPEED_MPS * 0.02s} each periodic cycle.
   * When released, latches the setpoint to current actual position and resets the PID controller
   * so the carriage stops immediately without coasting or hunting.
   *
   * @return upward jogging command
   */
  public Command up() {
    StateMachine sm = new StateMachine("Elevator Up");
    var moving = sm.addState(
        runRepeatedly(() -> {
          // Increment setpoint by 0.35 m/s * 0.02s (7mm per tick) up to MAX_HEIGHT
          setpoint = Meters.of(Math.min(MAX_HEIGHT.in(Meters), setpoint.in(Meters) + MANUAL_SPEED_MPS * 0.02));
          driveTowardSetpoint();
        }).named("Elevator Up"));
    moving.onEnter(() -> setpoint = getPosition());
    moving.onExit(() -> {
      // Lock position where user let go of the control and clear PID history
      setpoint = getPosition();
      pid.reset();
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Elevator Up");
  }

  /**
   * Drives down smoothly while held, stopping at {@link ElevatorConstants#MIN_HEIGHT}.
   *
   * <p>Decrements the setpoint by {@code MANUAL_SPEED_MPS * 0.02s} each periodic cycle.
   * When released, latches the setpoint to current actual position and resets the PID controller
   * so the carriage stops immediately without coasting or hunting.
   *
   * @return downward jogging command
   */
  public Command down() {
    StateMachine sm = new StateMachine("Elevator Down");
    var moving = sm.addState(
        runRepeatedly(() -> {
          // Decrement setpoint by 0.35 m/s * 0.02s (7mm per tick) down to MIN_HEIGHT
          setpoint = Meters.of(Math.max(MIN_HEIGHT.in(Meters), setpoint.in(Meters) - MANUAL_SPEED_MPS * 0.02));
          driveTowardSetpoint();
        }).named("Elevator Down"));
    moving.onEnter(() -> setpoint = getPosition());
    moving.onExit(() -> {
      // Lock position where user let go of the control and clear PID history
      setpoint = getPosition();
      pid.reset();
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Elevator Down");
  }

  /**
   * Holds the elevator at the current height. This is the default command, so it runs whenever
   * nothing else needs the elevator.
   *
   * <p>Applies gravity feedforward {@link ElevatorConstants#KG} (0.70V) plus PID feedback to counteract
   * the carriage weight and maintain steady position.
   *
   * @return continuous default holding command
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
      if (RobotBase.isDisabled()) {
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

  /**
   * Applies voltage calculated from gravity feedforward plus closed-loop position error:
   * {@code volts = KG + PID(position, setpoint)}, clamped between [-MAX_VOLTS, +MAX_VOLTS].
   */
  private void driveTowardSetpoint() {
    double volts = KG + pid.calculate(getPosition().in(Meters), setpoint.in(Meters));
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, TOLERANCE);
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_HEIGHT;
    }
    return Meters.of(pos);
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

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    if (vel == null) {
      return 0.0;
    }
    return vel;
  }

  /** The name of the most specific command currently using the elevator. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_HEIGHT;
    pid.reset();
  }
}
