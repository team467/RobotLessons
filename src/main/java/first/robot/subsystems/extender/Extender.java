package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;
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

/** The extender slides the claw in and out on a pulley (the x-axis). */
public class Extender implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private Distance setpoint = STARTING_EXTENSION;

  /** True while the extender is within tolerance of its setpoint. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public Extender() {
    this(Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless));
  }

  public Extender(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Clamps a requested extension distance within the physical travel limits of the horizontal slide
   * ({@link ExtenderConstants#MIN_EXTENSION} = 0.0m to {@link ExtenderConstants#MAX_EXTENSION} = 1.10m).
   *
   * @param extension the raw target distance
   * @return the distance clamped within mechanical bounds
   */
  private Distance clampExtension(Distance extension) {
    if (extension.lt(MIN_EXTENSION)) {
      return MIN_EXTENSION;
    } else if (extension.gt(MAX_EXTENSION)) {
      return MAX_EXTENSION;
    } else {
      return extension;
    }
  }

  /**
   * Moves the extender to a position and finishes once it gets there.
   *
   * <p>Features:
   * <ul>
   *   <li><b>Safety clamping</b>: Constrains targets between 0.0m (fully retracted) and 1.10m (fully extended).
   *   <li><b>PID reset on entry</b>: Clears prior error history via {@link PIDController#reset()} to eliminate windup.
   *   <li><b>Velocity-settled exit</b>: Ensures the slide is both within tolerance (&plusmn;0.02m) AND
   *       linear speed is &le; 0.05 m/s, preventing inertial overshoot before subsequent actions run.
   * </ul>
   *
   * @param extension how far to extend, clamped to the slide's travel
   * @return a command that drives the extender to the target position
   */
  public Command moveTo(Distance extension) {
    Distance target = clampExtension(extension);
    setpoint = target;
    String name = String.format("Extender To %.2fm", target.in(Meters));
    StateMachine sm = new StateMachine(name);
    var moving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    moving.onEnter(() -> {
      setpoint = target;
      pid.reset(); // Clear accumulated integral/derivative state
    });
    sm.setInitialState(moving);
    // Complete only when at setpoint and slide has settled to prevent overshoot
    moving.exitStateMachine().when(() -> atSetpoint() && Math.abs(getVelocity()) <= 0.05);
    return run(coroutine -> coroutine.await(sm)).named(name);
  }

  /**
   * Overload for moving the extender to a position specified in meters as a double.
   *
   * @param extensionMeters how far to extend in meters
   * @return a command that drives the extender to the target position
   */
  public Command moveTo(double extensionMeters) {
    return moveTo(Meters.of(extensionMeters));
  }

  /**
   * Extends outward smoothly while held, stopping at {@link ExtenderConstants#MAX_EXTENSION}.
   *
   * <p>Increments the setpoint by {@code MANUAL_SPEED_MPS * 0.02s} (10mm per tick) each 20ms cycle.
   * When released, immediately freezes the setpoint to current actual position and resets PID
   * so the slide stops dead without bouncing or creeping.
   *
   * @return outward jogging command
   */
  public Command out() {
    StateMachine sm = new StateMachine("Extender Out");
    var moving = sm.addState(
        runRepeatedly(() -> {
          // Increment setpoint by 0.50 m/s * 0.02s (10mm per tick) up to MAX_EXTENSION
          setpoint = Meters.of(Math.min(MAX_EXTENSION.in(Meters), setpoint.in(Meters) + MANUAL_SPEED_MPS * 0.02));
          driveTowardSetpoint();
        }).named("Extender Out"));
    moving.onEnter(() -> setpoint = getPosition());
    moving.onExit(() -> {
      // Lock position where user let go of the control and clear PID history
      setpoint = getPosition();
      pid.reset();
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Extender Out");
  }

  /**
   * Retracts inward smoothly while held, stopping at {@link ExtenderConstants#MIN_EXTENSION}.
   *
   * <p>Decrements the setpoint by {@code MANUAL_SPEED_MPS * 0.02s} (10mm per tick) each 20ms cycle.
   * When released, immediately freezes the setpoint to current actual position and resets PID
   * so the slide stops dead without bouncing or creeping.
   *
   * @return inward jogging command
   */
  public Command in() {
    StateMachine sm = new StateMachine("Extender In");
    var moving = sm.addState(
        runRepeatedly(() -> {
          // Decrement setpoint by 0.50 m/s * 0.02s (10mm per tick) down to MIN_EXTENSION
          setpoint = Meters.of(Math.max(MIN_EXTENSION.in(Meters), setpoint.in(Meters) - MANUAL_SPEED_MPS * 0.02));
          driveTowardSetpoint();
        }).named("Extender In"));
    moving.onEnter(() -> setpoint = getPosition());
    moving.onExit(() -> {
      // Lock position where user let go of the control and clear PID history
      setpoint = getPosition();
      pid.reset();
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Extender In");
  }

  /**
   * Holds the current position. This is the default command.
   *
   * <p>Runs at {@link Command#LOWEST_PRIORITY} so that any manual jogging or autonomous motions
   * take precedence and interrupt this hold automatically.
   *
   * @return continuous default holding command
   */
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
      if (RobotBase.isDisabled()) {
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

  /**
   * Calculates motor voltage from closed-loop PID position error:
   * {@code volts = PID(position, setpoint)}, clamped between [-MAX_VOLTS, +MAX_VOLTS].
   */
  private void driveTowardSetpoint() {
    double volts = pid.calculate(getPosition().in(Meters), setpoint.in(Meters));
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, TOLERANCE);
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_EXTENSION;
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

  /** The name of the most specific command currently using the extender. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_EXTENSION;
    pid.reset();
  }
}
