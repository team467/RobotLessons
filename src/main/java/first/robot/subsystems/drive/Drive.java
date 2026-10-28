package first.robot.subsystems.drive;

import static first.robot.subsystems.drive.DriveConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

/** The drivetrain moves the robot across the field between the start location and the table. */
public class Drive implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private Distance setpoint = START_POSITION;

  /** True while the robot is within tolerance of its target position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public Drive() {
    this(Mechanical.createSparkMax(Schematic.DRIVE_CAN_ID, MotorType.kBrushless));
  }

  public Drive(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Drives the robot to a target position on the field and finishes once it arrives.
   *
   * @param target the target field position distance
   * @return a command that moves the drivetrain to the target
   */
  public Command driveTo(Distance target) {
    return driveTo(target, String.format("Drive To %.2fm", target.in(Meters)));
  }

  /**
   * Overload for driving to a target position specified in meters as a double.
   *
   * @param targetMeters the target position in meters
   * @return a command that moves the drivetrain to the target
   */
  public Command driveTo(double targetMeters) {
    return driveTo(Meters.of(targetMeters));
  }

  /**
   * Internal implementation of point-to-point driving using a single-state StateMachine.
   *
   * <p>Features:
   * <ul>
   *   <li><b>Entry latching</b>: Sets the setpoint and clears PID history via {@link PIDController#reset()}
   *       to ensure no stale integral or derivative transients carry over.
   *   <li><b>Velocity-settled exit</b>: Exits only when the chassis is both within {@link #TOLERANCE} (0.03m)
   *       AND linear velocity has settled to &le; 0.05 m/s, preventing premature completion while still coasting.
   * </ul>
   *
   * @param target target distance
   * @param name command display name
   * @return coroutine command driving to the destination
   */
  private Command driveTo(Distance target, String name) {
    setpoint = target;
    StateMachine sm = new StateMachine(name);
    var driving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    driving.onEnter(() -> {
      setpoint = target;
      pid.reset(); // Clear accumulated integral and derivative error from prior moves
    });
    sm.setInitialState(driving);
    // Ensure both position within tolerance and chassis has settled to near-zero speed
    driving.exitStateMachine().when(() -> atSetpoint() && Math.abs(getVelocity()) <= 0.05);
    return run(coroutine -> coroutine.await(sm)).named(name);
  }

  /**
   * Drives to the table where Ball 1 and the target zone are located ({@link DriveConstants#TABLE_POSITION}).
   *
   * @return a command driving the chassis to the table
   */
  public Command driveToTable() {
    return driveTo(TABLE_POSITION, "Drive To Table");
  }

  /**
   * Drives back to the start location where Ball 2 is located on the ground ({@link DriveConstants#START_POSITION}).
   *
   * @return a command driving the chassis back to the starting line
   */
  public Command driveToStart() {
    return driveTo(START_POSITION, "Drive To Start");
  }

  /**
   * Drives forward towards the table while active (e.g. held by keyboard 'D' or gamepad).
   *
   * <p>Mechanics:
   * <ul>
   *   <li>Ramps the setpoint forward at {@link DriveConstants#TELEOP_SPEED_MPS} (5.5 m/s) clamped at {@link DriveConstants#TABLE_POSITION}.
   *   <li>Applies boosted {@link DriveConstants#TELEOP_VOLTS} (11.5V) for responsive manual jogging.
   *   <li><b>On release</b>: Latches the setpoint immediately to the current actual position, resets PID,
   *       and zeros motor output to hold still without rubber-banding or overshoot.
   * </ul>
   *
   * @return continuous forward jogging command
   */
  public Command forward() {
    StateMachine sm = new StateMachine("Drive Forward");
    var moving = sm.addState(
        runRepeatedly(() -> {
          // Increment setpoint by distance traveled in one 20ms tick, clamped to table boundary
          setpoint = Meters.of(Math.min(TABLE_POSITION.in(Meters), getPosition().in(Meters) + TELEOP_SPEED_MPS * 0.02));
          driveTowardSetpoint(TELEOP_VOLTS);
        }).named("Drive Forward"));
    moving.onExit(() -> {
      // Freeze setpoint at current position on key release to lock position without rollback
      setpoint = getPosition();
      pid.reset();
      motor.setVoltage(0.0);
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Drive Forward");
  }

  /**
   * Drives backward towards the start location while active (e.g. held by keyboard 'A' or gamepad).
   *
   * <p>Mechanics:
   * <ul>
   *   <li>Ramps the setpoint backward at {@link DriveConstants#TELEOP_SPEED_MPS} (5.5 m/s) clamped at {@link DriveConstants#START_POSITION}.
   *   <li>Applies boosted {@link DriveConstants#TELEOP_VOLTS} (11.5V) for responsive manual jogging.
   *   <li><b>On release</b>: Latches the setpoint immediately to the current actual position, resets PID,
   *       and zeros motor output to hold still without rubber-banding or overshoot.
   * </ul>
   *
   * @return continuous backward jogging command
   */
  public Command backward() {
    StateMachine sm = new StateMachine("Drive Backward");
    var moving = sm.addState(
        runRepeatedly(() -> {
          // Decrement setpoint by distance traveled in one 20ms tick, clamped to start boundary
          setpoint = Meters.of(Math.max(START_POSITION.in(Meters), getPosition().in(Meters) - TELEOP_SPEED_MPS * 0.02));
          driveTowardSetpoint(TELEOP_VOLTS);
        }).named("Drive Backward"));
    moving.onExit(() -> {
      // Freeze setpoint at current position on key release to lock position without rollback
      setpoint = getPosition();
      pid.reset();
      motor.setVoltage(0.0);
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Drive Backward");
  }

  /**
   * Drives with a continuous speed or analog joystick input.
   *
   * <p>When input magnitude is within the 5% deadband (&lt; 0.05), closed-loop position holding
   * maintains the robot's current position. When deflected beyond deadband, direct open-loop
   * voltage drives the chassis while continuously synchronizing the setpoint to current position.
   *
   * @param speedSupplier normalized speed input (-1.0 to 1.0)
   * @return analog driving command
   */
  public Command drive(DoubleSupplier speedSupplier) {
    StateMachine sm = new StateMachine("Drive Manual");
    var manual = sm.addState(
        runRepeatedly(() -> {
          double speed = speedSupplier.getAsDouble();
          // Deadband check: if joystick is centered, hold position
          if (Math.abs(speed) < 0.05) {
            driveTowardSetpoint();
          } else {
            // Actively driving: track position so release stops smoothly where the robot is
            setpoint = getPosition();
            motor.setVoltage(Math.clamp(speed * MAX_VOLTS, -MAX_VOLTS, MAX_VOLTS));
          }
        }).named("Drive Manual"));
    manual.onExit(() -> {
      setpoint = getPosition();
      pid.reset();
      motor.setVoltage(0.0);
    });
    sm.setInitialState(manual);
    return run(coroutine -> coroutine.await(sm)).named("Drive Manual");
  }

  /**
   * The default command: holds the robot stationary at its current position using closed-loop PID.
   *
   * <p>Runs at {@link Command#LOWEST_PRIORITY} so that any manual driving or autonomous trajectory
   * takes precedence when active.
   *
   * @return continuous default holding command
   */
  public Command hold() {
    StateMachine sm = new StateMachine("Drive Hold [FSM]");
    var holding = sm.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          driveTowardSetpoint();
        }).named("Drive Hold"));
    holding.onEnter(() -> {
      if (RobotBase.isDisabled()) {
        setpoint = getPosition();
      }
    });
    sm.setInitialState(holding);
    return run(coroutine -> coroutine.await(sm))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Drive Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    driveTowardSetpoint(MAX_VOLTS);
  }

  private void driveTowardSetpoint(double maxVolts) {
    double pos = getPosition().in(Meters);
    double volts = pid.calculate(pos, setpoint.in(Meters));
    motor.setVoltage(Math.clamp(volts, -maxVolts, maxVolts));
  }

  public boolean atSetpoint() {
    return getPosition().isNear(setpoint, TOLERANCE);
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return START_POSITION;
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

  public void resetSetpoint() {
    setpoint = getPosition();
    pid.reset();
  }

  public void resetPosition(Distance pos) {
    Mechanical.setEncoderPosition(motor, encoder, pos.in(Meters));
    setpoint = pos;
    pid.reset();
  }

  public void resetPosition(double pos) {
    resetPosition(Meters.of(pos));
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }
}
