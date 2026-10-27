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

  /** Drives the robot to a target position on the field and finishes once it arrives. */
  public Command driveTo(Distance target) {
    return driveTo(target, String.format("Drive To %.2fm", target.in(Meters)));
  }

  /** Overload for driving to a target position specified in meters as a double. */
  public Command driveTo(double targetMeters) {
    return driveTo(Meters.of(targetMeters));
  }

  private Command driveTo(Distance target, String name) {
    setpoint = target;
    StateMachine sm = new StateMachine(name);
    var driving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    driving.onEnter(() -> setpoint = target);
    driving.onExit(() -> motor.setVoltage(0.0));
    sm.setInitialState(driving);
    driving.exitStateMachine().when(() -> atSetpoint() && Math.abs(getVelocity()) <= 0.08);
    return run(coroutine -> coroutine.await(sm)).named(name);
  }

  /** Drives to the table where Ball 1 and the target zone are located. */
  public Command driveToTable() {
    return driveTo(TABLE_POSITION, "Drive To Table");
  }

  /** Drives back to the start location where Ball 2 is located on the ground. */
  public Command driveToStart() {
    return driveTo(START_POSITION, "Drive To Start");
  }

  /** Drives forward towards the table while active. */
  public Command forward() {
    StateMachine sm = new StateMachine("Drive Forward");
    var moving = sm.addState(
        runRepeatedly(() -> {
          setpoint = Meters.of(Math.min(TABLE_POSITION.in(Meters) + 0.5, getPosition().in(Meters) + TELEOP_SPEED_MPS * 0.02));
          driveTowardSetpoint(TELEOP_VOLTS);
        }).named("Drive Forward"));
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Drive Forward");
  }

  /** Drives backward towards the start location while active. */
  public Command backward() {
    StateMachine sm = new StateMachine("Drive Backward");
    var moving = sm.addState(
        runRepeatedly(() -> {
          setpoint = Meters.of(Math.max(START_POSITION.in(Meters) - 0.5, getPosition().in(Meters) - TELEOP_SPEED_MPS * 0.02));
          driveTowardSetpoint(TELEOP_VOLTS);
        }).named("Drive Backward"));
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Drive Backward");
  }

  /** Drives with a continuous speed or joystick input. */
  public Command drive(DoubleSupplier speedSupplier) {
    StateMachine sm = new StateMachine("Drive Manual");
    var manual = sm.addState(
        runRepeatedly(() -> {
          double speed = speedSupplier.getAsDouble();
          if (Math.abs(speed) < 0.05) {
            driveTowardSetpoint();
          } else {
            setpoint = getPosition();
            motor.setVoltage(Math.clamp(speed * MAX_VOLTS, -MAX_VOLTS, MAX_VOLTS));
          }
        }).named("Drive Manual"));
    sm.setInitialState(manual);
    return run(coroutine -> coroutine.await(sm)).named("Drive Manual");
  }

  /** The default command: holds the robot stationary at its current position. */
  public Command hold() {
    StateMachine sm = new StateMachine("Drive Hold [FSM]");
    var holding = sm.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getPosition();
          }
          driveTowardSetpoint();
        }).named("Drive Hold"));
    holding.onEnter(() -> setpoint = getPosition());
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
  }

  public void resetPosition(Distance pos) {
    Mechanical.setEncoderPosition(motor, encoder, pos.in(Meters));
    setpoint = pos;
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
