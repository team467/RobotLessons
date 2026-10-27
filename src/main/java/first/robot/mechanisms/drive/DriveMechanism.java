package first.robot.mechanisms.drive;

import static first.robot.mechanisms.drive.DriveConstants.*;

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

/** The drivetrain moves the robot across the field between the start location and the table. */
public class DriveMechanism implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = START_POSITION;

  /** True while the robot is within tolerance of its target position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public DriveMechanism() {
    this(Mechanical.createSparkMax(Schematic.DRIVE_CAN_ID, MotorType.kBrushless));
  }

  public DriveMechanism(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /** Drives the robot to a target position on the field and finishes once it arrives. */
  public Command driveTo(double targetMeters) {
    return driveTo(targetMeters, String.format("Drive To %.2fm", targetMeters));
  }

  private Command driveTo(double targetMeters, String name) {
    setpoint = targetMeters;
    StateMachine sm = new StateMachine(name);
    var driving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    driving.onEnter(() -> setpoint = targetMeters);
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
          setpoint = Math.min(TABLE_POSITION + 0.5, getPosition() + TELEOP_SPEED_MPS * 0.02);
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
          setpoint = Math.max(START_POSITION - 0.5, getPosition() - TELEOP_SPEED_MPS * 0.02);
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
    double pos = getPosition();
    double volts = pid.calculate(pos, setpoint);
    motor.setVoltage(Math.clamp(volts, -maxVolts, maxVolts));
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    Double pos = encoder.getPosition().get();
    return pos != null ? pos : START_POSITION;
  }

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    return vel != null ? vel : 0.0;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    Double val = motor.getAppliedOutput().get();
    return val != null ? val * 12.0 : 0.0;
  }

  public void resetSetpoint() {
    setpoint = getPosition();
  }

  public void resetPosition(double pos) {
    Mechanical.setEncoderPosition(motor, encoder, pos);
    setpoint = pos;
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }
}
