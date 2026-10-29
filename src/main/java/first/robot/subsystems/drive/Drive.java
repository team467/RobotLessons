package first.robot.subsystems.drive;

import static first.robot.subsystems.drive.DriveConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.LinearPositionMechanism;
import first.robot.mechanical.Mechanical;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Distance;

/** The drivetrain moves the robot across the field between the start location and the table. */
public class Drive implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final LinearPositionMechanism mechanism;

  /** True while the robot is within tolerance of its target position. */
  public final Trigger atSetpoint;

  public Drive() {
    this(Mechanical.createSparkMax(Schematic.DRIVE_CAN_ID, MotorType.kBrushless));
  }

  public Drive(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    Mechanical.register(motor);

    this.mechanism =
        new LinearPositionMechanism(
            "Drive",
            motor::setVoltage,
            this::getPosition,
            this::getVelocity,
            KP,
            KD,
            MAX_VOLTS,
            TOLERANCE,
            Meters.of(0.0),
            Meters.of(3.0),
            START_POSITION);

    this.atSetpoint = mechanism.atSetpoint;
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command driveTo(Distance target) {
    return driveTo(target, String.format("Drive To %.2fm", target.in(Meters)));
  }

  public Command driveTo(double targetMeters) {
    return driveTo(Meters.of(targetMeters));
  }

  private Command driveTo(Distance target, String name) {
    return mechanism.moveTo(this, target, name);
  }

  public Command driveToTable() {
    return driveTo(TABLE_POSITION, "Drive to Table");
  }

  public Command driveToStart() {
    return driveTo(START_POSITION, "Drive to Start");
  }

  public Command forward() {
    StateMachine sm = new StateMachine("Drive Forward");
    var moving = sm.addState(
        runRepeatedly(() -> {
          double nextMeters = Math.min(TABLE_POSITION.in(Meters), mechanism.getSetpoint().in(Meters) + TELEOP_SPEED_MPS * 0.02);
          mechanism.setSetpoint(Meters.of(nextMeters));
          mechanism.driveTowardSetpoint(TELEOP_VOLTS);
        }).named("Drive Forward"));
    moving.onEnter(() -> mechanism.setSetpoint(getPosition()));
    moving.onExit(() -> {
      mechanism.setSetpoint(getPosition());
      mechanism.resetPid();
      motor.setVoltage(0.0);
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Drive Forward");
  }

  public Command backward() {
    StateMachine sm = new StateMachine("Drive Backward");
    var moving = sm.addState(
        runRepeatedly(() -> {
          double nextMeters = Math.max(START_POSITION.in(Meters), mechanism.getSetpoint().in(Meters) - TELEOP_SPEED_MPS * 0.02);
          mechanism.setSetpoint(Meters.of(nextMeters));
          mechanism.driveTowardSetpoint(TELEOP_VOLTS);
        }).named("Drive Backward"));
    moving.onEnter(() -> mechanism.setSetpoint(getPosition()));
    moving.onExit(() -> {
      mechanism.setSetpoint(getPosition());
      mechanism.resetPid();
      motor.setVoltage(0.0);
    });
    sm.setInitialState(moving);
    return run(coroutine -> coroutine.await(sm)).named("Drive Backward");
  }

  public Command drive(DoubleSupplier speedSupplier) {
    StateMachine sm = new StateMachine("Drive Manual");
    var manual = sm.addState(
        runRepeatedly(() -> {
          double speed = speedSupplier.getAsDouble();
          if (Math.abs(speed) < 0.05) {
            mechanism.driveTowardSetpoint();
          } else {
            mechanism.setSetpoint(getPosition());
            motor.setVoltage(Math.clamp(speed * MAX_VOLTS, -MAX_VOLTS, MAX_VOLTS));
          }
        }).named("Drive Manual"));
    manual.onExit(() -> {
      mechanism.setSetpoint(getPosition());
      mechanism.resetPid();
      motor.setVoltage(0.0);
    });
    sm.setInitialState(manual);
    return run(coroutine -> coroutine.await(sm)).named("Drive Manual");
  }

  public Command hold() {
    return mechanism.hold(this);
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean atSetpoint() {
    return mechanism.atSetpoint();
  }

  public Distance getPosition() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return START_POSITION;
    }
    return Meters.of(pos);
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

  public double getVelocity() {
    Double vel = encoder.getVelocity().get();
    if (vel == null) {
      return 0.0;
    }
    return vel;
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

  public void resetPosition(Distance pos) {
    Mechanical.setEncoderPosition(motor, encoder, pos.in(Meters));
    mechanism.setSetpoint(pos);
    mechanism.resetPid();
  }

  public void resetPosition(double pos) {
    resetPosition(Meters.of(pos));
  }

  public LinearPositionMechanism getMechanism() {
    return mechanism;
  }
}
