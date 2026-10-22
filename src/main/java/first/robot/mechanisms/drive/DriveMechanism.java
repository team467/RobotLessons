/**
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  DONT EDIT THIS FILE FOR THE 467 SIM BOT
 * 
 * 
 * 
 * 
 * 
 * 
 */
package first.robot.mechanisms.drive;

import static first.robot.mechanisms.drive.DriveConstants.*;

import java.util.List;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;

/** The drivetrain moves the robot across the field between the start location and the table. */
public class DriveMechanism implements Mechanism {
  private final DriveIO io;
  private final DriveIO.DriveInputs inputs = new DriveIO.DriveInputs();
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = START_POSITION;

  /** True while the robot is within tolerance of its target position. */
  public final Trigger atSetpoint = new Trigger(this::atSetpoint);

  public DriveMechanism(DriveIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(io::updateSim);
    Scheduler.getDefault().addPeriodic(() -> io.updateInputs(inputs));
    io.updateInputs(inputs);
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
    return run(coroutine -> {
          setpoint = targetMeters;
          while (!atSetpoint() || Math.abs(getVelocity()) > 0.08) {
            driveTowardSetpoint();
            coroutine.yield();
          }
          io.setVoltage(0.0);
        })
        .named(name);
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
    return run(coroutine -> {
          while (true) {
            setpoint = Math.min(TABLE_POSITION + 0.5, getPosition() + TELEOP_SPEED_MPS * 0.02);
            driveTowardSetpoint(TELEOP_VOLTS);
            coroutine.yield();
          }
        })
        .named("Drive Forward");
  }

  /** Drives backward towards the start location while active. */
  public Command backward() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.max(START_POSITION - 0.5, getPosition() - TELEOP_SPEED_MPS * 0.02);
            driveTowardSetpoint(TELEOP_VOLTS);
            coroutine.yield();
          }
        })
        .named("Drive Backward");
  }

  /** Drives with a continuous speed or joystick input. */
  public Command drive(DoubleSupplier speedSupplier) {
    return run(coroutine -> {
          while (true) {
            double speed = speedSupplier.getAsDouble();
            if (Math.abs(speed) < 0.05) {
              driveTowardSetpoint();
            } else {
              setpoint = getPosition();
              io.setVoltage(Math.clamp(speed * MAX_VOLTS, -MAX_VOLTS, MAX_VOLTS));
            }
            coroutine.yield();
          }
        })
        .named("Drive Manual");
  }

  /** The default command: holds the robot stationary at its current position. */
  public Command hold() {
    return run(coroutine -> {
          setpoint = getPosition();
          while (true) {
            if (RobotBase.isDisabled()) {
              setpoint = getPosition();
            }
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
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
    io.setVoltage(Math.clamp(volts, -maxVolts, maxVolts));
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < TOLERANCE_METERS;
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public void resetSetpoint() {
    setpoint = getPosition();
  }

  public void resetPosition(double pos) {
    io.resetPosition(pos);
    setpoint = pos;
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }
}
