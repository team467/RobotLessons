package first.robot.subsystems.claw;

import static first.robot.subsystems.claw.ClawConstants.*;

import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;

/** The claw opens and closes its fingers to pick up a ball. */
public class ClawMechanism implements Mechanism {
  private final ClawIO io;
  private final ClawIO.ClawInputs inputs = new ClawIO.ClawInputs();
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = STARTING_ANGLE;

  /** True while the claw is holding a ball. */
  public final Trigger hasBall = new Trigger(this::hasBall);

  public ClawMechanism(ClawIO io) {
    this.io = io;
    setDefaultCommand(hold());
  }

  /** Call once per robot loop, before the scheduler runs, to refresh sensor readings. */
  public void periodic() {
    io.updateInputs(inputs);
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Moves the fingers to an angle and finishes once they get there.
   *
   * @param angleRadians 0 is closed, {@link ClawConstants#OPEN_ANGLE} is fully open
   */
  public Command moveTo(double angleRadians) {
    double target = Math.clamp(angleRadians, CLOSED_ANGLE, OPEN_ANGLE);
    return run(coroutine -> {
          setpoint = target;
          while (!atSetpoint()) {
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named(String.format("Claw To %.0f deg", Math.toDegrees(target)));
  }

  /** Opens the claw all the way. */
  public Command open() {
    return moveTo(OPEN_ANGLE);
  }

  /**
   * Closes the claw. Finishes when it is fully closed, or as soon as it grabs a ball (it can't
   * close any further when a ball is in the way!).
   */
  public Command close() {
    return run(coroutine -> {
          setpoint = CLOSED_ANGLE;
          while (!atSetpoint() && !hasBall()) {
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Claw Close");
  }

  /**
   * The default command. Keeps squeezing if we have a ball, otherwise holds the current angle.
   */
  public Command hold() {
    return run(coroutine -> {
          setpoint = getAngle();
          while (true) {
            if (RobotBase.isDisabled()) {
              setpoint = getAngle();
            }
            if (hasBall()) {
              io.setVoltage(GRIP_VOLTS);
            } else {
              driveTowardSetpoint();
            }
            coroutine.yield();
          }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Claw Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    double volts = pid.calculate(getAngle(), setpoint);
    io.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < TOLERANCE_RADIANS;
  }

  public boolean hasBall() {
    return inputs.hasBall;
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  /** The name of the most specific command currently using the claw. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    return running.isEmpty() ? "(none)" : running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_ANGLE;
  }
}
