package first.robot.mechanisms.extender;

import static first.robot.mechanisms.extender.ExtenderConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.sim.SimMechanical;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
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
    this(
        RobotBase.isSimulation()
            ? SimMechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless)
            : new SparkMax(CANPort.CAN_S0, Schematic.EXTENDER_CAN_ID, MotorType.kBrushless));
  }

  public ExtenderMechanism(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    if (RobotBase.isSimulation()) {
      SimMechanical.register(motor);
    }
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
    return run(coroutine -> {
          setpoint = target;
          while (!atSetpoint()) {
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named(String.format("Extender To %.2fm", target));
  }

  /** Extends outward smoothly while held, stopping at MAX_EXTENSION. */
  public Command out() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.min(MAX_EXTENSION, setpoint + MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Extender Out");
  }

  /** Retracts inward smoothly while held, stopping at MIN_EXTENSION. */
  public Command in() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.max(MIN_EXTENSION, setpoint - MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Extender In");
  }

  /** Holds the current position. This is the default command. */
  public Command hold() {
    return run(coroutine -> {
          if (Math.abs(getPosition() - setpoint) > 0.08) {
            setpoint = getPosition();
          }
          while (true) {
            if (RobotBase.isDisabled()) {
              setpoint = getPosition();
            }
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
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

  public SparkMax getMotor() {
    return motor;
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
