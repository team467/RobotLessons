package first.robot.mechanisms.elevator;

import static first.robot.mechanisms.elevator.ElevatorConstants.*;

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
    this(
        RobotBase.isSimulation()
            ? SimMechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless)
            : new SparkMax(CANPort.CAN_S0, Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless));
  }

  public ElevatorMechanism(SparkMax motor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    if (RobotBase.isSimulation()) {
      SimMechanical.register(motor);
    }
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
    return run(coroutine -> {
          setpoint = target;
          while (!atSetpoint()) {
            driveTowardSetpoint();
            coroutine.yield(); // wait for the next robot loop
          }
        })
        .named(String.format("Elevator To %.2fm", target));
  }

  /** Drives up smoothly while held, stopping at MAX_HEIGHT. */
  public Command up() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.min(MAX_HEIGHT, setpoint + MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Elevator Up");
  }

  /** Drives down smoothly while held, stopping at MIN_HEIGHT. */
  public Command down() {
    return run(coroutine -> {
          while (true) {
            setpoint = Math.max(MIN_HEIGHT, setpoint - MANUAL_SPEED_MPS * 0.02);
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .named("Elevator Down");
  }

  /**
   * Holds the elevator at the current height. This is the default command, so it runs whenever
   * nothing else needs the elevator.
   */
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

  public SparkMax getMotor() {
    return motor;
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
