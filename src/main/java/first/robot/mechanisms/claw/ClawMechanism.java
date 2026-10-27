package first.robot.mechanisms.claw;

import static first.robot.mechanisms.claw.ClawConstants.*;

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
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.controller.PIDController;

/** The claw opens and closes its fingers to pick up a ball. */
public class ClawMechanism implements Mechanism {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput ballSensor;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private double setpoint = STARTING_ANGLE;
  private double targetAngle = STARTING_ANGLE;
  private boolean requestOpen = false;
  private boolean requestClose = false;
  private boolean requestMove = false;

  /** True while the claw is holding a ball. */
  public final Trigger hasBall = new Trigger(this::hasBall);

  public ClawMechanism() {
    this(
        Mechanical.createSparkMax(Schematic.CLAW_CAN_ID, MotorType.kBrushless),
        Mechanical.createDigitalInput(Schematic.CLAW_BALL_SENSOR_DIO));
  }

  public ClawMechanism(SparkMax motor, DigitalInput ballSensor) {
    this.motor = motor;
    this.encoder = motor.getEncoder();
    this.ballSensor = ballSensor;
    Mechanical.register(motor);
    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------------------------------

  /**
   * Constructs the full lifecycle state machine for the claw mechanism, managing transitions
   * between holding, opening, closing, gripping, and moving.
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Claw");

    StateMachine.State holding = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getAngle();
          }
          driveTowardSetpoint();
        }).named("Claw Holding"));
    holding.onEnter(() -> {
      if (!requestOpen && !requestClose && !requestMove) {
        setpoint = getAngle();
      }
    });

    StateMachine.State opening = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Opening"));
    opening.onEnter(() -> setpoint = OPEN_ANGLE);
    opening.onExit(() -> requestOpen = false);

    StateMachine.State closing = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Closing"));
    closing.onEnter(() -> setpoint = CLOSED_ANGLE);
    closing.onExit(() -> requestClose = false);

    StateMachine.State gripping = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            motor.setVoltage(0.0);
          } else {
            motor.setVoltage(GRIP_VOLTS);
          }
        }).named("Claw Gripping"));
    gripping.onEnter(() -> {
      requestClose = false;
      requestMove = false;
    });

    StateMachine.State moving = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Moving"));
    moving.onEnter(() -> setpoint = targetAngle);
    moving.onExit(() -> requestMove = false);

    // Transitions from holding
    holding.switchTo(opening).when(() -> requestOpen);
    holding.switchTo(closing).when(() -> requestClose);
    holding.switchTo(moving).when(() -> requestMove);
    holding.switchTo(gripping).when(this::hasBall);

    // Transitions from opening
    opening.switchTo(holding).when(this::atSetpoint);
    opening.switchTo(closing).when(() -> requestClose);
    opening.switchTo(moving).when(() -> requestMove);

    // Transitions from closing
    closing.switchTo(gripping).when(this::hasBall);
    closing.switchTo(holding).when(this::atSetpoint);
    closing.switchTo(opening).when(() -> requestOpen);
    closing.switchTo(moving).when(() -> requestMove);

    // Transitions from moving
    moving.switchTo(gripping).when(this::hasBall);
    moving.switchTo(holding).when(this::atSetpoint);
    moving.switchTo(opening).when(() -> requestOpen);
    moving.switchTo(closing).when(() -> requestClose);

    // Transitions from gripping
    gripping.switchTo(opening).when(() -> requestOpen);
    gripping.switchTo(holding).when(() -> !hasBall());
    gripping.switchTo(moving).when(() -> requestMove);

    machine.setInitialState(hasBall() ? gripping : holding);
    return machine;
  }

  // ------------------------------------------------------------------------------------------
  // Commands and Request Triggers
  // ------------------------------------------------------------------------------------------

  public void requestOpen() {
    this.targetAngle = OPEN_ANGLE;
    this.setpoint = OPEN_ANGLE;
    this.requestOpen = true;
    this.requestClose = false;
    this.requestMove = false;
    driveTowardSetpoint();
  }

  public void requestClose() {
    this.targetAngle = CLOSED_ANGLE;
    this.setpoint = CLOSED_ANGLE;
    this.requestClose = true;
    this.requestOpen = false;
    this.requestMove = false;
    driveTowardSetpoint();
  }

  public void requestMove(double angle) {
    this.targetAngle = Math.clamp(angle, CLOSED_ANGLE, OPEN_ANGLE);
    this.setpoint = this.targetAngle;
    this.requestMove = true;
    this.requestOpen = false;
    this.requestClose = false;
    driveTowardSetpoint();
  }

  /**
   * Moves the fingers to an angle and finishes once they get there.
   *
   * @param angleRadians 0 is closed, {@link ClawConstants#OPEN_ANGLE} is fully open
   */
  public Command moveTo(double angleRadians) {
    double target = Math.clamp(angleRadians, CLOSED_ANGLE, OPEN_ANGLE);
    String name = String.format("Claw To %.0f deg", Math.toDegrees(target));
    if (Math.abs(getAngle() - target) < TOLERANCE_RADIANS) {
      setpoint = target;
      return Command.noRequirements(coroutine -> {}).named(name);
    }
    requestMove(target);
    return Command.noRequirements(coroutine -> {
      while (requestMove) {
        coroutine.yield();
      }
    }).whenCanceled(() -> requestMove = false).named(name);
  }

  /** Opens the claw all the way. */
  public Command open() {
    if (Math.abs(getAngle() - OPEN_ANGLE) < TOLERANCE_RADIANS) {
      setpoint = OPEN_ANGLE;
      return Command.noRequirements(coroutine -> {}).named("Claw Open");
    }
    requestOpen();
    return Command.noRequirements(coroutine -> {
      while (requestOpen) {
        coroutine.yield();
      }
    }).whenCanceled(() -> requestOpen = false).named("Claw Open");
  }

  /**
   * Closes the claw. Finishes when it is fully closed, or as soon as it grabs a ball (it can't
   * close any further when a ball is in the way!).
   */
  public Command close() {
    if (hasBall() || Math.abs(getAngle() - CLOSED_ANGLE) < TOLERANCE_RADIANS) {
      setpoint = CLOSED_ANGLE;
      return Command.noRequirements(coroutine -> {}).named("Claw Close");
    }
    requestClose();
    return Command.noRequirements(coroutine -> {
      while (requestClose) {
        coroutine.yield();
      }
    }).whenCanceled(() -> requestClose = false).named("Claw Close");
  }

  /**
   * The default command. Runs the claw state machine to hold, grip, open, close, or move.
   */
  public Command hold() {
    return run(coroutine -> coroutine.await(stateMachine()))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Claw Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
    if (RobotBase.isDisabled()) {
      motor.setVoltage(0.0);
      return;
    }
    double volts = pid.calculate(getAngle(), setpoint);
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < TOLERANCE_RADIANS;
  }

  public boolean hasBall() {
    return ballSensor.get();
  }

  public double getAngle() {
    Double pos = encoder.getPosition().get();
    return pos != null ? pos : STARTING_ANGLE;
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

  public DigitalInput getBallSensor() {
    return ballSensor;
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
