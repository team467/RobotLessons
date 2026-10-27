package first.robot.subsystems.claw;

import static first.robot.subsystems.claw.ClawConstants.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

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
import org.wpilib.units.measure.Angle;

/** The claw opens and closes its fingers to pick up a ball. */
public class Claw implements Mechanism {
  /** The target goal/intent for the claw state machine. */
  public enum Goal {
    HOLD,
    OPEN,
    CLOSE,
    MOVE
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput ballSensor;
  private final PIDController pid = new PIDController(KP, 0.0, KD);

  private Angle setpoint = STARTING_ANGLE;
  private Goal goal = Goal.HOLD;
  private Angle targetAngle = STARTING_ANGLE;

  /** True while the claw is holding a ball. */
  public final Trigger hasBall = new Trigger(this::hasBall);

  public Claw() {
    this(
        Mechanical.createSparkMax(Schematic.CLAW_CAN_ID, MotorType.kBrushless),
        Mechanical.createDigitalInput(Schematic.CLAW_BALL_SENSOR_DIO));
  }

  public Claw(SparkMax motor, DigitalInput ballSensor) {
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
      if (goal == Goal.HOLD) {
        setpoint = getAngle();
      }
    });

    StateMachine.State opening = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Opening"));
    opening.onEnter(() -> setpoint = OPEN_ANGLE);

    StateMachine.State closing = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Closing"));
    closing.onEnter(() -> setpoint = CLOSED_ANGLE);

    StateMachine.State gripping = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            motor.setVoltage(0.0);
          } else {
            motor.setVoltage(GRIP_VOLTS);
          }
        }).named("Claw Gripping"));

    StateMachine.State moving = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Moving"));
    moving.onEnter(() -> setpoint = targetAngle);

    // Transitions from holding
    holding.switchTo(gripping).when(this::hasBall);
    holding.switchTo(opening).when(() -> goal == Goal.OPEN && !atSetpoint());
    holding.switchTo(closing).when(() -> goal == Goal.CLOSE && !atSetpoint());
    holding.switchTo(moving).when(() -> goal == Goal.MOVE && !atSetpoint());

    // Transitions from opening
    opening.switchTo(holding).when(this::atSetpoint);
    opening.switchTo(closing).when(() -> goal == Goal.CLOSE);
    opening.switchTo(moving).when(() -> goal == Goal.MOVE);

    // Transitions from closing
    closing.switchTo(gripping).when(this::hasBall);
    closing.switchTo(holding).when(this::atSetpoint);
    closing.switchTo(opening).when(() -> goal == Goal.OPEN);
    closing.switchTo(moving).when(() -> goal == Goal.MOVE);

    // Transitions from moving
    moving.switchTo(gripping).when(this::hasBall);
    moving.switchTo(holding).when(this::atSetpoint);
    moving.switchTo(opening).when(() -> goal == Goal.OPEN);
    moving.switchTo(closing).when(() -> goal == Goal.CLOSE);

    // Transitions from gripping
    gripping.switchTo(opening).when(() -> goal == Goal.OPEN);
    gripping.switchTo(closing).when(() -> !hasBall() && goal == Goal.CLOSE);
    gripping.switchTo(moving).when(() -> goal == Goal.MOVE);
    gripping.switchTo(holding).when(() -> !hasBall() && goal != Goal.CLOSE);

    if (hasBall()) {
      machine.setInitialState(gripping);
    } else {
      machine.setInitialState(holding);
    }
    return machine;
  }

  // ------------------------------------------------------------------------------------------
  // Goals and Commands
  // ------------------------------------------------------------------------------------------

  public void setGoal(Goal goal) {
    this.goal = goal;
    if (goal == Goal.OPEN) {
      this.targetAngle = OPEN_ANGLE;
      this.setpoint = OPEN_ANGLE;
    } else if (goal == Goal.CLOSE) {
      this.targetAngle = CLOSED_ANGLE;
      this.setpoint = CLOSED_ANGLE;
    } else if (goal == Goal.HOLD) {
      this.setpoint = getAngle();
    }
    driveTowardSetpoint();
  }

  private Angle clampAngle(Angle angle) {
    if (angle.lt(CLOSED_ANGLE)) {
      return CLOSED_ANGLE;
    } else if (angle.gt(OPEN_ANGLE)) {
      return OPEN_ANGLE;
    } else {
      return angle;
    }
  }

  public void setGoal(Goal goal, Angle target) {
    this.goal = goal;
    this.targetAngle = clampAngle(target);
    this.setpoint = this.targetAngle;
    driveTowardSetpoint();
  }

  public Goal getGoal() {
    return goal;
  }

  public void requestOpen() {
    setGoal(Goal.OPEN);
  }

  public void requestClose() {
    setGoal(Goal.CLOSE);
  }

  public void requestMove(Angle angle) {
    setGoal(Goal.MOVE, angle);
  }

  public void requestMove(double angle) {
    requestMove(Radians.of(angle));
  }

  /**
   * Moves the fingers to an angle and finishes once they get there.
   *
   * @param targetAngle target angle of the fingers
   */
  public Command moveTo(Angle targetAngle) {
    Angle target = clampAngle(targetAngle);
    String name = String.format("Claw To %.0f deg", target.in(Degrees));
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.MOVE, target);
      if (getAngle().isNear(target, TOLERANCE)) {
        return;
      }
      coroutine.waitUntil(() -> hasBall() || atSetpoint());
    }).named(name);
  }

  /**
   * Overload for moving the fingers to an angle specified in radians as a double.
   *
   * @param angleRadians 0 is closed, {@link ClawConstants#OPEN_ANGLE} is fully open
   */
  public Command moveTo(double angleRadians) {
    return moveTo(Radians.of(angleRadians));
  }

  /** Opens the claw all the way. */
  public Command open() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.OPEN);
      if (getAngle().isNear(OPEN_ANGLE, TOLERANCE)) {
        return;
      }
      coroutine.waitUntil(this::atSetpoint);
    }).named("Claw Open");
  }

  /**
   * Closes the claw. Finishes when it is fully closed, or as soon as it grabs a ball (it can't
   * close any further when a ball is in the way!).
   */
  public Command close() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.CLOSE);
      if (hasBall() || getAngle().isNear(CLOSED_ANGLE, TOLERANCE)) {
        return;
      }
      coroutine.waitUntil(() -> hasBall() || atSetpoint());
    }).named("Claw Close");
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
    double volts = pid.calculate(getAngle().in(Radians), setpoint.in(Radians));
    motor.setVoltage(Math.clamp(volts, -MAX_VOLTS, MAX_VOLTS));
  }

  public boolean atSetpoint() {
    return getAngle().isNear(setpoint, TOLERANCE);
  }

  public boolean hasBall() {
    return ballSensor.get();
  }

  public Angle getAngle() {
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return STARTING_ANGLE;
    }
    return Radians.of(pos);
  }

  public Angle getSetpoint() {
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

  public DigitalInput getBallSensor() {
    return ballSensor;
  }

  /** The name of the most specific command currently using the claw. */
  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    setpoint = STARTING_ANGLE;
    goal = Goal.HOLD;
  }
}
