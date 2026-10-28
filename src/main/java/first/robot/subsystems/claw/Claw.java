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
   *
   * <p>States:
   * <ul>
   *   <li><b>holding</b>: Uses closed-loop PID control to maintain the current setpoint angle.
   *       Tracks position while disabled to avoid sudden motion upon re-enabling.
   *   <li><b>opening</b>: Actively drives the pinchers toward {@link ClawConstants#OPEN_ANGLE} (60 deg).
   *   <li><b>closing</b>: Actively drives the pinchers toward {@link ClawConstants#CLOSED_ANGLE} (0 deg).
   *   <li><b>gripping</b>: Applies a constant holding voltage ({@link ClawConstants#GRIP_VOLTS} = -2.0V)
   *       to squeeze a captured ball continuously without PID derivative fight or steady-state error.
   *   <li><b>moving</b>: Moves the claw to an intermediate target angle requested via {@link #moveTo}.
   * </ul>
   *
   * <p>Transitions:
   * <ul>
   *   <li>Any state -> <b>gripping</b>: Triggered immediately when the optical ball sensor detects a ball.
   *   <li><b>opening</b> / <b>closing</b> / <b>moving</b> -> <b>holding</b>: Triggered when within tolerance of setpoint.
   *   <li><b>gripping</b> -> <b>holding</b> / <b>closing</b>: Triggered if the ball drops or is released.
   * </ul>
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Claw");

    // State 1: Holding - Closed-loop PID holding at the current setpoint angle
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

    // State 2: Opening - Drives pinchers outward to full 60-degree open position
    StateMachine.State opening = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Opening"));
    opening.onEnter(() -> setpoint = OPEN_ANGLE);

    // State 3: Closing - Drives pinchers inward toward 0 degrees to capture a ball
    StateMachine.State closing = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Closing"));
    closing.onEnter(() -> setpoint = CLOSED_ANGLE);

    // State 4: Gripping - Direct voltage squeeze mode once ball is grasped
    StateMachine.State gripping = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            motor.setVoltage(0.0);
          } else {
            // Apply constant squeeze voltage to maintain grip on the ball without PID stalling
            motor.setVoltage(GRIP_VOLTS);
          }
        }).named("Claw Gripping"));

    // State 5: Moving - Moves to an arbitrary user-specified intermediate angle
    StateMachine.State moving = machine.addState(
        runRepeatedly(this::driveTowardSetpoint).named("Claw Moving"));
    moving.onEnter(() -> setpoint = targetAngle);

    // ---- Transitions from holding ----
    // If a ball enters the claw, immediately clamp down in gripping mode
    holding.switchTo(gripping).when(this::hasBall);
    holding.switchTo(opening).when(() -> goal == Goal.OPEN && !atSetpoint());
    holding.switchTo(closing).when(() -> goal == Goal.CLOSE && !atSetpoint());
    holding.switchTo(moving).when(() -> goal == Goal.MOVE && !atSetpoint());

    // ---- Transitions from opening ----
    opening.switchTo(holding).when(this::atSetpoint);
    opening.switchTo(closing).when(() -> goal == Goal.CLOSE);
    opening.switchTo(moving).when(() -> goal == Goal.MOVE);

    // ---- Transitions from closing ----
    // Switch to gripping immediately when optical sensor trips, or hold when fully closed
    closing.switchTo(gripping).when(this::hasBall);
    closing.switchTo(holding).when(this::atSetpoint);
    closing.switchTo(opening).when(() -> goal == Goal.OPEN);
    closing.switchTo(moving).when(() -> goal == Goal.MOVE);

    // ---- Transitions from moving ----
    moving.switchTo(gripping).when(this::hasBall);
    moving.switchTo(holding).when(this::atSetpoint);
    moving.switchTo(opening).when(() -> goal == Goal.OPEN);
    moving.switchTo(closing).when(() -> goal == Goal.CLOSE);

    // ---- Transitions from gripping ----
    gripping.switchTo(opening).when(() -> goal == Goal.OPEN);
    gripping.switchTo(closing).when(() -> !hasBall() && goal == Goal.CLOSE);
    gripping.switchTo(moving).when(() -> goal == Goal.MOVE);
    gripping.switchTo(holding).when(() -> !hasBall() && goal != Goal.CLOSE);

    // Initial state based on current ball sensor reading
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

  /**
   * Sets the high-level intent/goal for the claw and updates the setpoint angle accordingly.
   *
   * @param goal the target goal (HOLD, OPEN, CLOSE)
   */
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

  /** Clamps an angle to the physical mechanical travel limits of the claw fingers. */
  private Angle clampAngle(Angle angle) {
    if (angle.lt(CLOSED_ANGLE)) {
      return CLOSED_ANGLE;
    } else if (angle.gt(OPEN_ANGLE)) {
      return OPEN_ANGLE;
    } else {
      return angle;
    }
  }

  /**
   * Sets the goal to MOVE and targets a specific clamped angle.
   *
   * @param goal target goal (typically {@link Goal#MOVE})
   * @param target angle to move toward
   */
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
   * <p>Early exit: If a ball enters the claw during closing, this command completes immediately
   * so subsequent sequence actions can proceed without timing out.
   *
   * @param targetAngle target angle of the fingers
   * @return a command that awaits reaching the angle or grabbing a ball
   */
  public Command moveTo(Angle targetAngle) {
    Angle target = clampAngle(targetAngle);
    String name = String.format("Claw To %.0f deg", target.in(Degrees));
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.MOVE, target);
      // If already at setpoint, finish immediately
      if (getAngle().isNear(target, TOLERANCE)) {
        return;
      }
      // Wait until target angle is reached, or a ball is captured
      coroutine.waitUntil(() -> hasBall() || atSetpoint());
    }).named(name);
  }

  /**
   * Overload for moving the fingers to an angle specified in radians as a double.
   *
   * @param angleRadians 0 is closed, {@link ClawConstants#OPEN_ANGLE} is fully open
   * @return a command that awaits reaching the angle
   */
  public Command moveTo(double angleRadians) {
    return moveTo(Radians.of(angleRadians));
  }

  /**
   * Opens the claw all the way to {@link ClawConstants#OPEN_ANGLE} (60 degrees).
   *
   * @return a command that completes once the claw reaches the open position
   */
  public Command open() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.OPEN);
      // Skip wait if already open
      if (getAngle().isNear(OPEN_ANGLE, TOLERANCE)) {
        return;
      }
      // Await until claw position is within tolerance of OPEN_ANGLE
      coroutine.waitUntil(this::atSetpoint);
    }).named("Claw Open");
  }

  /**
   * Closes the claw. Finishes when it is fully closed (0 degrees), or as soon as it grabs a ball
   * (the ball prevents fingers from closing further, so waiting for 0 deg would stall indefinitely).
   *
   * @return a command that completes when the claw is closed or a ball is gripped
   */
  public Command close() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.CLOSE);
      // Skip wait if already holding a ball or already closed
      if (hasBall() || getAngle().isNear(CLOSED_ANGLE, TOLERANCE)) {
        return;
      }
      // Wait until either optical ball sensor detects a ball OR claw reaches 0 degrees
      coroutine.waitUntil(() -> hasBall() || atSetpoint());
    }).named("Claw Close");
  }

  /**
   * The default command. Runs the claw state machine to hold, grip, open, close, or move.
   *
   * <p>Runs at {@link Command#LOWEST_PRIORITY} so that any manual jog or autonomous command
   * can seamlessly interrupt it and take control when needed.
   *
   * @return the continuous default holding command
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
    pid.reset();
  }
}
