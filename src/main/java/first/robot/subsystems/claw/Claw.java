package first.robot.subsystems.claw;

import static first.robot.subsystems.claw.ClawConstants.*;
import static org.wpilib.units.Units.Radians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import first.robot.mechanisms.ClawMechanism;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.units.measure.Angle;

/** The claw opens and closes its fingers to pick up a ball. */
public class Claw implements Mechanism {
  /** The target goal/intent for the claw state machine. */
  public enum Goal {
    HOLD,
    OPEN,
    CLOSE
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput ballSensor;
  private final ClawMechanism clawMechanism;

  private Goal goal = Goal.HOLD;

  /** True while the claw is holding a ball. */
  public final Trigger hasBall = new Trigger(this::hasBall);

  public Claw() {
    this.motor = Mechanical.createSparkMax(Schematic.CLAW_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    this.ballSensor = Mechanical.createDigitalInput(Schematic.CLAW_BALL_SENSOR_DIO);
    Mechanical.register(motor);

    this.clawMechanism = new ClawMechanism(motor::setVoltage, this::getAngle, this::getVelocity);

    setDefaultCommand(hold());
  }

  // ------------------------------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------------------------------

  /**
   * Constructs the full lifecycle state machine for the claw mechanism, managing transitions
   * between holding, opening, closing, and gripping.
   */
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Claw");

    // State 1: Holding - Closed-loop PID holding at the current setpoint angle
    StateMachine.State holding = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            clawMechanism.setSetpoint(getAngle());
          }
          clawMechanism.driveTowardSetpoint();
        }).named("Claw Holding"));
    holding.onEnter(() -> {
      if (goal == Goal.HOLD) {
        clawMechanism.setSetpoint(getAngle());
      }
    });

    // State 2: Opening - Drives pinchers outward to full 60-degree open position
    StateMachine.State opening = machine.addState(
        runRepeatedly(clawMechanism::driveTowardSetpoint).named("Claw Opening"));
    opening.onEnter(() -> clawMechanism.setSetpoint(OPEN_ANGLE));

    // State 3: Closing - Drives pinchers inward toward 0 degrees to capture a ball
    StateMachine.State closing = machine.addState(
        runRepeatedly(clawMechanism::driveTowardSetpoint).named("Claw Closing"));
    closing.onEnter(() -> clawMechanism.setSetpoint(CLOSED_ANGLE));

    // State 4: Gripping - Direct voltage squeeze mode once ball is grasped
    StateMachine.State gripping = machine.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            motor.setVoltage(0.0);
          } else {
            motor.setVoltage(GRIP_VOLTS);
          }
        }).named("Claw Gripping"));

    // Transitions from holding
    holding.switchTo(gripping).when(this::hasBall);
    holding.switchTo(opening).when(() -> goal == Goal.OPEN && !atSetpoint());
    holding.switchTo(closing).when(() -> goal == Goal.CLOSE && !atSetpoint());

    // Transitions from opening
    opening.switchTo(holding).when(this::atSetpoint);
    opening.switchTo(closing).when(() -> goal == Goal.CLOSE);

    // Transitions from closing
    closing.switchTo(gripping).when(this::hasBall);
    closing.switchTo(holding).when(this::atSetpoint);
    closing.switchTo(opening).when(() -> goal == Goal.OPEN);

    // Transitions from gripping
    gripping.switchTo(opening).when(() -> goal == Goal.OPEN);
    gripping.switchTo(closing).when(() -> !hasBall() && goal == Goal.CLOSE);
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
      clawMechanism.setSetpoint(OPEN_ANGLE);
    } else if (goal == Goal.CLOSE) {
      clawMechanism.setSetpoint(CLOSED_ANGLE);
    } else if (goal == Goal.HOLD) {
      clawMechanism.setSetpoint(getAngle());
    }
    clawMechanism.driveTowardSetpoint();
  }

  public Command open() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.OPEN);
      if (getAngle().isNear(OPEN_ANGLE, TOLERANCE)) {
        return;
      }
      coroutine.waitUntil(this::atSetpoint);
    }).named("Claw Open");
  }

  public Command close() {
    return Command.noRequirements(coroutine -> {
      setGoal(Goal.CLOSE);
      if (hasBall() || getAngle().isNear(CLOSED_ANGLE, TOLERANCE)) {
        return;
      }
      coroutine.waitUntil(() -> hasBall() || atSetpoint());
    }).named("Claw Close");
  }

  public Command hold() {
    return run(coroutine -> coroutine.await(stateMachine()))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Claw Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  public boolean atSetpoint() {
    return clawMechanism.atSetpoint();
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
    return clawMechanism.getSetpoint();
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
    goal = Goal.HOLD;
    clawMechanism.resetSetpoint();
  }
}
