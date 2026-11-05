package first.robot.subsystems.claw;

import static first.robot.subsystems.claw.ClawConstants.*;
import static org.wpilib.units.Units.Radians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import first.robot.mechanisms.ClawMechanism;
import first.robot.mechanisms.TrackedStateMachine;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.units.measure.Angle;

/** The claw opens and closes its fingers to pick up a ball. */
public class Claw {
  /** The states of the claw state machine, which are also the goals commands can request. */
  private enum Mode {
    HOLD,
    OPEN,
    CLOSE,
    GRIP
  }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput ballSensor;
  private final ClawMechanism clawMechanism;
  private final TrackedStateMachine<Mode> stateMachine = new TrackedStateMachine<>("Claw");

  /** True while the claw is holding a ball. */
  public final Trigger hasBall = new Trigger(this::hasBall);

  public Claw() {
    this.motor = Mechanical.createSparkMax(Schematic.CLAW_CAN_ID, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    this.ballSensor = Mechanical.createDigitalInput(Schematic.CLAW_BALL_SENSOR_DIO);
    Mechanical.register(motor);

    this.clawMechanism = new ClawMechanism(motor::setVoltage, this::getAngle, this::getVelocity);

    buildStateMachine();
    clawMechanism.setDefaultCommand(stateMachine.runOn(clawMechanism));
  }

  // ------------------------------------------------------------------------------------------
  // State Machine
  // ------------------------------------------------------------------------------------------

  /** Declares the claw's states and transitions between holding, opening, closing and gripping. */
  private void buildStateMachine() {
    // Closed-loop PID holding at the angle the claw was in when it entered this state
    stateMachine.addState(
        Mode.HOLD,
        clawMechanism.runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            clawMechanism.setSetpoint(getAngle());
          }
          clawMechanism.driveTowardSetpoint();
        }).named("Claw Holding"),
        () -> clawMechanism.setSetpoint(getAngle()));

    // Drives pinchers outward to the open position
    stateMachine.addState(
        Mode.OPEN,
        clawMechanism.runRepeatedly(clawMechanism::driveTowardSetpoint).named("Claw Opening"),
        () -> clawMechanism.setSetpoint(OPEN_ANGLE));

    // Drives pinchers inward to capture a ball
    stateMachine.addState(
        Mode.CLOSE,
        clawMechanism.runRepeatedly(clawMechanism::driveTowardSetpoint).named("Claw Closing"),
        () -> clawMechanism.setSetpoint(CLOSED_ANGLE));

    // Direct voltage squeeze once a ball is grasped
    stateMachine.addState(
        Mode.GRIP,
        clawMechanism.runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            motor.setVoltage(0.0);
          } else {
            motor.setVoltage(GRIP_VOLTS);
          }
        }).named("Claw Gripping"));

    stateMachine.transition(Mode.HOLD, Mode.GRIP, this::hasBall);
    stateMachine.transition(Mode.HOLD, Mode.OPEN, stateMachine.isRequested(Mode.OPEN));
    stateMachine.transition(Mode.HOLD, Mode.CLOSE, stateMachine.isRequested(Mode.CLOSE));

    stateMachine.transition(Mode.OPEN, Mode.HOLD, this::atSetpoint);
    stateMachine.transition(Mode.OPEN, Mode.CLOSE, stateMachine.isRequested(Mode.CLOSE));

    stateMachine.transition(Mode.CLOSE, Mode.GRIP, this::hasBall);
    stateMachine.transition(Mode.CLOSE, Mode.HOLD, this::atSetpoint);
    stateMachine.transition(Mode.CLOSE, Mode.OPEN, stateMachine.isRequested(Mode.OPEN));

    stateMachine.transition(Mode.GRIP, Mode.OPEN, stateMachine.isRequested(Mode.OPEN));
    stateMachine.transition(Mode.GRIP, Mode.CLOSE, stateMachine.isRequested(Mode.CLOSE));
    stateMachine.transition(Mode.GRIP, Mode.HOLD, () -> !hasBall());

    stateMachine.setInitialState(Mode.HOLD);
  }

  // ------------------------------------------------------------------------------------------
  // Commands
  // ------------------------------------------------------------------------------------------

  public Command open() {
    return stateMachine.setGoal(Mode.OPEN);
  }

  public Command close() {
    return stateMachine.setGoal(Mode.CLOSE);
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
    List<Command> running = clawMechanism.getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }

  public void resetSetpoint() {
    clawMechanism.resetSetpoint();
  }
}
