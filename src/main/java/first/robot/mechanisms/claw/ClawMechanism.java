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
  // Commands
  // ------------------------------------------------------------------------------------------

  /**
   * Moves the fingers to an angle and finishes once they get there.
   *
   * @param angleRadians 0 is closed, {@link ClawConstants#OPEN_ANGLE} is fully open
   */
  public Command moveTo(double angleRadians) {
    double target = Math.clamp(angleRadians, CLOSED_ANGLE, OPEN_ANGLE);
    setpoint = target;
    String name = String.format("Claw To %.0f deg", Math.toDegrees(target));
    StateMachine sm = new StateMachine(name);
    var moving = sm.addState(runRepeatedly(this::driveTowardSetpoint).named(name));
    moving.onEnter(() -> setpoint = target);
    sm.setInitialState(moving);
    moving.exitStateMachine().when(this::atSetpoint);
    return run(coroutine -> coroutine.await(sm)).named(name);
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
    setpoint = CLOSED_ANGLE;
    StateMachine sm = new StateMachine("Claw Close");
    var closing = sm.addState(runRepeatedly(this::driveTowardSetpoint).named("Claw Close"));
    closing.onEnter(() -> setpoint = CLOSED_ANGLE);
    sm.setInitialState(closing);
    closing.exitStateMachine().when(() -> atSetpoint() || hasBall());
    return run(coroutine -> coroutine.await(sm)).named("Claw Close");
  }

  /**
   * The default command. Keeps squeezing if we have a ball, otherwise holds the current angle.
   */
  public Command hold() {
    StateMachine sm = new StateMachine("Claw Hold [FSM]");
    var holding = sm.addState(
        runRepeatedly(() -> {
          if (RobotBase.isDisabled()) {
            setpoint = getAngle();
          }
          if (hasBall()) {
            motor.setVoltage(GRIP_VOLTS);
          } else {
            driveTowardSetpoint();
          }
        }).named("Claw Hold"));
    holding.onEnter(() -> setpoint = getAngle());
    sm.setInitialState(holding);
    return run(coroutine -> coroutine.await(sm))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Claw Hold");
  }

  // ------------------------------------------------------------------------------------------
  // Helpers and state
  // ------------------------------------------------------------------------------------------

  private void driveTowardSetpoint() {
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
