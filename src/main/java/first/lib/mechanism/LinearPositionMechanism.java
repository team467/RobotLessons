package first.lib.mechanism;

import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;

/**
 * Reusable generic mechanism representing a single-degree-of-freedom linear position axis
 * controlled via closed-loop PID and optional feedforward (e.g. gravity compensation).
 */
@Logged
public class LinearPositionMechanism implements Mechanism {
  private final String name;
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;
  private final double kG;
  private final Distance minPosition;
  private final Distance maxPosition;
  private final Distance startingPosition;
  private final Distance tolerance;
  private final double maxVolts;

  private Distance targetPosition;

  public LinearPositionMechanism(
      String name,
      SparkMax motor,
      double kP,
      double kD,
      double maxVolts,
      Distance tolerance,
      Distance minPosition,
      Distance maxPosition,
      Distance startingPosition) {
    this(
        name,
        motor,
        kP,
        kD,
        0.0,
        maxVolts,
        tolerance,
        minPosition,
        maxPosition,
        startingPosition);
  }

  public LinearPositionMechanism(
      String name,
      SparkMax motor,
      double kP,
      double kD,
      double kG,
      double maxVolts,
      Distance tolerance,
      Distance minPosition,
      Distance maxPosition,
      Distance startingPosition) {
    this.name = name;
    this.motor = motor;
    this.encoder = motor.getEncoder();
    this.kG = kG;
    this.maxVolts = maxVolts;
    this.tolerance = tolerance;
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
    this.startingPosition = startingPosition;
    this.targetPosition = startingPosition;

    this.pid = new PIDController(kP, 0.0, kD);
    setDefaultCommand(hold());
  }

  public Distance getPosition() {
    if (encoder == null) {
      return startingPosition;
    }
    Double pos = encoder.getPosition().get();
    if (pos == null) {
      return startingPosition;
    }
    return Meters.of(pos);
  }

  public double getVelocity() {
    if (encoder == null) {
      return 0.0;
    }
    Double vel = encoder.getVelocity().get();
    if (vel == null) {
      return 0.0;
    }
    return vel;
  }

  public double getAppliedVolts() {
    if (motor == null) {
      return 0.0;
    }
    Double val = motor.getAppliedOutput().get();
    if (val == null) {
      return 0.0;
    }
    return val * 12.0;
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public Distance clampPosition(Distance pos) {
    if (pos.lt(minPosition)) {
      return minPosition;
    } else if (pos.gt(maxPosition)) {
      return maxPosition;
    } else {
      return pos;
    }
  }

  public void driveTowardSetpoint() {
    if (RobotBase.isDisabled()) {
      setVoltage(0.0);
      return;
    }
    double currentMeters = getPosition().in(Meters);
    double targetMeters = targetPosition.in(Meters);
    double volts = kG + pid.calculate(currentMeters, targetMeters);
    setVoltage(Math.clamp(volts, -maxVolts, maxVolts));
  }

  public boolean atSetpoint() {
    return getPosition().isNear(targetPosition, tolerance);
  }

  public boolean isSettledAtSetpoint() {
    return atSetpoint() && Math.abs(getVelocity()) <= 0.05;
  }

  public Command moveTo(Distance position) {
    Distance target = clampPosition(position);
    String cmdName = String.format("%s To %.2fm", name, target.in(Meters));
    return moveTo(target, cmdName);
  }

  public Command moveTo(double meters) {
    return moveTo(Meters.of(meters));
  }

  public Command moveTo(Distance position, String cmdName) {
    Distance target = clampPosition(position);
    targetPosition = target;
    return run(coroutine -> {
      targetPosition = target;
      pid.reset();
      while (!isSettledAtSetpoint()) {
        driveTowardSetpoint();
        coroutine.yield();
      }
      driveTowardSetpoint();
    }).named(cmdName);
  }

  public Command jog(String cmdName, double deltaMetersPerTick) {
    return run(coroutine -> {
      targetPosition = getPosition();
      while (true) {
        double nextMeters = targetPosition.in(Meters) + deltaMetersPerTick;
        targetPosition = clampPosition(Meters.of(nextMeters));
        driveTowardSetpoint();
        coroutine.yield();
      }
    }).whenCanceled(() -> {
      targetPosition = getPosition();
      pid.reset();
    }).named(cmdName);
  }

  public Command hold() {
    return run(coroutine -> {
          if (!atSetpoint()) {
            Distance pos = getPosition();
            targetPosition = pos.isNear(startingPosition, tolerance) ? startingPosition : pos;
            pid.reset();
          }
          while (true) {
            if (RobotBase.isDisabled()) {
              targetPosition = getPosition();
            }
            driveTowardSetpoint();
            coroutine.yield();
          }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named(name + " Hold");
  }

  public Distance getSetpoint() {
    return clampPosition(targetPosition);
  }

  public void setSetpoint(Distance target) {
    this.targetPosition = clampPosition(target);
  }

  public void resetSetpoint() {
    targetPosition = startingPosition;
    pid.reset();
  }

  public String getCurrentCommandName() {
    List<Command> running = getRunningCommands();
    if (running.isEmpty()) {
      return "(none)";
    }
    return running.get(running.size() - 1).name();
  }
}
