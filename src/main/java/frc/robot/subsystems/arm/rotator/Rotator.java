package frc.robot.subsystems.arm.rotator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Rotator extends SubsystemBase {

  private final Logger logger = Logger.getInstance();

  private static final double ROTATE_TOLERANCE_METERS = 0.0023;
  private static final double ROTATE_RAISE_METERS = 0.025;

  private final RotatorIO io;
  private final RotatorIOInputsAutoLogged inputs = new RotatorIOInputsAutoLogged();

  private boolean isCalibrated = false;

  private PIDController pid = new PIDController(800, 0, 0);
  private double setpoint = 0.0;

  private double volts = 0.0;

  /**
   * Configures the arm subsystem
   *
   * @param armIO Arm IO
   */
  public Rotator(RotatorIO io) {
    super();
    this.io = io;
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    logger.processInputs("Rotator", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    logger.recordOutput("Rotator/IsCalibrated", isCalibrated);
    io.setVoltage(volts);
  }

  // Methods for use by commands

  private void stop() {
    volts = 0.0;
    io.setVoltage(0.0);
  }

  private void manual(double volts) {
    this.volts = volts;
  }

  /** Zeros the positions of both motors, assuming that we're already at HOME position. */
  private void setCalibrated() {
    io.resetEncoderPosition();
    isCalibrated = true;
  }

  private void setTargetPosition(double position) {
    this.setpoint =
        MathUtil.clamp(
            position,
            RobotConstants.get().armRotateMinMeters,
            RobotConstants.get().armRotateMaxMeters);
    if (!isCalibrated) {
      volts = 0;
      return;
    }
    volts = pid.calculate(inputs.position, setpoint);
    logger.recordOutput("Rotator/FbOutput", volts);
  }

  private boolean isFinished() {
    return Math.abs(inputs.position - setpoint) <= ROTATE_TOLERANCE_METERS
        || (inputs.highLimitSwitch && inputs.position < setpoint)
        || (inputs.lowLimitSwitch && inputs.position > setpoint);
  }

  // Readouts

  public boolean isCalibrated() {
    return isCalibrated;
  }

  public double position() {
    return inputs.position;
  }

  public double setpoint() {
    return setpoint;
  }

  public double velocity() {
    return inputs.velocity;
  }

  public double appliedVoltage() {
    return inputs.appliedVolts;
  }

  public double current() {
    return inputs.current;
  }

  public double temperature() {
    return inputs.temp;
  }

  public boolean highLimitSwitch() {
    return inputs.highLimitSwitch;
  }

  public boolean lowLimitSwitch() {
    return inputs.lowLimitSwitch;
  }

  // Commands

  public Command rotate(double targetPosition) {
    return Commands.run(() -> this.setTargetPosition(targetPosition), this)
        .until(this::isFinished)
        .andThen(this::stop);
  }

  public Command manualUp() {
    return Commands.runEnd(() -> this.manual(6.0), this::stop, this);
  }

  public Command manualDown() {
    return Commands.runEnd(() -> this.manual(-6.0), this::stop, this);
  }

  public Command raise() {
    // Can only raise if not too high
    if (inputs.position < 0.1) {
      return Commands.run(() -> this.setTargetPosition(inputs.position + ROTATE_RAISE_METERS), this)
        .until(this::isFinished)
        .andThen(this::stop);
    }
    return Commands.none();
  }

  public Command forceCalibrated() {
    return Commands.run(this::setCalibrated, this);
  }

  public Command checkCalibrated() {
    if (inputs.lowLimitSwitch) {
      return Commands.run(this::setCalibrated, this);
    } else {
      return Commands.none();
    }
  }

  public Command setUncalibrated() {
    return Commands.run(() -> isCalibrated = false, this);
  }

}
