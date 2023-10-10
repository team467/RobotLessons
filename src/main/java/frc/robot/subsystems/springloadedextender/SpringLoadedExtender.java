package frc.robot.subsystems.springloadedextender;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class SpringLoadedExtender extends SubsystemBase {

  private static final double EXTEND_TOLERANCE_METERS = 0.008;

  private static final double BACK_FORCE = -1.4;
  private static final double HOLD_BACK_FORCE = -0.5;
  private static final double RETRACT_POSITION_CLOSE_TO_LIMIT = 0.1;
  private static final double RETRACT_VOLTAGE_CLOSE_TO_LIMIT = -0.7;

  private static final double CALIBRATE_RETRACT_VOLTAGE = -1.5;

  private final Logger logger = Logger.getInstance();

  private final SpringLoadedExtenderIO io;
  private final SpringLoadedExtenderIOInputsAutoLogged inputs =
      new SpringLoadedExtenderIOInputsAutoLogged();

  private boolean isHolding = false;
  private boolean isCalibrated = false;

  private double setpoint;
  private double volts = 0.0;
  private PIDController pid = new PIDController(60, 0, 0);

  /**
   * Configures the arm subsystem
   *
   * @param armIO Arm IO
   */
  public SpringLoadedExtender(SpringLoadedExtenderIO io) {
    super();
    this.io = io;
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    logger.processInputs("Arm Extender", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    logger.recordOutput("Arm/IsCalibrated", isCalibrated);
    logger.recordOutput("Arm/isHolding", isHolding);

    if (isHolding) {
      io.setVoltageWhileHold(volts);
    } else {
      io.setVoltage(volts);
    }
  }

  // Methods for use by commands

  private void stop() {
    isHolding = false;
    volts = 0.0;
    io.setVoltage(0.0);
  }

  private void manual(double volts) {
    isHolding = false;
    if (isCalibrated && inputs.position < RETRACT_POSITION_CLOSE_TO_LIMIT && volts < 0) {
      setVoltage(Math.max(volts, RETRACT_VOLTAGE_CLOSE_TO_LIMIT));
    } else {
      setVoltage(volts);
    }
  }

  /** Zeros the positions of both motors, assuming that we're already at HOME position. */
  private void setCalibrated() {
    io.resetEncoderPosition();
    isCalibrated = true;
  }

  private boolean checkIfInCalibrationPosition() {
    if (inputs.reverseLimitSwitch) {
      setCalibrated();
      return true;
    }
    return false;
  }

  private void hold() {
    isHolding = true;
    setpoint = inputs.position;
    setVoltage(calculateExtendPid(setpoint) + HOLD_BACK_FORCE);
  }

  private void setTargetPosition(double setpoint) {
    isHolding = false;
    this.setpoint =
        MathUtil.clamp(
            setpoint,
            RobotConstants.get().armExtendMinMeters,
            RobotConstants.get().armExtendMaxMeters);
    logger.recordOutput("Extender/Setpoint", setpoint);
    setVoltage(calculateExtendPid(this.setpoint));
  }

  private void setVoltage(double volts) {
    if (volts < 0) {
      volts = volts + BACK_FORCE;
    } else {
      io.setVoltage(volts);
    }
  }

  private boolean isExtendPositionNear() {
    return Math.abs(inputs.position - setpoint) <= EXTEND_TOLERANCE_METERS;
  }

  private boolean isLatched() {
    return inputs.ratchetLocked;
  }

  private double calculateExtendPid(double targetPosition) {
    if (!isCalibrated) {
      return 0;
    }
    double pidValue = pid.calculate(inputs.position, targetPosition);
    logger.recordOutput("ArmExtender/FbOutput", pidValue);
    return pidValue;
  }

  // Readouts

  public boolean isCalibrated() {
    return isCalibrated;
  }

  public boolean isStopped() {
    return volts == 0;
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

  public boolean isRatchetLocked() {
    return inputs.ratchetLocked;
  }

  // Commands
  private Command releaseLatch() {
    return Commands.either(
        Commands.run(() -> this.manual(-2.0)).withTimeout(0.1), Commands.none(), this::isLatched);
  }

  public Command moveTo(double position) {
    if (position > inputs.position && isRatchetLocked()) { // Need to release latch when moving out
      return releaseLatch()
          .andThen(
              Commands.run(() -> this.setTargetPosition(position), this)
                  .until(this::isExtendPositionNear))
          .andThen(this::hold);
    } else {
      return Commands.run(() -> this.setTargetPosition(position), this)
          .until(this::isExtendPositionNear)
          .andThen(this::hold);
    }
  }

  public Command manualExtend() {
    return releaseLatch().andThen(Commands.runEnd(() -> this.manual(1.5), this::hold, this));
  }

  public Command manualRetract() {
    return Commands.runEnd(() -> this.manual(-1.5), this::hold, this);
  }

  public Command calibrate() {
    return Commands.run(() -> this.manual(CALIBRATE_RETRACT_VOLTAGE), this)
        .until(this::checkIfInCalibrationPosition)
        .andThen(this::hold);
  }

  public Command forceCalibrated() {
    return Commands.runOnce(this::setCalibrated, this);
  }

  public Command setUncalibrated() {
    return Commands.runOnce(() -> isCalibrated = false, this);
  }
}
