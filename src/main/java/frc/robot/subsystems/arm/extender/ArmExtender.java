package frc.robot.subsystems.arm.extender;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.arm.extender.ArmPositionConstants.ArmPosition;
import org.littletonrobotics.junction.Logger;

public class ArmExtender extends SubsystemBase {

  private static final double EXTEND_TOLERANCE_METERS = 0.008;

  private static final double BACK_FORCE = -1.4;
  private static final double HOLD_BACK_FORCE = -0.5;
  private static final double RETRACT_POSITION_CLOSE_TO_LIMIT = 0.1;
  private static final double RETRACT_VOLTAGE_CLOSE_TO_LIMIT = -0.7;

  private final Logger logger = Logger.getInstance();

  private final ArmExtenderIO io;
  private final ArmExtenderIOInputsAutoLogged inputs = new ArmExtenderIOInputsAutoLogged();

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
  public ArmExtender(ArmExtenderIO io) {
    super();
    this.io = io;
    io.updateInputs(inputs);
  }

  public void stop() {
    isHolding = false;
    volts = 0.0;
    io.setVoltage(0.0);
  }

  public void manual(double volts) {
    isHolding = false;
    if (isCalibrated && inputs.position < RETRACT_POSITION_CLOSE_TO_LIMIT && volts < 0) {
      setVoltage(Math.max(volts, RETRACT_VOLTAGE_CLOSE_TO_LIMIT));
    } else {
      setVoltage(volts);
    }
  }

  /** Zeros the positions of both motors, assuming that we're already at HOME position. */
  public void setCalibratedAssumeHomePosition() {
    io.resetEncoderPosition();
    isCalibrated = true;
  }

  public void hold() {
    isHolding = true;
    setpoint = inputs.position;
    setVoltage(calculateExtendPid(setpoint) + HOLD_BACK_FORCE);
  }

  public boolean isCalibrated() {
    return isCalibrated;
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

  public void setTargetPosition(double setpoint) {
    isHolding = false;
    this.setpoint =
        MathUtil.clamp(
            setpoint,
            RobotConstants.get().armExtendMinMeters,
            RobotConstants.get().armExtendMaxMeters);
    logger.recordOutput("Extender/Setpoint", setpoint);
    setVoltage(calculateExtendPid(this.setpoint));
  }

  public boolean isStopped() {
    return volts == 0;
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
    boolean isSafe = isCalibrated && (targetPosition < inputs.position);
    logger.recordOutput("Arm/IsExtendSafe", isSafe);
    if (!isSafe) {
      return 0;
    }
    double pidValue = pid.calculate(inputs.position, targetPosition);
    logger.recordOutput("ArmExtender/FbOutput", pidValue);
    return pidValue;
  }

  // Commands
  private Command releaseLatch() {
    return Commands.either(
        Commands.run(() -> this.manual(-2.0)).withTimeout(0.1), Commands.none(), this::isLatched);
  }

  private Command extend(double setpoint) {
    if (setpoint > this.setpoint) { // Need to release latch when moving out
      return releaseLatch()
          .andThen(
              Commands.run(() -> this.setTargetPosition(setpoint), this)
                  .until(this::isExtendPositionNear))
          .andThen(this::hold);
    } else {
      return Commands.run(() -> this.setTargetPosition(setpoint), this)
          .until(this::isExtendPositionNear)
          .andThen(this::hold);
    }
  }

  public Command retract() {
    return extend(0.05);
  }

  public Command moveTo(ArmPosition position) {
    return extend(position.extendSetpoint);
  }

  public Command manualExtend() {
    return releaseLatch().andThen(Commands.runEnd(() -> this.manual(1.5), this::hold, this));
  }

  public Command manualRetract() {
    return Commands.runEnd(() -> this.manual(-1.5), this::hold, this);
  }

  // Calibration Order
  // 1. Retract arm until limit switch is hit
  // 2. Set encoder position to 0
  // 3. Extend arm until in position to lower arm
  // 4. Lower arm until limit switch is hit
  //   4.1 If the rotate posiition is more than 0.15, restart calibration
  // 5. Retract arm some more

}
