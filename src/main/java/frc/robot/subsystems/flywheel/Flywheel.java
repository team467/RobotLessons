package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final Logger logger = Logger.getInstance();

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double volts = 0.0;

  /**
   * Configures the arm subsystem
   *
   * @param armIO Arm IO
   */
  public Flywheel(FlywheelIO io) {
    super();
    this.io = io;
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    logger.processInputs("Flywheel", inputs);
    io.setVoltage(volts);
  }

  private void manualVolts(double volts) {
    this.volts = volts;
  }

  // Readouts

  public double position() {
    return inputs.positionInMeters;
  }

  public double velocity() {
    return inputs.velocityInMetersPerSec;
  }

  public double appliedVoltage() {
    return inputs.appliedVolts;
  }

  public double current() {
    return inputs.currentAmps;
  }

  public double temperature() {
    return inputs.temperature;
  }

  public boolean forwardLimitSwitch() {
    return inputs.forwardLimitSwitch;
  }

  public boolean reverseLimitSwitch() {
    return inputs.reverseLimitSwitch;
  }

  // Commands

  public Command manualSpin(double speed) {
    return Commands.run(() -> this.manualVolts(speed * 12.0), this);
  }

  public Command stop() {
    return Commands.run(() -> this.manualVolts(0.0), this);
  }
}
