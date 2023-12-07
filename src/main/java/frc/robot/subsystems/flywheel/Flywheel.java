package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.controls.FeedbackConstant;

public class Flywheel extends SubsystemBase{
  
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  
  private double kP = 3.2526;
  private double kD = 0.05;
  private double maxVelocity = 550.6;
  private double maxAcceleration = 7585;
  private final ProfiledPIDController controller = new FeedbackConstant(kP, kD)
    .getProfiledPIDController(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

  private boolean isAutomated = false;
  private double volts = 0.0;

  /**
   * Configures the arm subsystem
   *
   * @param flywheelIO Flywheel IO
   */
  public Flywheel(FlywheelIO io) {
    super();
    this.io = io;
    io.updateInputs(inputs);
    logger.processInputs("Flywheel", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    if (isAutomated) {
      volts = controller.calculate(inputs.velocityInMetersPerSec);
    }
    io.setVoltage(volts);
  }

  private void manualVolts(double volts) {
    isAutomated = false;
    this.volts = volts;
  }

  private void speed(double speed) {
    isAutomated = true;
    controller.setGoal(speed);

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

  public Command spinToSpeed(double speed) {
    return Commands.run(() -> this.speed(speed), this);
  }

}
