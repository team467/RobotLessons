package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.025);
  private double appliedVolts = 0.0;

  // For testing
  double simTempurature = 30.0;
  boolean simForwardLimitSwitch = false;
  boolean simReverseLimitSwitch = false;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.update(0.02);

    inputs.positionInMeters += sim.getAngularVelocityRadPerSec() * 0.02 * inputs.wheelRadius;
    inputs.velocityInMetersPerSec = sim.getAngularVelocityRadPerSec() * inputs.wheelRadius;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.temperature = simTempurature;
    inputs.forwardLimitSwitch = simForwardLimitSwitch;
    inputs.reverseLimitSwitch = simReverseLimitSwitch;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setBrakeMode(boolean brake) {}
}
