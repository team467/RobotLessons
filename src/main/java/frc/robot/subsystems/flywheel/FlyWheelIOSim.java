package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlyWheelIOSim implements FlyWheelIO {
  private static final double TIME_STEP = 0.02;
  private static final double MAX_VOLTAGE = 12.0;
  private static final double MIN_VOLTAGE = -12.0;
  private static final FlyWheelIOSim driveSim = new FlyWheelIOSim();

  private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private double appliedVolts = 0.0;

  public void updateInputs(frc.robot.subsystems.flywheel.FlyWheelIO.FlyWheelIOInputs inputs) {
    try {
      sim.update(TIME_STEP);
    } catch (NumberFormatException e) {
      // Replace SpecificException with the specific exception you're expecting
      System.out.println("Error updating inputs: " + e.getMessage());
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, MIN_VOLTAGE, MAX_VOLTAGE);
    driveSim.setInputVoltage(appliedVolts);
  }

  private void setInputVoltage(double appliedVolts2) {}

  public void updateInputs(
      frc.robot.subsystems.flywheel.FlyWheelIO.FlyWheelIOInputs inputs, double volts) {
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}
