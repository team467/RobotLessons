package first.robot.mechanisms.extender;

import static first.robot.Schematic.EXTENDER_CAN_ID;

import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;

/**
 * Simulated extender hardware using {@link FakeSparkMax} connected to the extender motor in {@link
 * SimMechanical}.
 */
public class ExtenderIOSim implements ExtenderIO {
  private final FakeSparkMax motor = new FakeSparkMax(EXTENDER_CAN_ID);

  @Override
  public void updateInputs(ExtenderInputs inputs) {
    inputs.positionMeters = motor.getEncoder().getPosition();
    inputs.velocityMetersPerSec = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedVolts();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
