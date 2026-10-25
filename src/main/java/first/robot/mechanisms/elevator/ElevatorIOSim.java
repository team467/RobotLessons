package first.robot.mechanisms.elevator;

import static first.robot.Schematic.ELEVATOR_CAN_ID;

import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;

/**
 * Simulated elevator hardware using {@link FakeSparkMax} connected to the elevator motor in {@link
 * SimMechanical}.
 */
public class ElevatorIOSim implements ElevatorIO {
  private final FakeSparkMax motor = new FakeSparkMax(ELEVATOR_CAN_ID);

  @Override
  public void updateInputs(ElevatorInputs inputs) {
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
