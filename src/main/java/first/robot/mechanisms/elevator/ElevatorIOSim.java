package first.robot.mechanisms.elevator;

import static first.robot.Schematic.ELEVATOR_CAN_ID;

import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimMechanical.ElevatorMotorEntry;

/**
 * Simulated elevator hardware using {@link FakeSparkMax} connected to the elevator motor in {@link
 * SimMechanical}.
 */
public class ElevatorIOSim implements ElevatorIO {
  private final FakeSparkMax motor = new FakeSparkMax(ELEVATOR_CAN_ID);

  private ElevatorMotorEntry simMotor() {
    return (ElevatorMotorEntry) motor.getSimMotor();
  }

  public void setMinHeightLimit(double minHeight) {
    simMotor().setMinHeightLimit(minHeight);
  }

  public double getMinHeightLimit() {
    return simMotor().getMinHeightLimit();
  }

  @Override
  public void updateSim() {
    motor.updateSim();
  }

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

  public void resetSim() {
    motor.resetSim();
  }
}
