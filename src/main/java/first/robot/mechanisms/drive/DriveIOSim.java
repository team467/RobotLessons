package first.robot.mechanisms.drive;

import static first.robot.Schematic.DRIVE_CAN_ID;
import static first.robot.mechanisms.drive.DriveConstants.START_POSITION;

import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;

/**
 * Simulated drivetrain hardware using {@link FakeSparkMax} connected to the drivetrain motor in
 * {@link SimMechanical}.
 */
public class DriveIOSim implements DriveIO {
  private final FakeSparkMax motor = new FakeSparkMax(DRIVE_CAN_ID);

  @Override
  public void updateSim() {
    motor.updateSim();
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.positionMeters = motor.getEncoder().getPosition();
    inputs.velocityMetersPerSec = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedVolts();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void resetPosition() {
    resetPosition(START_POSITION);
  }

  @Override
  public void resetPosition(double positionMeters) {
    motor.getEncoder().setPosition(positionMeters);
  }

  public double getSimPosition() {
    return motor.getEncoder().getPosition();
  }

  public double getSimVelocity() {
    return motor.getEncoder().getVelocity();
  }
}
