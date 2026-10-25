package first.robot.mechanisms.claw;

import static first.robot.Schematic.CLAW_BALL_SENSOR_DIO;
import static first.robot.Schematic.CLAW_CAN_ID;

import first.robot.sim.FakeDigitalInput;
import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimMechanical.ClawMotorEntry;

/**
 * Simulated claw hardware using {@link FakeSparkMax} and {@link FakeDigitalInput} connected to the
 * claw motor and limit switch in {@link SimMechanical}.
 */
public class ClawIOSim implements ClawIO {
  private final FakeSparkMax motor = new FakeSparkMax(CLAW_CAN_ID);
  private final FakeDigitalInput ballSensor = new FakeDigitalInput(CLAW_BALL_SENSOR_DIO);

  private ClawMotorEntry simMotor() {
    return (ClawMotorEntry) motor.getSimMotor();
  }

  @Override
  public void updateSim() {
    motor.updateSim();
  }

  @Override
  public void updateInputs(ClawInputs inputs) {
    inputs.angleRadians = motor.getEncoder().getPosition();
    inputs.velocityRadPerSec = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedVolts();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.hasBall = ballSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  // ---- Sim-only hooks, called by the simulated world ----

  /** Stops the fingers from closing past {@code angleRadians}. Pass NaN to remove the stop. */
  public void setGripStop(double angleRadians) {
    simMotor().setGripStop(angleRadians);
  }

  /** Sets what the simulated ball limit switch reports. */
  public void setBallDetected(boolean detected) {
    ballSensor.set(detected);
  }

  /** The true simulated angle (no one-loop delay), for the simulated world. */
  public double getSimAngle() {
    return simMotor().getSimAngle();
  }

  public void resetSim() {
    motor.resetSim();
    ballSensor.resetSim();
  }
}
