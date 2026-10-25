package first.robot.mechanisms.shooter;

import static first.robot.Schematic.SHOOTER_CAN_ID;
import static first.robot.mechanisms.shooter.ShooterConstants.LOADED_PULLBACK;
import static first.robot.mechanisms.shooter.ShooterConstants.TOLERANCE_METERS;

import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimMechanical.ShooterMotorEntry;
import org.wpilib.framework.RobotBase;

/**
 * Simulated spring-loaded shooter hardware using {@link FakeSparkMax} connected to the shooter
 * pullback motor in {@link SimMechanical}.
 */
public class ShooterIOSim implements ShooterIO {
  private final FakeSparkMax motor = new FakeSparkMax(SHOOTER_CAN_ID);

  private ShooterMotorEntry simMotor() {
    return (ShooterMotorEntry) motor.getSimMotor();
  }

  @Override
  public void updateSim() {
    motor.updateSim();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    boolean enabled = RobotBase.isEnabled();
    inputs.pullbackMeters = motor.getEncoder().getPosition();
    inputs.velocityMetersPerSec = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedVolts();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.solenoidReleased = enabled && simMotor().isSolenoidReleased();
    inputs.ratchetLocked = !inputs.solenoidReleased;
    inputs.isLoaded =
        inputs.ratchetLocked && inputs.pullbackMeters >= (LOADED_PULLBACK - TOLERANCE_METERS);
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSolenoidReleased(boolean released) {
    simMotor().setSolenoidReleased(released);
  }

  @Override
  public void reset() {
    motor.resetSim();
  }

  /**
   * Consumes and returns the kick strength (1.0 = full loaded shot, 0.0 = no kick) if the solenoid
   * just released a cocked spring during this simulation step.
   */
  public double consumeKickStrength() {
    return simMotor().consumeKickStrength();
  }

  public double getSimPullback() {
    return simMotor().getSimPullback();
  }
}
