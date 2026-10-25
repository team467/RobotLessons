package first.robot.mechanisms.shooter;

import static first.robot.Schematic.SHOOTER_CAN_ID;
import static first.robot.Schematic.SHOOTER_SOLENOID_CHANNEL;
import static first.robot.mechanisms.shooter.ShooterConstants.LOADED_PULLBACK;
import static first.robot.mechanisms.shooter.ShooterConstants.TOLERANCE_METERS;

import first.robot.sim.FakeSolenoid;
import first.robot.sim.FakeSparkMax;
import first.robot.sim.SimMechanical;
import org.wpilib.framework.RobotBase;

/**
 * Simulated spring-loaded shooter hardware using {@link FakeSparkMax} and {@link FakeSolenoid}
 * connected to the shooter pullback motor and release solenoid in {@link SimMechanical}.
 */
public class ShooterIOSim implements ShooterIO {
  private final FakeSparkMax motor = new FakeSparkMax(SHOOTER_CAN_ID);
  private final FakeSolenoid solenoid = new FakeSolenoid(SHOOTER_SOLENOID_CHANNEL);

  @Override
  public void updateInputs(ShooterInputs inputs) {
    boolean enabled = RobotBase.isEnabled();
    inputs.pullbackMeters = motor.getEncoder().getPosition();
    inputs.velocityMetersPerSec = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedVolts();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.solenoidReleased = enabled && solenoid.get();
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
    solenoid.set(released);
  }
}
