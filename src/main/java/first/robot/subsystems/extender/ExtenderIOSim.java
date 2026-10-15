package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;

/**
 * Simulated extender: one NEO motor pulling a slide in and out on a belt/pulley.
 *
 * <p>A horizontal linear slide behaves just like an elevator without gravity, so we reuse WPILib's
 * {@link ElevatorSim} with gravity turned off.
 */
public class ExtenderIOSim implements ExtenderIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          GEARING,
          SLIDE_MASS_KG,
          PULLEY_RADIUS_METERS,
          MIN_EXTENSION,
          MAX_EXTENSION,
          false, // horizontal: no gravity
          STARTING_EXTENSION);

  private double requestedVolts = 0.0;
  private double maxExtensionLimit = MAX_EXTENSION;

  public void setMaxExtensionLimit(double maxExtension) {
    this.maxExtensionLimit = maxExtension;
  }

  public double getMaxExtensionLimit() {
    return maxExtensionLimit;
  }

  @Override
  public void updateSim() {
    double volts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    sim.setInputVoltage(volts);
    sim.update(LOOP_PERIOD_SECONDS);

    if (sim.getPosition() > maxExtensionLimit) {
      sim.setState(maxExtensionLimit, Math.min(0.0, sim.getVelocity()));
    }
  }

  @Override
  public void updateInputs(ExtenderInputs inputs) {
    inputs.positionMeters = sim.getPosition();
    inputs.velocityMetersPerSec = sim.getVelocity();
    inputs.appliedVolts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    inputs.currentAmps = Math.abs(sim.getCurrentDraw());
  }

  @Override
  public void setVoltage(double volts) {
    requestedVolts = volts;
  }

  public void resetSim() {
    sim.setState(STARTING_EXTENSION, 0.0);
    requestedVolts = 0.0;
    maxExtensionLimit = MAX_EXTENSION;
  }
}
