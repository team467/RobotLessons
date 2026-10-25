package first.robot.mechanisms.elevator;

import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;
import static first.robot.mechanisms.elevator.ElevatorConstants.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;

/** Simulated elevator: one NEO motor lifting the carriage on a chain, with gravity. */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          GEARING,
          CARRIAGE_MASS_KG,
          DRUM_RADIUS_METERS,
          MIN_HEIGHT,
          MAX_HEIGHT,
          true, // simulate gravity
          STARTING_HEIGHT);

  private double requestedVolts = 0.0;
  private double minHeightLimit = MIN_HEIGHT;

  public void setMinHeightLimit(double minHeight) {
    this.minHeightLimit = minHeight;
  }

  public double getMinHeightLimit() {
    return minHeightLimit;
  }

  @Override
  public void updateSim() {
    double volts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    sim.setInputVoltage(volts);
    sim.update(LOOP_PERIOD_SECONDS);

    if (sim.getPosition() < minHeightLimit) {
      sim.setState(minHeightLimit, Math.max(0.0, sim.getVelocity()));
    }
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
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
    sim.setState(STARTING_HEIGHT, 0.0);
    requestedVolts = 0.0;
    minHeightLimit = MIN_HEIGHT;
  }
}
