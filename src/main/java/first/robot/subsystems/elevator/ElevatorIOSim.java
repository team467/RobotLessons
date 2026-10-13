package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;

/** Simulated elevator: one NEO motor lifting the carriage on a chain, with gravity. */
public class ElevatorIOSim implements ElevatorIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

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
  public void updateInputs(ElevatorInputs inputs) {
    // A real motor controller outputs nothing while the robot is disabled.
    double volts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    sim.setInputVoltage(volts);
    sim.update(LOOP_PERIOD_SECONDS);

    if (sim.getPosition() < minHeightLimit) {
      sim.setState(minHeightLimit, Math.max(0.0, sim.getVelocity()));
    }

    inputs.positionMeters = sim.getPosition();
    inputs.velocityMetersPerSec = sim.getVelocity();
    inputs.appliedVolts = volts;
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
