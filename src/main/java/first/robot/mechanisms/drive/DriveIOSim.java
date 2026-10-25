package first.robot.mechanisms.drive;

import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;
import static first.robot.mechanisms.drive.DriveConstants.*;

import first.robot.constants.FieldConstants;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;

/** Simulated drivetrain hardware using WPILib's linear physics simulator. */
public class DriveIOSim implements DriveIO {
  // Horizontal linear carriage with 2 NEOs, toughbox gearing, robot mass, and wheels.
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          GEARING,
          ROBOT_MASS_KG,
          WHEEL_RADIUS_METERS,
          -1.0, // allowed range (meters)
          FieldConstants.FIELD_LENGTH_METERS + 1.0,
          false, // no gravity
          START_POSITION);

  private double requestedVolts = 0.0;

  @Override
  public void updateSim() {
    double volts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    sim.setInputVoltage(volts);
    sim.update(LOOP_PERIOD_SECONDS);
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.positionMeters = sim.getPosition();
    inputs.velocityMetersPerSec = sim.getVelocity();
    inputs.appliedVolts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    inputs.currentAmps = Math.abs(sim.getCurrentDraw());
  }

  @Override
  public void setVoltage(double volts) {
    requestedVolts = volts;
  }

  @Override
  public void resetPosition() {
    resetPosition(START_POSITION);
  }

  @Override
  public void resetPosition(double positionMeters) {
    sim.setState(positionMeters, 0.0);
    requestedVolts = 0.0;
  }

  public double getSimPosition() {
    return sim.getPosition();
  }

  public double getSimVelocity() {
    return sim.getVelocity();
  }
}
