package first.robot.mechanisms.claw;

import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;
import static first.robot.mechanisms.claw.ClawConstants.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Simulated claw: one NEO 550 swinging the fingers open and closed.
 *
 * <p>The fingers rotate around a pivot, so we use WPILib's {@link SingleJointedArmSim} with gravity
 * turned off. The simulated world ({@code first.robot.sim.SimWorld}) tells this class when a ball
 * is between the fingers so they stop closing when they touch it.
 */
public class ClawIOSim implements ClawIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          GEARING,
          FINGERS_MOI_KG_M2,
          FINGER_LENGTH_METERS,
          CLOSED_ANGLE,
          OPEN_ANGLE,
          false, // no gravity
          STARTING_ANGLE);

  private double requestedVolts = 0.0;
  private double gripStopAngle = Double.NaN;
  private boolean ballDetected = false;

  @Override
  public void updateSim() {
    double volts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    sim.setInputVoltage(volts);
    sim.update(LOOP_PERIOD_SECONDS);

    // A ball between the fingers physically stops them from closing any further.
    if (!Double.isNaN(gripStopAngle) && sim.getAngle() < gripStopAngle) {
      sim.setState(gripStopAngle, 0.0);
    }
  }

  @Override
  public void updateInputs(ClawInputs inputs) {
    inputs.angleRadians = sim.getAngle();
    inputs.velocityRadPerSec = sim.getVelocity();
    inputs.appliedVolts = RobotBase.isEnabled() ? requestedVolts : 0.0;
    inputs.currentAmps = Math.abs(sim.getCurrentDraw());
    inputs.hasBall = ballDetected;
  }

  @Override
  public void setVoltage(double volts) {
    requestedVolts = volts;
  }

  // ---- Sim-only hooks, called by the simulated world ----

  /** Stops the fingers from closing past {@code angleRadians}. Pass NaN to remove the stop. */
  public void setGripStop(double angleRadians) {
    gripStopAngle = angleRadians;
  }

  /** Sets what the simulated ball sensor reports. */
  public void setBallDetected(boolean detected) {
    ballDetected = detected;
  }

  /** The true simulated angle (no one-loop delay), for the simulated world. */
  public double getSimAngle() {
    return sim.getAngle();
  }

  public void resetSim() {
    sim.setState(STARTING_ANGLE, 0.0);
    requestedVolts = 0.0;
    gripStopAngle = Double.NaN;
    ballDetected = false;
  }
}
