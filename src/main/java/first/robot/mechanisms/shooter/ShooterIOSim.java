package first.robot.mechanisms.shooter;

import static first.robot.mechanisms.shooter.ShooterConstants.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;

/**
 * Simulated spring-loaded shooter: a NEO winch motor pulls a plunger back while a one-way ratchet
 * locks its position so it cannot slip forward when the motor stops. Releasing the solenoid
 * disengages the ratchet, letting the spring snap the plunger back to {@code 0.0} and kick the ball.
 */
public class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          GEARING,
          PLUNGER_MASS_KG,
          DRUM_RADIUS_METERS,
          MIN_PULLBACK,
          MAX_PULLBACK,
          false,
          STARTING_PULLBACK);

  private double requestedVolts = 0.0;
  private boolean solenoidReleased = false;
  private double ratchetFloor = STARTING_PULLBACK;
  private double pendingKickStrength = 0.0;

  @Override
  public void updateSim() {
    boolean enabled = RobotBase.isEnabled();
    boolean activeRelease = enabled && solenoidReleased;

    if (activeRelease) {
      double currentPos = sim.getPosition();
      if (ratchetFloor >= LOADED_PULLBACK - TOLERANCE_METERS) {
        pendingKickStrength = 1.0;
      } else if (ratchetFloor > 0.05) {
        pendingKickStrength = Math.clamp(currentPos / LOADED_PULLBACK, 0.0, 1.0);
      }
      ratchetFloor = MIN_PULLBACK;

      double nextPos = Math.max(MIN_PULLBACK, currentPos - SPRING_RELEASE_SPEED_MPS * LOOP_PERIOD_SECONDS);
      double nextVel = (nextPos > MIN_PULLBACK) ? -SPRING_RELEASE_SPEED_MPS : 0.0;
      sim.setInputVoltage(0.0);
      sim.setState(nextPos, nextVel);
    } else {
      double volts = enabled ? requestedVolts : 0.0;
      // With the one-way ratchet engaged, only positive pullback voltage can advance the plunger
      if (volts > 0.0) {
        sim.setInputVoltage(volts);
        sim.update(LOOP_PERIOD_SECONDS);
        if (sim.getPosition() > ratchetFloor) {
          ratchetFloor = sim.getPosition();
        }
        if (sim.getPosition() >= MAX_PULLBACK) {
          ratchetFloor = MAX_PULLBACK;
          sim.setState(MAX_PULLBACK, 0.0);
        }
      } else {
        sim.setInputVoltage(volts);
        sim.setState(ratchetFloor, 0.0);
      }
    }
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    boolean enabled = RobotBase.isEnabled();
    inputs.pullbackMeters = sim.getPosition();
    inputs.velocityMetersPerSec = sim.getVelocity();
    inputs.appliedVolts = enabled ? requestedVolts : 0.0;
    inputs.currentAmps = Math.abs(sim.getCurrentDraw());
    inputs.solenoidReleased = enabled && solenoidReleased;
    inputs.ratchetLocked = !inputs.solenoidReleased;
    inputs.isLoaded =
        inputs.ratchetLocked && inputs.pullbackMeters >= (LOADED_PULLBACK - TOLERANCE_METERS);
  }

  @Override
  public void setMotorVoltage(double volts) {
    requestedVolts = volts;
  }

  @Override
  public void setSolenoidReleased(boolean released) {
    solenoidReleased = released;
  }

  @Override
  public void reset() {
    sim.setState(STARTING_PULLBACK, 0.0);
    requestedVolts = 0.0;
    solenoidReleased = false;
    ratchetFloor = STARTING_PULLBACK;
    pendingKickStrength = 0.0;
  }

  /**
   * Consumes and returns the kick strength (1.0 = full loaded shot, 0.0 = no kick) if the solenoid
   * just released a cocked spring during this simulation step.
   */
  public double consumeKickStrength() {
    double strength = pendingKickStrength;
    pendingKickStrength = 0.0;
    return strength;
  }

  public double getSimPullback() {
    return sim.getPosition();
  }
}
