package first.robot.subsystems.shooter;

/**
 * Hardware interface for the spring-loaded {@link ShooterMechanism}: a pullback motor with a
 * one-way ratchet and a release solenoid.
 */
public interface ShooterIO {
  /** Sensor readings from the shooter hardware. */
  class ShooterInputs {
    /** How far back the plunger/spring is currently pulled, in meters (0 = fully released). */
    public double pullbackMeters = 0.0;

    /** Pullback velocity in meters per second. */
    public double velocityMetersPerSec = 0.0;

    /** Voltage currently applied to the pullback motor. */
    public double appliedVolts = 0.0;

    /** Current drawn by the pullback motor, in amps. */
    public double currentAmps = 0.0;

    /** True when the ratchet pawl is engaged, locking the spring so it cannot slip forward. */
    public boolean ratchetLocked = true;

    /** True when the solenoid is energized to disengage the ratchet and fire the spring. */
    public boolean solenoidReleased = false;

    /** True when the shooter is pulled back to the loaded position and locked by the ratchet. */
    public boolean isLoaded = false;
  }

  /** Steps simulation physics for the shooter. */
  default void updateSim() {}

  /** Fills {@code inputs} with the latest sensor readings. */
  default void updateInputs(ShooterInputs inputs) {}

  /** Commands the pullback motor voltage (positive = pull back against the spring). */
  default void setMotorVoltage(double volts) {}

  /** Sets whether the release solenoid is energized (true = release ratchet and kick). */
  default void setSolenoidReleased(boolean released) {}

  /** Resets the simulation state back to resting (unloaded). */
  default void reset() {}
}
