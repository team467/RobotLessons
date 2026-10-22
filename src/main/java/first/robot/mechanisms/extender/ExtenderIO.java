package first.robot.mechanisms.extender;

/**
 * The hardware layer for the extender. See {@link ExtenderIOSim} for the simulated version.
 */
public interface ExtenderIO {
  /** Sensor readings, refreshed once per loop by {@link #updateInputs}. */
  class ExtenderInputs {
    /** How far the arm is extended, in meters. */
    public double positionMeters = 0.0;

    /** Extension velocity, in meters per second. Positive is outward. */
    public double velocityMetersPerSec = 0.0;

    /** Voltage actually applied to the motor. */
    public double appliedVolts = 0.0;

    /** Current drawn by the motor, in amps. */
    public double currentAmps = 0.0;
  }

  /** Reads the latest sensor values into {@code inputs}. */
  void updateInputs(ExtenderInputs inputs);

  /** Commands the motor with a voltage. Positive extends the arm outward. */
  void setVoltage(double volts);

  /** Updates simulation physics (called during simulationPeriodic). Default does nothing for real hardware. */
  default void updateSim() {}
}
