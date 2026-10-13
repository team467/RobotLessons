package first.robot.subsystems.elevator;

/**
 * The hardware layer for the elevator. The {@link ElevatorMechanism} only talks to the motor
 * through this interface, so the same mechanism code can run on a simulated motor ({@link
 * ElevatorIOSim}) or, later, a real one (for example an {@code ElevatorIOSparkMax}).
 */
public interface ElevatorIO {
  /** Sensor readings, refreshed once per loop by {@link #updateInputs}. */
  class ElevatorInputs {
    /** Carriage (wrist) height above the floor, in meters. */
    public double positionMeters = 0.0;

    /** Carriage velocity, in meters per second. Positive is up. */
    public double velocityMetersPerSec = 0.0;

    /** Voltage actually applied to the motor. */
    public double appliedVolts = 0.0;

    /** Current drawn by the motor, in amps. */
    public double currentAmps = 0.0;
  }

  /** Reads the latest sensor values into {@code inputs}. */
  void updateInputs(ElevatorInputs inputs);

  /** Commands the motor with a voltage. Positive moves the carriage up. */
  void setVoltage(double volts);
}
