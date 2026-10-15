package first.robot.subsystems.claw;

/** The hardware layer for the claw. See {@link ClawIOSim} for the simulated version. */
public interface ClawIO {
  /** Sensor readings, refreshed once per loop by {@link #updateInputs}. */
  class ClawInputs {
    /** How far open the fingers are, in radians (0 = closed). */
    public double angleRadians = 0.0;

    /** Finger velocity, in radians per second. Positive is opening. */
    public double velocityRadPerSec = 0.0;

    /** Voltage actually applied to the motor. */
    public double appliedVolts = 0.0;

    /** Current drawn by the motor, in amps. */
    public double currentAmps = 0.0;

    /**
     * True when a ball is held between the fingers. On a real robot this could come from a
     * beam-break sensor or a limit switch in the palm.
     */
    public boolean hasBall = false;
  }

  /** Reads the latest sensor values into {@code inputs}. */
  void updateInputs(ClawInputs inputs);

  /** Commands the motor with a voltage. Positive opens the claw, negative closes it. */
  void setVoltage(double volts);

  /** Updates simulation physics (called during simulationPeriodic). Default does nothing for real hardware. */
  default void updateSim() {}
}
