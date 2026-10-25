package first.robot.sim;

import first.robot.sim.SimMechanical.SimMotorEntry;

/**
 * A simulated REV SparkMax motor controller that connects by CAN ID to a motor physics model in
 * {@link SimMechanical}.
 *
 * <p>Allows {@code *IOSim} hardware classes to be written just like real robot hardware IO classes
 * (instantiating a motor controller by CAN ID, commanding voltage, and reading encoder position,
 * velocity, and current draw).
 */
public class FakeSparkMax {
  private final int canId;
  private final SimMotorEntry motor;
  private final FakeRelativeEncoder encoder;

  /**
   * Creates a {@link FakeSparkMax} attached to the simulated motor with {@code canId} in {@link
   * SimMechanical}.
   */
  public FakeSparkMax(int canId) {
    this.canId = canId;
    this.motor = SimMechanical.createMotor(canId);
    this.encoder = new FakeRelativeEncoder(motor);
  }

  /** Returns the CAN ID of this motor controller. */
  public int getDeviceId() {
    return canId;
  }

  /** Commands the motor output voltage. */
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  /** Sets the motor output as a duty cycle in {@code [-1.0, 1.0]} (scaled to 12 V nominal). */
  public void set(double speed) {
    motor.setVoltage(Math.clamp(speed, -1.0, 1.0) * 12.0);
  }

  /** Stops the motor (sets voltage to 0 V). */
  public void stopMotor() {
    motor.setVoltage(0.0);
  }

  /** Returns the voltage currently applied to the motor (0 V when the robot is disabled). */
  public double getAppliedVolts() {
    return motor.getAppliedVolts();
  }

  /** Returns the motor's output current draw in amps. */
  public double getOutputCurrent() {
    return motor.getCurrentAmps();
  }

  /** Returns the built-in encoder for reading position and velocity. */
  public FakeRelativeEncoder getEncoder() {
    return encoder;
  }

  /** Convenience method for reading the mechanism position directly from the motor. */
  public double getPosition() {
    return motor.getPosition();
  }

  /** Convenience method for reading the mechanism velocity directly from the motor. */
  public double getVelocity() {
    return motor.getVelocity();
  }

  /** Resets the mechanism position to {@code position}. */
  public void setPosition(double position) {
    motor.setPosition(position);
  }

  /** Simulated relative encoder attached to a {@link FakeSparkMax}. */
  public static class FakeRelativeEncoder {
    private final SimMotorEntry motor;

    private FakeRelativeEncoder(SimMotorEntry motor) {
      this.motor = motor;
    }

    /** Returns the mechanism position (meters or radians). */
    public double getPosition() {
      return motor.getPosition();
    }

    /** Returns the mechanism velocity (meters/sec or radians/sec). */
    public double getVelocity() {
      return motor.getVelocity();
    }

    /** Resets the encoder/mechanism position to {@code position}. */
    public void setPosition(double position) {
      motor.setPosition(position);
    }
  }
}
