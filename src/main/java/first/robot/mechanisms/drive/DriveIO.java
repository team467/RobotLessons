package first.robot.mechanisms.drive;

/** Hardware interface for the drivetrain. */
public interface DriveIO {
  class DriveInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Reads latest sensors into inputs. */
  default void updateInputs(DriveInputs inputs) {}

  /** Sets motor voltage (positive drives forward toward table). */
  default void setVoltage(double volts) {}

  /** Resets measured position to 0.0 or a specified position. */
  default void resetPosition() {}

  default void resetPosition(double positionMeters) {}
}
