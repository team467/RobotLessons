package frc.robot.subsystems.drive.gyro;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    /** CCW positive, degrees */
    public double roll = 0.0;
    /** CCW positive, degrees */
    public double yaw = 0.0;
    /** CCW positive, degrees */
    public double pitch = 0.0;
    /** CCW positive, degrees */
    public double rollRate = 0.0;
    /** CCW positive, degrees */
    public double yawRate = 0.0;
    /** CCW positive, degrees */
    public double pitchRate = 0.0;
    /** [x,y,z] */
    public double[] gravVector = new double[] {0, 0, 1};
  }

  /**
   * Update the inputs of the IMU
   *
   * @param inputs The inputs to update
   */
  default void updateInputs(GyroIOInputs inputs) {}

  default void estimateFromMovement(double yawInRads) {};

}
