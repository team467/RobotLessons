package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.util.Units;

public class GyroIOSim implements GyroIO {
  private double[] rate = new double[3];
  private double[] gravVector = new double[3];

  private double estimatedYaw = 0.0;

  public GyroIOSim() {}

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    inputs.connected = false;
    inputs.yaw = estimatedYaw;
    inputs.roll = 0.0;
    inputs.pitch = 0.0;
    inputs.rollRate = rate[0];
    inputs.pitchRate = rate[1];
    inputs.yawRate = rate[2];
    inputs.gravVector = gravVector;
  }

  public void estimateFromMovement(double yawInRads) {
    estimatedYaw = Units.degreesToRadians(Units.radiansToDegrees(estimatedYaw) + yawInRads);
  }
}
