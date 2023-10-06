package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
  private final WPI_Pigeon2 pigeon;
  private double[] rate = new double[3];
  private double[] gravVector = new double[3];

  private boolean connected = false;
  private double estimatedYaw = 0.0;

  public GyroIOPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    // Check to see if we have a connection to the gyro
    connected = pigeon.getLastError() == ErrorCode.OK;
    inputs.connected = connected;

    if (connected) {
      inputs.yaw = pigeon.getYaw();
      inputs.roll = pigeon.getRoll();
      inputs.pitch = pigeon.getPitch();
      pigeon.getRawGyro(rate);
      inputs.rollRate = rate[0];
      inputs.pitchRate = rate[1];
      inputs.yawRate = rate[2];
      pigeon.getGravityVector(gravVector);
      inputs.gravVector = gravVector;
    } else {
      inputs.yaw = estimatedYaw;
    }
  }

  public void estimateFromMovement(double yawInRads) {
    estimatedYaw = Units.degreesToRadians(Units.radiansToDegrees(estimatedYaw) + yawInRads);
  }
}
