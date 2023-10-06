package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  public Gyro(GyroIO io) {
    this.io = io;
    this.io.updateInputs(inputs);
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public double roll() {
    return inputs.roll;
  }

  public double yaw() {
    return inputs.yaw;
  }
}
