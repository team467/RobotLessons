package frc.robot.subsystems.drive.wheelpod;

import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

public interface WheelPodIO {
  @AutoLog
  class WheelPodIOInputs {
    public double wheelRadius = (RobotConstants.get().moduleWheelDiameter / 2);

    public double drivePositionInMeters = 0.0;
    public double driveVelocityInMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public double turnPositionAbsoluteRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
  }

  default void updateInputs(WheelPodIOInputs inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setTurnVoltage(double volts) {}

  default void setDriveBrakeMode(boolean brake) {}

  default void setTurnBrakeMode(boolean brake) {}
}
