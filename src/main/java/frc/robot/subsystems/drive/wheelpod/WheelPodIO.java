package frc.robot.subsystems.drive.wheelpod;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConstants;

public interface WheelPodIO {
  @AutoLog
  class WheelPodIOInputs {
    public final double wheelRadius = (RobotConstants.get().moduleWheelDiameter() / 2);

    public double drivePositionInMeters = 0.0;
    public double driveVelocityInMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public double turnPositionAbsoluteRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    // Wheel Pod Derived Values
    public Rotation2d angle = new Rotation2d(0.0);
    public SwerveModulePosition position = new SwerveModulePosition();
    public SwerveModuleState state = new SwerveModuleState();
  }

  default void updateInputs(WheelPodIOInputs inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setTurnVoltage(double volts) {}

  default void setDriveBrakeMode(boolean brake) {}

  default void setTurnBrakeMode(boolean brake) {}

}
