package frc.robot.subsystems.flywheel;

import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double wheelRadius = (RobotConstants.get().flywheelDiameter / 2);

    public double positionInMeters = 0.0;
    public double velocityInMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperature = 0.0;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;

  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setBrakeMode(boolean brake) {}

}
