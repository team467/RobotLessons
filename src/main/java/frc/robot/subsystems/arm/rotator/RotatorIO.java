package frc.robot.subsystems.arm.rotator;

import org.littletonrobotics.junction.AutoLog;

public interface RotatorIO {

  @AutoLog
  class RotatorIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double current = 0.0;
    public double temp = 0.0;

    public boolean highLimitSwitch = false;
    public boolean lowLimitSwitch = false;
  }

  default void updateInputs(RotatorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void resetEncoderPosition() {}
}
