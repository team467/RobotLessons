package frc.robot.subsystems.springloadedextender;

import org.littletonrobotics.junction.AutoLog;

public interface SpringLoadedExtenderIO {

  @AutoLog
  class SpringLoadedExtenderIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double current = 0.0;
    public double temp = 0.0;

    public boolean reverseLimitSwitch = false;
    public boolean ratchetLocked = false;
  }

  default void updateInputs(SpringLoadedExtenderIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setVoltageWhileHold(double volts) {}

  default void resetEncoderPosition() {}

  default void setRatchetLocked(boolean locked) {}
}
