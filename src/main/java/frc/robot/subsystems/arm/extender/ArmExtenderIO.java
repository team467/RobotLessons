package frc.robot.subsystems.arm.extender;

import org.littletonrobotics.junction.AutoLog;

public interface ArmExtenderIO {

  @AutoLog
  class ArmExtenderIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double current = 0.0;
    public double temp = 0.0;

    public boolean reverseLimitSwitch = false;
    public boolean ratchetLocked = false;
  }

  default void updateInputs(ArmExtenderIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setVoltageWhileHold(double volts) {}

  default void resetEncoderPosition() {}

  default void setRatchetLocked(boolean locked) {}
}
