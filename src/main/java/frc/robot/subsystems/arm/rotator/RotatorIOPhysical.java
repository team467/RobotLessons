package frc.robot.subsystems.arm.rotator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.robot.RobotConstants;

public class RotatorIOPhysical implements RotatorIO {

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxLimitSwitch highLimitSwitch;
  private final SparkMaxLimitSwitch lowLimitSwitch;

  public RotatorIOPhysical(int motorId, int ratchetSolenoidId) {
    motor = new CANSparkMax(motorId, MotorType.kBrushless);
    motor.setInverted(true);
    motor.enableVoltageCompensation(11);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(80);

    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(RobotConstants.get().armExtendConversionFactor);

    highLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    lowLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void updateInputs(RotatorIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.position = encoder.getPosition();
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.current = motor.getOutputCurrent();
    inputs.temp = motor.getMotorTemperature();
    inputs.highLimitSwitch = highLimitSwitch.isPressed();
    inputs.lowLimitSwitch = lowLimitSwitch.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void resetEncoderPosition() {
    encoder.setPosition(0);
  }
}
