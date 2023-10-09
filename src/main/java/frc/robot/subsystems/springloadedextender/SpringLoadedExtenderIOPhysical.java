package frc.robot.subsystems.springloadedextender;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.RobotConstants;

public class SpringLoadedExtenderIOPhysical implements SpringLoadedExtenderIO {

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxLimitSwitch reverseLimitSwitch;
  private final Relay ratchetSolenoid;
  private boolean ratchetLocked = false;

  public SpringLoadedExtenderIOPhysical(int motorId, int ratchetSolenoidId) {
    ratchetSolenoid = new Relay(ratchetSolenoidId, Direction.kForward);
    motor = new CANSparkMax(motorId, MotorType.kBrushless);
    reverseLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(RobotConstants.get().armExtendConversionFactor);

    motor.setInverted(true);
    motor.enableVoltageCompensation(11);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(80);
  }

  @Override
  public void updateInputs(SpringLoadedExtenderIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.position = encoder.getPosition();
    inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.current = motor.getOutputCurrent();
    inputs.temp = motor.getMotorTemperature();
    inputs.reverseLimitSwitch = reverseLimitSwitch.isPressed();
    inputs.ratchetLocked = ratchetLocked;
  }

  @Override
  public void setVoltage(double volts) {
    volts = Math.min(volts, 5.0);
    setRatchetLocked(volts == 0);
    motor.setVoltage(volts);
  }

  @Override
  public void setVoltageWhileHold(double volts) {
    setRatchetLocked(volts <= 0 && volts >= -1);
    motor.setVoltage(volts);
  }

  @Override
  public void resetEncoderPosition() {
    encoder.setPosition(0);
  }

  @Override
  public void setRatchetLocked(boolean locked) {
    ratchetSolenoid.set(locked ? Value.kOff : Value.kOn);
    ratchetLocked = locked;
  }
}
