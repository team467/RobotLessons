package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotConstants;

public class FlywheelIOSparkMax implements FlywheelIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxLimitSwitch forwardLimitSwitch;
  private final SparkMaxLimitSwitch reverseLimitSwitch;

  public FlywheelIOSparkMax(int motorId) {
    motor = new CANSparkMax(motorId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    forwardLimitSwitch = motor.getForwardLimitSwitch(Type.kNormallyOpen);
    reverseLimitSwitch = motor.getReverseLimitSwitch(Type.kNormallyOpen);

    // Convert rotations to radians
    double rotsToRads =
        Units.rotationsToRadians(1)
            * RobotConstants.get().flywheelGearRatio.getRotationsPerInput();
    encoder.setPositionConversionFactor(rotsToRads);

    // Convert rotations per minute to radians per second
    encoder.setVelocityConversionFactor(rotsToRads / 60);

    // Invert motors
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
    motor.enableVoltageCompensation(12);
    motor.setSmartCurrentLimit(80);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityInMetersPerSec = encoder.getVelocity() * inputs.wheelRadius;
    inputs.positionInMeters = encoder.getPosition() * inputs.wheelRadius;
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperature = motor.getMotorTemperature();
    inputs.forwardLimitSwitch = forwardLimitSwitch.isPressed();
    inputs.reverseLimitSwitch = reverseLimitSwitch.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

}
