package frc.robot.subsystems.flywheel;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotConstants;

// This class represents a FlyWheelIO implementation using SparkMax motor controllers.
public class FlyWheelIOSparkMax implements FlyWheelIO {
  private final CANSparkMax turnMotor;
  private final RelativeEncoder turnEncoder;
  private final WPI_CANCoder turnEncoderAbsolute;

  private int resetCount = 0;
  private int index;

  /**
   * @param driveMotorId The ID of the drive motor.
   * @param turnMotorId The ID of the turn motor.
   * @param turnAbsEncoderId The ID of the absolute encoder for the turn motor.
   * @param index The index of the module.
   */
  public FlyWheelIOSparkMax(int turnMotorId, int turnAbsEncoderId) {
    // Initialize motors and encoders
    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    turnEncoder = turnMotor.getEncoder();
    turnEncoderAbsolute = new WPI_CANCoder(turnAbsEncoderId);

    // Configure motors and encoders
    configureMotorsAndEncoders();

    this.index = index;

    turnEncoderAbsolute.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    this.index = index;
  }

  private void configureMotorsAndEncoders() {}

  /** Configures the motors and encoders. */
  public void updateInputs(FlyWheelIOInputsAutoLogged inputs) {

    // Reset the turn encoder sometimes when not moving
    if (turnEncoder.getVelocity() < Units.degreesToRadians(0.5)) {
      if (++resetCount >= 500) {
        resetCount = 0;
        turnEncoder.setPosition(
            Rotation2d.fromDegrees(turnEncoderAbsolute.getAbsolutePosition())
                .minus(RobotConstants.get().absoluteAngleOffset[index])
                .getRadians());
      }
    } else {
      resetCount = 0;
    }
    double turnPositionRad = turnEncoder.getPosition();

    Object turnPositionAbsoluteRad =
        Rotation2d.fromDegrees(turnEncoderAbsolute.getAbsolutePosition())
            .minus(RobotConstants.get().absoluteAngleOffset[index])
            .getRadians();
    Object turnAppliedVolts = turnMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    Object turnCurrentAmps = new double[] {turnMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double d) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnVoltage'");
  }

  public boolean setTurnBrakeMode(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnBrakeMode'");
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}
