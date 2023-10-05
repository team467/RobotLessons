package frc.robot.subsystems.drive.wheelpod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class WheelPod {
  private final WheelPodIO io;
  private final WheelPodIOInputsAutoLogged inputs = new WheelPodIOInputsAutoLogged();
  private final int index;

  private final SimpleMotorFeedforward driveFF =
      RobotConstants.get().moduleDriveFF().getFeedforward();
  private final SimpleMotorFeedforward turnFF =
      RobotConstants.get().moduleTurnFF().getFeedforward();
  private final ProfiledPIDController turnFB =
      RobotConstants.get()
          .moduleTurnFB()
          .getProfiledPIDController(new TrapezoidProfile.Constraints(550.6, 7585));

  private final double wheelRadius = (RobotConstants.get().moduleWheelDiameter() / 2);

  public WheelPod(WheelPodIO io, int index) {
    this.io = io;
    this.index = index;

    turnFB.enableContinuousInput(-Math.PI, Math.PI);
    this.io.updateInputs(this.inputs);
    turnFB.reset(this.inputs.turnPositionAbsoluteRad);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Drive" + index, inputs);
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean isStationary) {
    // Optimize state based on current angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnVoltage(
        isStationary
            ? 0.0
            : turnFB.calculate(getAngle().getRadians(), optimizedState.angle.getRadians())
                + turnFF.calculate(turnFB.getSetpoint().velocity));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFB.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius;
    io.setDriveVoltage(driveFF.calculate(velocityRadPerSec));

    return optimizedState;
  }

  public void runCharacterization(double volts) {
    io.setTurnVoltage(
        turnFB.calculate(getAngle().getRadians(), 0.0)
            + turnFB.calculate(turnFB.getSetpoint().velocity));
    io.setDriveVoltage(volts);
  }

  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnPositionAbsoluteRad));
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
