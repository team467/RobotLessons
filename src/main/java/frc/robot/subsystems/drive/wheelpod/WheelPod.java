package frc.robot.subsystems.drive.wheelpod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class WheelPod extends SubsystemBase {
  private final WheelPodIO io;
  private final WheelPodIOInputsAutoLogged inputs = new WheelPodIOInputsAutoLogged();
  private final int index;

  // Wheel Pod Derived Values
  private Rotation2d angle = new Rotation2d(0.0);
  private SwerveModulePosition position = new SwerveModulePosition();
  private SwerveModuleState state = new SwerveModuleState();

  private final SimpleMotorFeedforward driveFF =
      RobotConstants.get().wheelPodDriveFeedForward.getFeedforward();
  private final SimpleMotorFeedforward turnFF =
      RobotConstants.get().wheelPodTurnFeedForward.getFeedforward();
  private final ProfiledPIDController turnFB =
      RobotConstants.get()
          .moduleTurnFB
          .getProfiledPIDController(new TrapezoidProfile.Constraints(550.6, 7585));

  public WheelPod(WheelPodIO io, int index) {
    this.io = io;
    this.index = index;

    turnFB.enableContinuousInput(-Math.PI, Math.PI);
    this.io.updateInputs(this.inputs);
    turnFB.reset(this.inputs.turnPositionAbsoluteRad);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Drive" + index, inputs);

    // Updated derived values
    angle = new Rotation2d(MathUtil.angleModulus(inputs.turnPositionAbsoluteRad));
    position = new SwerveModulePosition(inputs.drivePositionInMeters, angle);
    state = new SwerveModuleState(inputs.driveVelocityInMetersPerSec, angle);
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state, boolean isStationary) {
    // Optimize state based on current angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, angle);

    io.setTurnVoltage(
        isStationary
            ? 0.0
            : turnFB.calculate(angle.getRadians(), optimizedState.angle.getRadians())
                + turnFF.calculate(turnFB.getSetpoint().velocity));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFB.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / inputs.wheelRadius;
    io.setDriveVoltage(driveFF.calculate(velocityRadPerSec));

    return optimizedState;
  }

  public void runCharacterization(double volts) {
    io.setTurnVoltage(
        turnFB.calculate(angle.getRadians(), 0.0)
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

  // Readouts

  public double getCharacterizationVelocity() {
    return inputs.driveVelocityInMetersPerSec;
  }

  public double drivePositionInMeters() {
    return inputs.drivePositionInMeters;
  }

  public double driveVelocityInMetersPerSec() {
    return inputs.driveVelocityInMetersPerSec;
  }

  public double driveAppliedVolts() {
    return inputs.driveAppliedVolts;
  }

  public double[] driveCurrentAmps() {
    return inputs.driveCurrentAmps;
  }

  public double turnPositionAbsoluteRad() {
    return inputs.turnPositionAbsoluteRad;
  }

  public double turnPositionRad() {
    return inputs.turnPositionRad;
  }

  public double turnVelocityRadPerSec() {
    return inputs.turnVelocityRadPerSec;
  }

  public double turnAppliedVolts() {
    return inputs.turnAppliedVolts;
  }

  public double[] turnCurrentAmps() {
    return inputs.turnCurrentAmps;
  }

  public Rotation2d angle() {
    return angle;
  }

  public SwerveModulePosition position() {
    return position;
  }

  public SwerveModuleState state() {
    return state;
  }
}
