package frc.robot.subsystems.drive.wheelpod;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class WheelPod  extends SubsystemBase {
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
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, inputs.angle);

    io.setTurnVoltage(
        isStationary
            ? 0.0
            : turnFB.calculate(inputs.angle.getRadians(), optimizedState.angle.getRadians())
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
        turnFB.calculate(inputs.angle.getRadians(), 0.0)
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

  public double getCharacterizationVelocity() {
    return inputs.driveVelocityInMetersPerSec;
  }

  public WheelPodIOInputsAutoLogged inputs() {
    return inputs;
  }

}
