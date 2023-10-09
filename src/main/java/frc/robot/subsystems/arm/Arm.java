package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.arm.extender.SpringLoadedExtender;
import frc.robot.subsystems.arm.ArmPositionConstants.ArmPosition;
import frc.robot.subsystems.arm.extender.ArmExtenderIO;
import frc.robot.subsystems.arm.rotator.Rotator;
import frc.robot.subsystems.arm.rotator.RotatorIO;

public class Arm extends SubsystemBase {

  private static final double EXTEND_CALIBRATION_POSITION = 0.01;
  private static final double SAFE_RETRACT_NON_HOME = 0.05;

  private final Logger logger = Logger.getInstance();

  private final SpringLoadedExtender extender;
  private final Rotator rotator;

  private boolean isCalibrated = false;

  /**
   * Configures the arm subsystem
   *
   * @param armIO Arm IO
   */
  public Arm(ArmExtenderIO extenderIO, RotatorIO rotatorIO) {
    super();
    extender = new SpringLoadedExtender(extenderIO);
    rotator = new Rotator(rotatorIO);
  }

  // Commands

  /**
   * Force calibrated is used when we assume we are in the home position.
   * It is used at the start of the match to make calibration faster.
   * 
   * @return Command that sets the arm to the calibrated state.
   */
  public Command forceCalibrated() {
    return Commands.parallel(
      extender.forceCalibrated(),
      rotator.forceCalibrated(),
      Commands.run(() -> this.isCalibrated = true, this));
  }

  private Command rotateForCalibration() {    
    double rotatorStartPosition = rotator.position();
    return extender.calibrate()
      .andThen(extender.moveTo(EXTEND_CALIBRATION_POSITION)
      .andThen(rotator.manualDown().until(() -> (rotatorStartPosition - rotator.position() > 0.15 || rotator.lowLimitSwitch())))
      .andThen(rotator.checkCalibrated())
      );

    }

  public Command calibrate() {    
    return Commands.parallel( // Reset state as uncalibrated
        Commands.run(() -> this.isCalibrated = false, this),
        rotator.setUncalibrated(), 
        extender.setUncalibrated())
      .andThen(rotateForCalibration().until(rotator::isCalibrated)) // Call multiple times to prevent belt from breaking.
      .andThen(extender.calibrate()) // Need to run one last time since the arm is down.
      .andThen(Commands.run(() -> this.isCalibrated = true, this));
  }

  public Command moveTo(ArmPosition position) {
    if (!isCalibrated) {
      return Commands.none();
    }
    Command moveSequence;
    // if The rotate setpoint is too high, we need to retract the arm first
    // so that there is slack in the rope and it doesn't break.
    if (position.rotateSetpoint > 0.13) {
      moveSequence = extender.moveTo(RobotConstants.get().armExtendMinMeters).withTimeout(2.0);
    } else {
      moveSequence = extender.moveTo(SAFE_RETRACT_NON_HOME).withTimeout(1.0);
    }
    return moveSequence
        .andThen(rotator.rotate(position.rotateSetpoint))
        .andThen(extender.moveTo(position.extendSetpoint));
  }

}


