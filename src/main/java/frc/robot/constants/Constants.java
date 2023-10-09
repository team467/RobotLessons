package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;

public abstract class Constants {

  public static enum RobotType {
    ROBOT_COMP,
    ROBOT_SIMBOT,
    ROBOT_BRIEFCASE,
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM,
  }

  // All values are defaults that may be overriden by the robot-specific constants

  /** Robot Type is used for determining which versions of constants and subsystems to use */
  public final RobotType robot = null;

  /**
   * @return Check if robot is real, sim, or replay
   */
  public Mode mode() {
    return switch (robot) {
      case ROBOT_COMP, ROBOT_BRIEFCASE -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case ROBOT_SIMBOT -> Mode.SIM;
      default -> Mode.REAL;
    };
  }

  /**
   * @return Folder to put logs into (nullable)
   */
  public final String logFolder = "/media/sda1";

  public final double driveMaxCoastVelocity = 0.5;

  public final double maxLinearSpeed = Units.feetToMeters(14.5);

  public final double moduleWheelDiameter = Units.inchesToMeters(4);

  public final GearRatio moduleDriveGearRatio = new GearRatio(6.75, 1);

  public final GearRatio moduleTurnGearRatio = new GearRatio(12.8, 1);

  public final SimpleFeedforwardConstant wheelPodDriveFeedForward =
      new SimpleFeedforwardConstant(0.15026, 0.13682);

  public final SimpleFeedforwardConstant wheelPodTurnFeedForward =
      new SimpleFeedforwardConstant(0.16302, 0.0089689, 0.00034929);

  public final FeedbackConstant moduleTurnFB = new FeedbackConstant(3.2526, 0.05);

  public final Rotation2d[] absoluteAngleOffset =
      new Rotation2d[] {
        Rotation2d.fromDegrees(24.3),
        Rotation2d.fromDegrees(42.4),
        Rotation2d.fromDegrees(169.7),
        Rotation2d.fromDegrees(101.5),
      };

  public final double chassisDriveMaxVelocity = 2.0;

  public final double chassisDriveMaxAcceleration = 2.0;

  public final double chassisTurnMaxVelocity = 0.4;

  public final double chassisTurnMaxAcceleration = 0.4;

  public final FeedbackConstant chassisDriveFB = new FeedbackConstant(0.1);
  ;

  public final FeedbackConstant chassisTurnFB = new FeedbackConstant(0.5);
  ;

  public final int intakeMotorID = 11;

  public final int intakeCubeLimitSwitchID = 1;

  public final int ledChannel = 0;

  public int led2023LedCount = 10;

  // Arm Subsystem Constants

  public final int armExtendMotorId = 10;

  public final int armRotateMotorId = 9;

  public final double armExtendConversionFactor = 0.02;

  public final double armExtendMaxMeters = 0.65;

  public final double armExtendMinMeters = 0.0;

  public final double armRotateMaxMeters = 0.18;

  public final double armRotateMinMeters = 0.0;

  public final int ratchetSolenoidId = 1;

  public final int armRotateHighLimitSwitchId = 4;

  public final int armRotateLowLimitSwitchId = 5;

  public final double armRotateConversionFactor = 0.00236706;

  public final double armExtendMinDown = 0.2;
}
