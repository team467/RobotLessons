package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.wheelpod.WheelPod;
import frc.robot.subsystems.drive.wheelpod.WheelPodIO;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final Gyro gyro;
  private final WheelPod[] wheelPods = new WheelPod[4];
  private final SwerveDriveKinematics kinematics = RobotConstants.get().kinematics;

  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private SwerveModuleState[] lastSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final SwerveModulePosition[] modulePositions;
  private SwerveDrivePoseEstimator odometry;
  private double previousYawInRad = 0.0;

  private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;

  public Drive(
      GyroIO gyroIO,
      WheelPodIO frontLeftWheelPodIO,
      WheelPodIO frontRightWheelPodIO,
      WheelPodIO backLeftWheelPodIO,
      WheelPodIO backRightWheelPodIO) {
    gyro = new Gyro(gyroIO);
    wheelPods[0] = new WheelPod(frontLeftWheelPodIO, 0);
    wheelPods[1] = new WheelPod(frontRightWheelPodIO, 1);
    wheelPods[2] = new WheelPod(backLeftWheelPodIO, 2);
    wheelPods[3] = new WheelPod(backRightWheelPodIO, 3);
    for (var wheelPod : wheelPods) {
      wheelPod.setBrakeMode(true);
      wheelPod.periodic();
    }

    modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = wheelPods[i].inputs().position;
    }
  }

  @Override
  public void periodic() {

    // Update inputs
    gyro.periodic();
    for (var wheelPod : wheelPods) {
      wheelPod.periodic();
    }

    // Run modules
    if (DriverStation.isDisabled()) {

      // Disable output while disabled
      for (var wheelPod : wheelPods) {
        wheelPod.stop();
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    } else if (isCharacterizing) {

      // Run in characterization mode
      for (var wheelPod : wheelPods) {
        wheelPod.runCharacterization(characterizationVolts);
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    } else {
      Twist2d setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      setpoint.vxMetersPerSecond * 0.020,
                      setpoint.vyMetersPerSecond * 0.020,
                      new Rotation2d(setpoint.omegaRadiansPerSecond * 0.020)));
      ChassisSpeeds adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / 0.020, setpointTwist.dy / 0.020, setpointTwist.dtheta / 0.020);
      // In normal mode, run the controllers for turning and driving based on the current
      // setpoint
      SwerveModuleState[] setpointStates =
          RobotConstants.get().kinematics.toSwerveModuleStates(adjustedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          setpointStates, RobotConstants.get().maxLinearSpeed);

      // Set to last angles if zero
      if (adjustedSpeeds.vxMetersPerSecond == 0.0
          && adjustedSpeeds.vyMetersPerSecond == 0.0
          && adjustedSpeeds.omegaRadiansPerSecond == 0) {
        for (int i = 0; i < 4; i++) {
          setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
        }
      }
      lastSetpointStates = setpointStates;

      boolean isStationary = false;
      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = wheelPods[i].runSetpoint(setpointStates[i], isStationary);
      }

      // Log setpoint states
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    this.estimate();
    Logger.getInstance().recordOutput("Odometry", getPose());
  }

  public void estimate() {

    SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      measuredPositions[i] = wheelPods[i].inputs().position;
    }

    if (gyro.isConnected()) {
      odometry.update(Rotation2d.fromDegrees(gyro.yaw()), measuredPositions);
      previousYawInRad = Units.degreesToRadians(gyro.yaw());
    } else {
      SwerveModuleState[] measuredStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        measuredStates[i] = wheelPods[i].inputs().state;
      }
      previousYawInRad += kinematics.toChassisSpeeds(measuredStates).omegaRadiansPerSecond * 0.02;
      odometry.update(new Rotation2d(previousYawInRad), measuredPositions);
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    isCharacterizing = false;
    setpoint = speeds;
  }

  public void stop() {
    isCharacterizing = false;
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    stop();
    for (int i = 0; i < 4; i++) {
      lastSetpointStates[i] =
          new SwerveModuleState(
              lastSetpointStates[i].speedMetersPerSecond,
              RobotConstants.get().moduleTranslations[i].getAngle());
    }
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = wheelPods[i].inputs().position;
    }
    odometry.resetPosition(Rotation2d.fromRadians(previousYawInRad), modulePositions, pose);
  }

  public void chassisDrive(double x, double y, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getPose().getRotation()));
    } else {
      runVelocity(new ChassisSpeeds(x, y, rot));
    }
  }

  public void runCharacterizationVolts(double volts) {
    isCharacterizing = true;
    characterizationVolts = volts;
  }

  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var wheelPod : wheelPods) {
      driveVelocityAverage += wheelPod.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }
}
