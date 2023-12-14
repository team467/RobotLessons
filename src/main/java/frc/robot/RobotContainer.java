// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveWithDpad;
import frc.robot.subsystems.drive.DriveWithJoysticks;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.wheelpod.WheelPodIO;
import frc.robot.subsystems.drive.wheelpod.WheelPodIOSim;
import frc.robot.subsystems.drive.wheelpod.WheelPodIOSparkMax;
import frc.robot.subsystems.rotator.RotatorIO;
import frc.robot.subsystems.rotator.RotatorIOPhysical;
import frc.robot.subsystems.springloadedextender.SpringLoadedExtenderIO;
import frc.robot.subsystems.springloadedextender.SpringLoadedExtenderIOPhysical;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Arm arm;

  // private led;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private boolean isRobotOriented = true; // Workaround, change if needed

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.get().mode()) {
        // Real robot, instantiate hardware IO implementations
      case REAL -> {
        switch (RobotConstants.get().robot) {
          case ROBOT_COMP -> {
            // Transform3d front =
            //     new Transform3d(
            //         new Translation3d(6 * 0.01, -10 * 0.01 - Units.inchesToMeters(2.0), 42 *
            // 0.01),
            //         new Rotation3d());
            // Transform3d right =
            //     new Transform3d(
            //         new Translation3d(2 * 0.01, -12 * 0.01 - Units.inchesToMeters(2.0), 42 *
            // 0.01),
            //         new Rotation3d(0, 0, -0.5 * Math.PI));
            drive =
                new Drive(
                    new GyroIOPigeon2(17),
                    new WheelPodIOSparkMax(3, 4, 13, 0),
                    new WheelPodIOSparkMax(5, 6, 14, 1),
                    new WheelPodIOSparkMax(1, 2, 15, 2),
                    new WheelPodIOSparkMax(7, 8, 16, 3));
            // List.of(
            //     new VisionIOAprilTag("front", front, FieldConstants.aprilTags),
            //     new VisionIOAprilTag("right", right, FieldConstants.aprilTags)));
            arm =
                new Arm(
                    new SpringLoadedExtenderIOPhysical(
                        RobotConstants.get().armExtendMotorId,
                        RobotConstants.get().ratchetSolenoidId),
                    new RotatorIOPhysical(RobotConstants.get().armRotateMotorId));

            // effector =
            //     new Effector(
            //         new EffectorIOBrushed(
            //             RobotConstants.get().intakeMotorID(),
            //             RobotConstants.get().intakeCubeLimitSwitchID()));
          }
          case ROBOT_BRIEFCASE -> {
            drive =
                new Drive(
                    new GyroIO() {},
                    new WheelPodIO() {},
                    new WheelPodIO() {},
                    new WheelPodIO() {},
                    new WheelPodIO() {});
            // List.of(new VisionIO() {}));
            arm = new Arm(new SpringLoadedExtenderIO() {}, new RotatorIO() {});
            // effector = new Effector(new EffectorIO() {});

          }
          default -> {
            drive =
                new Drive(
                    new GyroIO() {},
                    new WheelPodIO() {},
                    new WheelPodIO() {},
                    new WheelPodIO() {},
                    new WheelPodIO() {});
            // List.of(new VisionIO() {}));
            arm = new Arm(new SpringLoadedExtenderIO() {}, new RotatorIO() {});
            // effector = new Effector(new EffectorIO() {});
          }
        }
      }
        // Sim robot, instantiate physics sim IO implementations
      case SIM -> {
        // Init subsystems
        // subsystem = new Subsystem(new SubsystemIOSim());
        drive =
            new Drive(
                new GyroIOSim() {},
                new WheelPodIOSim(),
                new WheelPodIOSim(),
                new WheelPodIOSim(),
                new WheelPodIOSim());
        // List.of(new VisionIO() {}));
        arm = new Arm(new SpringLoadedExtenderIO() {}, new RotatorIO() {});
        // effector = new Effector(new EffectorIO() {});
      }

        // Replayed robot, disable IO implementations
      default -> {
        // subsystem = new Subsystem(new SubsystemIO() {});
        drive =
            new Drive(
                new GyroIOSim() {},
                new WheelPodIOSim(),
                new WheelPodIOSim(),
                new WheelPodIOSim(),
                new WheelPodIOSim());
        // List.of(new VisionIO() {}));
        arm = new Arm(new SpringLoadedExtenderIO() {}, new RotatorIO() {});
        // effector = new Effector(new EffectorIO() {});
      }
    }
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    driverController.y().onTrue(Commands.runOnce(() -> isRobotOriented = !isRobotOriented));

    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> isRobotOriented // TODO: add toggle
            ));
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));
    driverController
        .pov(-1)
        .whileFalse(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));

    // led2023.setDefaultCommand(new LedRainbowCMD(led2023).ignoringDisable(true));
    // effector.setDefaultCommand(new HoldCMD(effector));

    // driverController.leftBumper().toggleOnTrue(new IntakeAndRaise(arm, effector));
    // driverController.rightBumper().toggleOnTrue(new ReleaseCMD(effector, arm));

    // Set the game piece type
    // operatorController.back().whileFalse(new WantConeCMD(effector));
    // operatorController.back().whileTrue(new WantCubeCMD(effector));

    // Manual arm movements
    operatorController.pov(90).whileTrue(arm.manualExtend());
    operatorController.pov(270).whileTrue(arm.manualRetract());
    operatorController.pov(0).whileTrue(arm.manualUp());
    operatorController.pov(180).whileTrue(arm.manualDown());
  
    // // Placing cone or cube, gets what it wants from in the command
    // operatorController.a().onTrue(new ArmScoreLowNodeCMD(arm));
    // operatorController.b().onTrue(new ArmScoreMidNodeCMD(arm, effector::wantsCone));
    // operatorController.y().onTrue(new ArmScoreHighNodeCMD(arm, effector::wantsCone));

    // // Home will be for movement
    // operatorController.x().onTrue(new ArmHomeCMD(arm, effector::wantsCone));
    // driverController.x().onTrue(new ArmHomeCMD(arm, effector::wantsCone));

    // Need to set to use automated movements, should be set in Autonomous init.
    driverController.back().onTrue(arm.calibrate());
    driverController.b().onTrue(arm.forceCalibrated());

    driverController.a().onTrue(Commands.runOnce(() -> drive.stopWithX(), drive));

    // // Manual arm movements
    // operatorController.leftStick().onTrue(new ArmStopCMD(arm));
    // operatorController.rightStick().onTrue(new ArmStopCMD(arm));
    // operatorController.leftBumper().onTrue(new ArmShelfCMD(arm, effector));
    // operatorController.rightBumper().onTrue(new ArmFloorCMD(arm, effector));

    // // Auto grid align
    // driverController.rightTrigger().whileTrue(new NewAlignToNode(drive, effector));
  }
  /** Runs post-creation actions and eliminates warning for not using the RobotContainer. */
  public void init() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
