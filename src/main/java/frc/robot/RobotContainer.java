// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Flywheel flywheel;

  

  // private led;

  // Controllers
  // private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
            flywheel = new Flywheel(new FlywheelIO() {} );
          }
          case ROBOT_BRIEFCASE -> {
            flywheel = new Flywheel(new FlywheelIOSparkMax(RobotConstants.get().flywheelMotorId));
          }
          default -> {
            flywheel = new Flywheel(new FlywheelIOSim());
          }
        }
      }
        // Sim robot, instantiate physics sim IO implementations
      case SIM -> {
        flywheel = new Flywheel(new FlywheelIOSim());
      }

        // Replayed robot, disable IO implementations
      default -> {
        flywheel = new Flywheel(new FlywheelIO() { });
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

    // driverController.start();
    // led2023.setDefaultCommand(new LedRainbowCMD(led2023).ignoringDisable(true));

    // move the flywheel if the left trigger of the operator's joystick moves.
    operatorController.start();

    // Flywheel Controls
    flywheel.setDefaultCommand(flywheel.stop());
    operatorController.leftTrigger(0.1).whileTrue(flywheel.manualSpin(-0.5));
    operatorController.rightTrigger(0.1).whileTrue(flywheel.manualSpin(0.5));
    operatorController.a().whileTrue(flywheel.manualSpin(1.0));
    operatorController.x().whileTrue(flywheel.stop());
    operatorController.y().whileTrue(flywheel.spinToSpeed(20));

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
