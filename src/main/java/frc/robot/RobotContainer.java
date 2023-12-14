// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelIO;
import frc.robot.subsystems.flywheel.FlyWheelIOSim;
import frc.robot.subsystems.flywheel.FlyWheelIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Flywheel
  private final FlyWheel flywheel;

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
            flywheel = new FlyWheel(new FlyWheelIO() {});
          }
          case ROBOT_BRIEFCASE -> {
            flywheel = new FlyWheel(new FlyWheelIOSparkMax(1, 25));
          }
          default -> {
            flywheel = new FlyWheel(new FlyWheelIO() {});
          }
        }
      }
        // Sim robot, instantiate physics sim IO implementations
      case SIM -> {
        // Init subsystems
        // subsystem = new Subsystem(new SubsystemIOSim());

        flywheel = new FlyWheel(new FlyWheelIOSim());
      }

        // Replayed robot, disable IO implementations
      default -> {
        // subsystem = new Subsystem(new SubsystemIO() {});

        flywheel = new FlyWheel(new FlyWheelIO() {});
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
    // Flywheel
    operatorController.a().whileTrue(flywheel.startCommand());
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
