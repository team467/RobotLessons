package first.robot;

import first.robot.opmodes.DefaultTeleop;
import first.robot.sim.SimKeyboard;
import first.robot.sim.SimWindow;
import first.robot.sim.SimWorld;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.extender.Extender;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.OpModeRobot;

import static first.robot.subsystems.drive.DriveConstants.*;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Distance;


/**
 * Main robot container: coordinates the subsystems.
 *
 * <p>
 * Extends {@link OpModeRobot}, which automatically manages opmode lifecycle and
 * discovers registered {@code @Teleop} and {@code @Autonomous} classes.
 */
@Logged
public class Robot extends OpModeRobot {
  /** Drivetrain subsystem for driving the robot on the field. */
  public final Drive drive = new Drive();

  /** Telescoping extender subsystem controlling horizontal arm reach (x-axis). */
  public final Extender extender = new Extender();

  // Add new subsystems here.

  public final Elevator elevator = new Elevator();




  /**
   * 
   * DO NOT EDIT BELOW THIS LINE FOR 467 Sim Lessons.
   * 
   */

  /** Driver gamepad controller on USB port 1. */
  public final CommandGamepad driverController = new CommandGamepad(Schematic.DRIVER_CONTROLLER_PORT);

  /**
   * Keyboard keys captured from the simulator window (active during simulation
   * only).
   */
  public final SimKeyboard keyboard = new SimKeyboard();

  /** Physics and collision simulation world backing the graphical simulator. */
  private SimWorld simWorld;

  /** Active teleoperated control bindings. */
  private DefaultTeleop defaultTeleop;

  public Robot() {
  }

  /** Activates and binds the Teleop controls if not already initialized. */
  public void activateTeleop() {
    if (defaultTeleop == null) {
      defaultTeleop = new DefaultTeleop(this);
    }
  }

  @Override
  public void robotPeriodic() {
    // Step the Command3 scheduler (runs registered mechanism periodic callbacks,
    // polls triggers,
    // and executes running commands).
    Scheduler.getDefault().run();
    Epilogue.update(this);
  }

  @Override
  public void simulationInit() {
    // Initialize teleop button and keyboard bindings
    activateTeleop();
    Epilogue.update(this);

    // Create the simulation physics world
    simWorld = new SimWorld();
    simWorld.setTeleopActivator(this::activateTeleop);

    // Launch the Swing graphical user interface
    SimWindow.launch(simWorld, keyboard);
  }

  @Override
  public void simulationPeriodic() {
    // Step mechanism physics, ball collisions, and table contact limits
    simWorld.update();
  }

  /**
   * Returns the active simulation world instance (used by integration and unit
   * tests).
   */
  SimWorld getSimWorld() {
    return simWorld;
  }
}
