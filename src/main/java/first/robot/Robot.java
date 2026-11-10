package first.robot;

import first.robot.constants.DriverConstants;
import first.robot.opmodes.DefaultTeleop;
import first.robot.sim.SimKeyboard;
import first.robot.sim.SimWindow;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.kicker.Kicker;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.OpModeRobot;

/**
 * Main robot container: coordinates the drivetrain, elevator (up/down), extender (in/out),
 * claw (open/close), and spring-loaded kicker subsystems.
 *
 * <p>Extends {@link OpModeRobot}, which automatically manages opmode lifecycle and discovers
 * registered {@code @Teleop} and {@code @Autonomous} classes.
 */
@Logged
public class Robot extends OpModeRobot {
  /** Drivetrain subsystem for field translation between start and table. */
  public final Drive drive = new Drive();

  /** Elevator subsystem controlling vertical carriage/arm motion (y-axis). */
  public final Elevator elevator = new Elevator();

  /** Telescoping extender subsystem controlling horizontal arm reach (x-axis). */
  public final Extender extender = new Extender();

  /** Claw subsystem controlling pinchers/fingers to grasp and release game balls. */
  public final Claw claw = new Claw();

  /** Spring-loaded kicker subsystem with pullback motor, ratchet lock, and release solenoid. */
  public final Kicker kicker = new Kicker();

  /** Driver gamepad controller on USB port 1. */
  public final CommandGamepad driverController = new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  /** Keyboard keys captured from the simulator window (active during simulation only). */
  public final SimKeyboard keyboard = new SimKeyboard();

  /** Physics and collision simulation world backing the graphical simulator. */
  private SimWorld simWorld;

  /** Active teleoperated control bindings. */
  private DefaultTeleop defaultTeleop;

  public Robot() {}

  /** Activates and binds the Teleop controls if not already initialized. */
  public void activateTeleop() {
    if (defaultTeleop == null) {
      defaultTeleop = new DefaultTeleop(this);
    }
  }

  @Override
  public void robotPeriodic() {
    // Step the Command3 scheduler (runs registered mechanism periodic callbacks, polls triggers,
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

  /** Returns the active simulation world instance (used by integration and unit tests). */
  SimWorld getSimWorld() {
    return simWorld;
  }
}
