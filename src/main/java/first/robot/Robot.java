package first.robot;

import first.robot.constants.DriverConstants;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderMechanism;
import first.robot.mechanisms.shooter.ShooterMechanism;
import first.robot.sim.SimKeyboard;
import first.robot.sim.SimWindow;
import first.robot.sim.SimWorld;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;

/**
 * Main robot container: coordinates the drivetrain, elevator (up/down), extender (in/out),
 * claw (open/close), and spring-loaded shooter mechanisms.
 *
 * <p>Extends {@link OpModeRobot}, which automatically manages opmode lifecycle and discovers
 * registered {@code @Teleop} and {@code @Autonomous} classes.
 */
public class Robot extends OpModeRobot {
  /** Drivetrain mechanism for field translation between start and table. */
  public final DriveMechanism drive = new DriveMechanism();

  /** Elevator mechanism controlling vertical carriage/arm motion (y-axis). */
  public final ElevatorMechanism elevator = new ElevatorMechanism();

  /** Telescoping extender mechanism controlling horizontal arm reach (x-axis). */
  public final ExtenderMechanism extender = new ExtenderMechanism();

  /** Claw mechanism controlling pinchers/fingers to grasp and release game balls. */
  public final ClawMechanism claw = new ClawMechanism();

  /** Spring-loaded shooter mechanism with pullback motor, ratchet lock, and release solenoid. */
  public final ShooterMechanism shooter = new ShooterMechanism();

  /** Driver gamepad controller on USB port 1. */
  final CommandGamepad controller = new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  /** Keyboard keys captured from the simulator window (active during simulation only). */
  final SimKeyboard keyboard = new SimKeyboard();

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
  }

  @Override
  public void simulationInit() {
    // Initialize teleop button and keyboard bindings
    activateTeleop();

    // Create the simulation physics world with all mechanisms
    simWorld = new SimWorld(elevator, extender, claw, drive, shooter);
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
