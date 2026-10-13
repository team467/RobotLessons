package first.robot;

import first.robot.constants.DriverConstants;
import first.robot.sim.SimKeyboard;
import first.robot.sim.SimWindow;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveIOSim;
import first.robot.subsystems.drive.DriveMechanism;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderIOSim;
import first.robot.subsystems.extender.ExtenderMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;

/**
 * Main robot container: coordinates the drivetrain, elevator (up/down), extender (in/out),
 * and claw (open/close) mechanisms.
 *
 * <p>Extends {@link OpModeRobot}, which automatically manages opmode lifecycle and discovers
 * registered {@code @Teleop} and {@code @Autonomous} classes.
 *
 * <p>Hardware IO layers are injected via simulation implementations (e.g. {@link DriveIOSim},
 * {@link ElevatorIOSim}), allowing the same mechanism logic to run on physical robot hardware
 * simply by swapping IO implementations (such as CAN SparkMax or TalonFX).
 */
public class Robot extends OpModeRobot {
  // Simulated hardware IO implementations. To run on a real robot, implement matching
  // hardware classes (e.g., DriveIOTalonFX, ElevatorIOSparkMax) and pass them in here.
  private final DriveIOSim driveIO = new DriveIOSim();
  private final ElevatorIOSim elevatorIO = new ElevatorIOSim();
  private final ExtenderIOSim extenderIO = new ExtenderIOSim();
  private final ClawIOSim clawIO = new ClawIOSim();

  /** Drivetrain mechanism for field translation between start and table. */
  public final DriveMechanism drive = new DriveMechanism(driveIO);

  /** Elevator mechanism controlling vertical carriage/arm motion (y-axis). */
  public final ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);

  /** Telescoping extender mechanism controlling horizontal arm reach (x-axis). */
  public final ExtenderMechanism extender = new ExtenderMechanism(extenderIO);

  /** Claw mechanism controlling pinchers/fingers to grasp and release game balls. */
  public final ClawMechanism claw = new ClawMechanism(clawIO);

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
    // 1. Refresh sensor inputs from hardware/simulation IO.
    drive.periodic();
    elevator.periodic();
    extender.periodic();
    claw.periodic();

    // 2. Step the Command3 scheduler to execute running commands and update motor outputs.
    Scheduler.getDefault().run();
  }

  @Override
  public void simulationInit() {
    // Initialize teleop button and keyboard bindings
    activateTeleop();

    // Create the simulation physics world with all mechanisms
    simWorld = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO, drive, driveIO);
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
