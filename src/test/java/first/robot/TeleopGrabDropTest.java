package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.constants.FieldConstants;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderConstants;
import first.robot.subsystems.extender.ExtenderIOSim;
import first.robot.subsystems.extender.ExtenderMechanism;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class TeleopGrabDropTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize();
  }

  @BeforeEach
  void setUp() {
    Scheduler.getDefault().cancelAll();
    DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testTeleopPickUpAndDropBall() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO);

    // 1. Open claw (Z key equivalent) and raise elevator to clear table
    Scheduler.getDefault().schedule(claw.open());
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    stepUntil(() -> claw.atSetpoint() && elevator.atSetpoint(), 120, elevator, extender, claw, world);
    assertTrue(claw.atSetpoint());
    assertTrue(elevator.atSetpoint());

    // 2. Extend over ball (Right arrow equivalent)
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    stepUntil(extender::atSetpoint, 100, elevator, extender, claw, world);
    assertTrue(extender.atSetpoint());

    // 3. Lower onto ball (Down arrow equivalent)
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.TABLE_GRAB_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);
    assertTrue(elevator.atSetpoint());

    // 4. Close claw (X key equivalent)
    Scheduler.getDefault().schedule(claw.close());
    stepUntil(claw::hasBall, 80, elevator, extender, claw, world);
    assertTrue(claw.hasBall(), "Claw should have gripped the ball in teleop sequence");
    assertTrue(world.snapshot().ballHeld(), "SimWorld should record ball is held");

    // 5. Lift ball up (Up arrow equivalent)
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);
    assertTrue(world.snapshot().ballY() > FieldConstants.TABLE_HEIGHT + 0.10,
        "Held ball should lift up with elevator");

    // 6. Retract over drop target on floor (Left arrow equivalent)
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.TARGET_EXTENSION));
    stepUntil(extender::atSetpoint, 100, elevator, extender, claw, world);
    assertTrue(Math.abs(world.snapshot().ballX() - FieldConstants.TARGET_CENTER_X) < 0.05,
        "Ball should be centered above the floor target");

    // 6b. Lower arm first (Down arrow equivalent to SCORE_HEIGHT) so it doesn't bounce away
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.SCORE_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);
    assertTrue(elevator.atSetpoint());

    // 7. Open claw to drop the ball gently (Z key equivalent)
    Scheduler.getDefault().schedule(claw.open());
    stepUntil(() -> world.snapshot().scored(), 100, elevator, extender, claw, world);

    assertFalse(world.snapshot().ballHeld(), "Ball should no longer be held after claw opens");
    assertTrue(world.snapshot().scored(), "Ball dropped gently over target in teleop should score");
  }

  @Test
  void testTeleopHighDropBouncesAway() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO);

    // Pick up ball
    Scheduler.getDefault().schedule(claw.open());
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    stepUntil(() -> claw.atSetpoint() && elevator.atSetpoint(), 120, elevator, extender, claw, world);

    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    stepUntil(extender::atSetpoint, 100, elevator, extender, claw, world);

    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.TABLE_GRAB_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);

    Scheduler.getDefault().schedule(claw.close());
    stepUntil(claw::hasBall, 80, elevator, extender, claw, world);
    assertTrue(claw.hasBall());

    // Lift high and retract over target
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);

    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.TARGET_EXTENSION));
    stepUntil(extender::atSetpoint, 100, elevator, extender, claw, world);

    // Drop from HIGH without lowering arm
    Scheduler.getDefault().schedule(claw.open());
    stepUntil(claw::atSetpoint, 60, elevator, extender, claw, world);

    // Step physics for 150 ticks (~3 seconds) to let bounce finish
    for (int i = 0; i < 150; i++) {
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
    }

    assertFalse(world.snapshot().ballHeld());
    assertFalse(world.snapshot().scored(), "Ball dropped from high should bounce away and NOT score");
    assertTrue(world.snapshot().ballX() > FieldConstants.TARGET_RIGHT_X, "Ball should bounce past target");
  }

  @Test
  void testTeleopClosedClawMovingOverBallDoesNotGrabOrPopOpen() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO);

    // Claw starts closed at 0.0 radians
    assertEquals(first.robot.subsystems.claw.ClawConstants.CLOSED_ANGLE, claw.getAngle(), 1e-3);

    // 1. Raise elevator and extend over ball WITHOUT opening claw
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);

    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    stepUntil(extender::atSetpoint, 100, elevator, extender, claw, world);

    // 2. Lower elevator directly over ball while claw remains closed
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.TABLE_GRAB_HEIGHT));
    stepUntil(elevator::atSetpoint, 100, elevator, extender, claw, world);

    // Run for 50 ticks in this overlapping position
    for (int i = 0; i < 50; i++) {
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
    }

    // Verify claw did NOT pop open and did NOT grab the ball
    assertFalse(claw.hasBall(), "Claw must not detect ball when overlapping while closed");
    assertFalse(world.snapshot().ballHeld(), "World must not record ball as held");
    assertEquals(
        first.robot.subsystems.claw.ClawConstants.CLOSED_ANGLE,
        claw.getAngle(),
        0.05,
        "Claw must stay closed and not pop open");

    // 3. Now open the claw while in position
    Scheduler.getDefault().schedule(claw.open());
    stepUntil(claw::atSetpoint, 80, elevator, extender, claw, world);
    assertTrue(claw.atSetpoint());
    assertFalse(claw.hasBall(), "Opening claw over ball does not grab yet");

    // 4. Now close the claw onto the ball
    Scheduler.getDefault().schedule(claw.close());
    stepUntil(claw::hasBall, 80, elevator, extender, claw, world);

    assertTrue(claw.hasBall(), "Closing claw onto ball after opening must grab it");
    assertTrue(world.snapshot().ballHeld(), "SimWorld must register ball is held");
    assertEquals(
        FieldConstants.BALL_CONTACT_ANGLE,
        claw.getAngle(),
        0.05,
        "Claw fingers should be stopped at contact angle around ball");
  }

  @Test
  void testTeleopActivationAndKeyboardControl() {
    try (Robot robot = new Robot()) {
      robot.simulationInit();
      SimWorld world = robot.getSimWorld();
      assertNotNull(world);

      // Request Teleop (equivalent to clicking "Teleop (T)" button or pressing T)
      world.requestTeleop();
      robot.simulationPeriodic();
      robot.robotPeriodic();

      assertTrue(org.wpilib.framework.RobotBase.isTeleop(), "Robot should switch to Teleop mode");
      assertTrue(org.wpilib.framework.RobotBase.isEnabled(), "Robot should be enabled in Teleop mode");
      assertEquals("Teleop", world.snapshot().mode());

      // Hold Up arrow key -> elevator should move up
      double initialHeight = robot.elevator.getPosition();
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_ELEVATOR_UP, true);
      for (int i = 0; i < 30; i++) {
        robot.simulationPeriodic();
        robot.robotPeriodic();
      }
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_ELEVATOR_UP, false);
      assertTrue(robot.elevator.getPosition() > initialHeight + 0.10, "Elevator should raise while Up key is pressed");

      // Hold Right arrow key -> extender should move out
      double initialExt = robot.extender.getPosition();
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_EXTENDER_OUT, true);
      for (int i = 0; i < 30; i++) {
        robot.simulationPeriodic();
        robot.robotPeriodic();
      }
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_EXTENDER_OUT, false);
      assertTrue(robot.extender.getPosition() > initialExt + 0.05, "Extender should extend while Right key is pressed");

      // Press Z -> claw should open
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_CLAW_OPEN, true);
      robot.simulationPeriodic();
      robot.robotPeriodic();
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_CLAW_OPEN, false);
      for (int i = 0; i < 40; i++) {
        robot.simulationPeriodic();
        robot.robotPeriodic();
      }
      assertTrue(robot.claw.getAngle() > first.robot.subsystems.claw.ClawConstants.OPEN_ANGLE - 0.10, "Claw should open on Z key");

      // Hold D key -> drive should move forward
      double initialDrive = robot.drive.getPosition();
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_DRIVE_FORWARD, true);
      for (int i = 0; i < 40; i++) {
        robot.simulationPeriodic();
        robot.robotPeriodic();
      }
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_DRIVE_FORWARD, false);
      assertTrue(robot.drive.getPosition() > initialDrive + 0.10, "Drive should move forward while D key is pressed");

      // Hold A key -> drive should move backward
      double driveAfterForward = robot.drive.getPosition();
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_DRIVE_BACKWARD, true);
      for (int i = 0; i < 40; i++) {
        robot.simulationPeriodic();
        robot.robotPeriodic();
      }
      robot.keyboard.setPressed(first.robot.constants.DriverConstants.KEY_DRIVE_BACKWARD, false);
      assertTrue(robot.drive.getPosition() < driveAfterForward - 0.10, "Drive should move backward while A key is pressed");

      // Request Disable (equivalent to clicking "Disable" button)
      world.requestDisable();
      robot.simulationPeriodic();
      robot.robotPeriodic();
    }
    assertFalse(org.wpilib.framework.RobotBase.isEnabled(), "Robot should be disabled after disable request");
  }

  private void stepUntil(
      java.util.function.BooleanSupplier condition,
      int maxTicks,
      ElevatorMechanism elevator,
      ExtenderMechanism extender,
      ClawMechanism claw,
      SimWorld world) {
    for (int i = 0; i < maxTicks; i++) {
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
      if (condition.getAsBoolean()) {
        break;
      }
    }
  }
}
