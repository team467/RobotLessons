package first.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import first.robot.subsystems.claw.ClawConstants;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveIOSim;
import first.robot.subsystems.drive.DriveMechanism;
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

public class MotorStateSimTest {
  private ElevatorIOSim elevatorIO;
  private ElevatorMechanism elevator;
  private ExtenderIOSim extenderIO;
  private ExtenderMechanism extender;
  private ClawIOSim clawIO;
  private ClawMechanism claw;
  private DriveIOSim driveIO;
  private DriveMechanism drive;
  private SimWorld world;

  @BeforeAll
  static void initHal() {
    HAL.initialize();
  }

  @BeforeEach
  void setUp() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Scheduler.getDefault().cancelAll();

    elevatorIO = new ElevatorIOSim();
    elevator = new ElevatorMechanism(elevatorIO);
    extenderIO = new ExtenderIOSim();
    extender = new ExtenderMechanism(extenderIO);
    clawIO = new ClawIOSim();
    claw = new ClawMechanism(clawIO);
    driveIO = new DriveIOSim();
    drive = new DriveMechanism(driveIO);

    world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO, drive, driveIO);
  }

  private void step() {
    drive.periodic();
    elevator.periodic();
    extender.periodic();
    claw.periodic();
    world.update();
    Scheduler.getDefault().run();
  }

  @Test
  void testDisabledMotorsAreIdle() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step();

    SimWorld.Snapshot s = world.snapshot();
    assertEquals(SimWorld.MotorState.IDLE, s.elevatorMotorState(), "Elevator should be IDLE when disabled");
    assertEquals(SimWorld.MotorState.IDLE, s.extenderMotorState(), "Extender should be IDLE when disabled");
    assertEquals(SimWorld.MotorState.IDLE, s.clawMotorState(), "Claw should be IDLE when disabled");
    assertEquals(SimWorld.MotorState.IDLE, s.driveMotorState(), "Drive should be IDLE when disabled");
  }

  @Test
  void testElevatorRunningAndStalled() {
    // 1. Move elevator up - should be RUNNING
    drive.resetPosition(FieldConstants.ROBOT_TABLE_X);
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    step();
    step();
    assertEquals(SimWorld.MotorState.RUNNING, world.snapshot().elevatorMotorState(), "Elevator should be RUNNING while traveling up");

    // Let elevator reach setpoint and settle
    for (int i = 0; i < 100 && (Math.abs(elevator.getVelocity()) > 0.015 || !elevator.atSetpoint()); i++) {
      step();
    }
    assertEquals(SimWorld.MotorState.IDLE, world.snapshot().elevatorMotorState(), "Elevator should be IDLE when holding at setpoint");

    // 2. Extend arm over the table and then drive elevator down into the table (table collision stall)
    Scheduler.getDefault().schedule(extender.moveTo(FieldConstants.BALL_START_X - RobotConstants.WRIST_BASE_X));
    for (int i = 0; i < 80; i++) {
      step();
    }

    // Command elevator down to floor while over table
    Scheduler.getDefault().schedule(elevator.moveTo(0.35));
    for (int i = 0; i < 80; i++) {
      step();
    }

    assertTrue(world.snapshot().tableContact(), "Arm should make contact with the table top");
    assertEquals(SimWorld.MotorState.STALLED, world.snapshot().elevatorMotorState(), "Elevator should be STALLED when pressing into table limit");
  }

  @Test
  void testExtenderRunningAndStalled() {
    // 1. Move extender out - should be RUNNING
    drive.resetPosition(FieldConstants.ROBOT_TABLE_X);
    Scheduler.getDefault().schedule(extender.moveTo(0.50));
    step();
    step();
    assertEquals(SimWorld.MotorState.RUNNING, world.snapshot().extenderMotorState(), "Extender should be RUNNING while extending");

    // 2. Reset extender back in
    for (int i = 0; i < 40 && !extender.atSetpoint(); i++) {
      step();
    }
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.MIN_EXTENSION));
    for (int i = 0; i < 40 && !extender.atSetpoint(); i++) {
      step();
    }

    // Keep elevator low (MIN_HEIGHT), command extender into the side of the table
    Scheduler.getDefault().schedule(extender.moveTo(1.10));
    for (int i = 0; i < 80; i++) {
      step();
    }

    assertTrue(world.snapshot().tableContact(), "Extender should hit side of the table");
    assertEquals(SimWorld.MotorState.STALLED, world.snapshot().extenderMotorState(), "Extender should be STALLED when hitting table side limit");
  }

  @Test
  void testClawRunningAndStalled() {
    drive.resetPosition(FieldConstants.ROBOT_START_X);

    // 1. Open claw - should be RUNNING
    Scheduler.getDefault().schedule(claw.open());
    step();
    step();
    assertEquals(SimWorld.MotorState.RUNNING, world.snapshot().clawMotorState(), "Claw should be RUNNING while opening");

    for (int i = 0; i < 60 && (Math.abs(claw.getVelocity()) > 0.02 || !claw.atSetpoint()); i++) {
      step();
    }
    assertEquals(SimWorld.MotorState.IDLE, world.snapshot().clawMotorState(), "Claw should be IDLE when reached open setpoint");

    // 2. Extend over ground ball
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    for (int i = 0; i < 80; i++) {
      step();
    }

    // 3. Lower onto ground ball
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.GROUND_GRAB_HEIGHT));
    for (int i = 0; i < 80; i++) {
      step();
    }

    // 4. Close claw to grab Ball 2
    Scheduler.getDefault().schedule(claw.close());
    for (int i = 0; i < 80; i++) {
      step();
    }

    assertTrue(claw.hasBall(), "Claw should register ball detected");
    assertTrue(world.snapshot().ball2Held(), "Snapshot should register Ball 2 is held");
    assertEquals(SimWorld.MotorState.STALLED, world.snapshot().clawMotorState(), "Claw should be STALLED while applying grip voltage to ball");
  }

  @Test
  void testDriveRunning() {
    Scheduler.getDefault().schedule(drive.driveTo(FieldConstants.ROBOT_TABLE_X));
    step();
    step();
    assertEquals(SimWorld.MotorState.RUNNING, world.snapshot().driveMotorState(), "Drive should be RUNNING while driving to setpoint");
  }
}
