package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.constants.FieldConstants;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveConstants;
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

public class TwoBallSimTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize();
  }

  @BeforeEach
  void setUp() {
    Scheduler.getDefault().cancelAll();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testInitialDualBallWorldState() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();
    DriveIOSim driveIO = new DriveIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);
    DriveMechanism drive = new DriveMechanism(driveIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO, drive, driveIO);
    SimWorld.Snapshot s = world.snapshot();

    assertEquals(DriveConstants.START_POSITION, s.drivePosition(), 1e-3);
    assertEquals(FieldConstants.ROBOT_TABLE_X + FieldConstants.BALL1_START_X, s.ball1FieldX(), 1e-3);
    assertEquals(FieldConstants.ROBOT_START_X + FieldConstants.BALL2_START_X, s.ball2FieldX(), 1e-3);
    assertFalse(s.ballHeld());
    assertFalse(s.ball2Held());
    assertFalse(s.scored());
  }

  @Test
  void testGroundBallGrabAtStartLocation() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();
    DriveIOSim driveIO = new DriveIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);
    DriveMechanism drive = new DriveMechanism(driveIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO, drive, driveIO);

    // Grab Ball 2 off the ground while parked at start:
    // 1. Open claw and extend over ground ball
    Scheduler.getDefault().schedule(claw.open());
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    for (int i = 0; i < 80; i++) {
      drive.periodic();
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
    }

    // 2. Lower onto ground ball
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.GROUND_GRAB_HEIGHT));
    for (int i = 0; i < 80; i++) {
      drive.periodic();
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
    }

    // 3. Close claw to grab Ball 2
    Scheduler.getDefault().schedule(claw.close());
    for (int i = 0; i < 60; i++) {
      drive.periodic();
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
    }

    assertTrue(claw.hasBall(), "Claw should detect ball grabbed from ground");
    assertTrue(world.snapshot().ball2Held(), "SimWorld should record Ball 2 is held");

    // 4. Drive forward with held ball and verify ball travels with robot
    Scheduler.getDefault().schedule(drive.driveTo(1.5));
    for (int i = 0; i < 150; i++) {
      drive.periodic();
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();
      if (drive.atSetpoint()) break;
    }

    assertEquals(1.5, drive.getPosition(), 0.05);
    assertTrue(world.snapshot().ball2Held(), "Ball 2 should remain held while driving");
    assertEquals(1.5 + FieldConstants.WRIST_BASE_X + ExtenderConstants.BALL_EXTENSION, world.snapshot().ball2FieldX(), 0.08);
  }
}
