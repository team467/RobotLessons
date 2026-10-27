package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Meters;

import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.drive.DriveConstants;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.mechanisms.elevator.ElevatorConstants;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderConstants;
import first.robot.mechanisms.extender.ExtenderMechanism;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimWorld;
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
    SimMechanical.resetAll();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testInitialDualBallWorldState() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();
    DriveMechanism drive = new DriveMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw, drive);
    SimWorld.Snapshot s = world.snapshot();

    assertEquals(DriveConstants.START_POSITION.in(Meters), s.drivePosition(), 1e-3);
    assertEquals(FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.BALL1_START_X.in(Meters), s.ball1FieldX(), 1e-3);
    assertEquals(FieldConstants.ROBOT_START_X.in(Meters) + FieldConstants.BALL2_START_X.in(Meters), s.ball2FieldX(), 1e-3);
    assertFalse(s.ballHeld());
    assertFalse(s.ball2Held());
    assertFalse(s.scored());
  }

  @Test
  void testGroundBallGrabAtStartLocation() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();
    DriveMechanism drive = new DriveMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw, drive);

    // Grab Ball 2 off the ground while parked at start:
    // 1. Open claw and extend over ground ball
    Scheduler.getDefault().schedule(claw.open());
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    for (int i = 0; i < 80; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // 2. Lower onto ground ball
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.GROUND_GRAB_HEIGHT));
    for (int i = 0; i < 80; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // 3. Close claw to grab Ball 2
    Scheduler.getDefault().schedule(claw.close());
    for (int i = 0; i < 60; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    assertTrue(claw.hasBall(), "Claw should detect ball grabbed from ground");
    assertTrue(world.snapshot().ball2Held(), "SimWorld should record Ball 2 is held");

    // 4. Drive forward with held ball and verify ball travels with robot
    Scheduler.getDefault().schedule(drive.driveTo(1.5));
    for (int i = 0; i < 150; i++) {
      world.update();
      Scheduler.getDefault().run();
      if (drive.atSetpoint()) break;
    }

    assertEquals(1.5, drive.getPosition().in(Meters), 0.05);
    assertTrue(world.snapshot().ball2Held(), "Ball 2 should remain held while driving");
    assertEquals(1.5 + RobotConstants.WRIST_BASE_X.in(Meters) + ExtenderConstants.BALL_EXTENSION.in(Meters), world.snapshot().ball2FieldX(), 0.08);
  }
}
