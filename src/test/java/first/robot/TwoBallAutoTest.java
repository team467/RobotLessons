package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.auto.TwoBallAuto;
import first.robot.constants.FieldConstants;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.DriveIOSim;
import first.robot.subsystems.drive.DriveMechanism;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderIOSim;
import first.robot.subsystems.extender.ExtenderMechanism;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class TwoBallAutoTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize();
  }

  @BeforeEach
  void setUp() {
    Scheduler.getDefault().cancelAll();
    DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testTwoBallRoutineExecution() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();
    DriveIOSim driveIO = new DriveIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);
    DriveMechanism drive = new DriveMechanism(driveIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO, drive, driveIO);

    Command twoBall = TwoBallAuto.twoBallRoutine(drive, elevator, extender, claw);
    Scheduler.getDefault().schedule(twoBall);

    // Simulate routine up to 26 seconds (1300 ticks @ 20ms)
    boolean completed = false;
    for (int i = 0; i < 1300; i++) {
      drive.periodic();
      elevator.periodic();
      extender.periodic();
      claw.periodic();
      world.update();
      Scheduler.getDefault().run();

      if (world.snapshot().scored() && world.snapshot().ball2Scored() && drive.atSetpoint()) {
        completed = true;
        break;
      }
    }

    assertTrue(world.snapshot().scored(), "Ball 1 should be scored in green floor target zone");
    assertTrue(world.snapshot().ball2Scored(), "Ball 2 should be scored in orange table target box");
    assertEquals(DriveConstants.TABLE_POSITION, drive.getPosition(), 0.08);
    assertTrue(world.snapshot().autoTimeSeconds() > 0.0, "Auto timer should have recorded time");
    assertTrue(world.snapshot().autoComplete(), "Auto should be marked complete when both balls scored");
    assertFalse(world.snapshot().autoRunning(), "Auto timer should stop when both balls scored");
    assertTrue(completed, "Two ball autonomous routine should complete all steps and score both balls");
  }
}

