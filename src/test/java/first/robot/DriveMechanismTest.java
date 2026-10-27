package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.mechanisms.drive.DriveConstants;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.sim.SimMechanical;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class DriveMechanismTest {
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
  void testDriveToSetpoint() {
    DriveMechanism drive = new DriveMechanism();

    Command cmd = drive.driveTo(DriveConstants.TABLE_POSITION);
    Scheduler.getDefault().schedule(cmd);

    // Simulate for up to 3 seconds (150 ticks @ 20ms)
    boolean reached = false;
    for (int i = 0; i < 150; i++) {
      Scheduler.getDefault().run();
      if (drive.atSetpoint()) {
        reached = true;
        break;
      }
    }

    assertTrue(reached, "Drive should reach table position setpoint");
    assertEquals(DriveConstants.TABLE_POSITION, drive.getPosition(), DriveConstants.TOLERANCE_METERS * 2);
  }

  @Test
  void testDriveToStartAndTable() {
    DriveMechanism drive = new DriveMechanism();

    // Start by moving to table
    Scheduler.getDefault().schedule(drive.driveToTable());
    for (int i = 0; i < 150; i++) {
      Scheduler.getDefault().run();
      if (drive.atSetpoint()) break;
    }
    assertTrue(drive.atSetpoint());

    // Now drive back to start
    Scheduler.getDefault().schedule(drive.driveToStart());
    for (int i = 0; i < 150; i++) {
      Scheduler.getDefault().run();
      if (drive.atSetpoint()) break;
    }
    assertTrue(drive.atSetpoint());
    assertEquals(DriveConstants.START_POSITION, drive.getPosition(), DriveConstants.TOLERANCE_METERS * 2);
  }
}
