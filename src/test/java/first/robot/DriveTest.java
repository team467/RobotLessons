package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Meters;

import first.robot.sim.SimMechanical;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class DriveTest {
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
    Drive drive = new Drive();

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
    assertEquals(DriveConstants.TABLE_POSITION.in(Meters), drive.getPosition().in(Meters), DriveConstants.TOLERANCE.in(Meters) * 2);
  }

  @Test
  void testDriveToStartAndTable() {
    Drive drive = new Drive();

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
    assertEquals(DriveConstants.START_POSITION.in(Meters), drive.getPosition().in(Meters), DriveConstants.TOLERANCE.in(Meters) * 2);
  }
}
