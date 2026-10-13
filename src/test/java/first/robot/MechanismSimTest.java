package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderConstants;
import first.robot.subsystems.extender.ExtenderIOSim;
import first.robot.subsystems.extender.ExtenderMechanism;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class MechanismSimTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testElevatorMoveToSetpoint() {
    ElevatorIOSim io = new ElevatorIOSim();
    ElevatorMechanism elevator = new ElevatorMechanism(io);

    double target = ElevatorConstants.ABOVE_TABLE_HEIGHT;
    var cmd = elevator.moveTo(target);
    Scheduler.getDefault().schedule(cmd);

    // Run periodic loop for up to 2 seconds (100 ticks @ 20ms)
    for (int i = 0; i < 100; i++) {
      elevator.periodic();
      Scheduler.getDefault().run();
      if (elevator.atSetpoint()) {
        break;
      }
    }

    assertTrue(elevator.atSetpoint(), "Elevator should reach setpoint");
    assertEquals(target, elevator.getPosition(), ElevatorConstants.TOLERANCE_METERS);
  }

  @Test
  void testExtenderMoveToSetpoint() {
    ExtenderIOSim io = new ExtenderIOSim();
    ExtenderMechanism extender = new ExtenderMechanism(io);

    double target = ExtenderConstants.BALL_EXTENSION;
    var cmd = extender.moveTo(target);
    Scheduler.getDefault().schedule(cmd);

    for (int i = 0; i < 100; i++) {
      extender.periodic();
      Scheduler.getDefault().run();
      if (extender.atSetpoint()) {
        break;
      }
    }

    assertTrue(extender.atSetpoint(), "Extender should reach setpoint");
    assertEquals(target, extender.getPosition(), ExtenderConstants.TOLERANCE_METERS);
  }

  @Test
  void testClawMoveToSetpoint() {
    ClawIOSim io = new ClawIOSim();
    ClawMechanism claw = new ClawMechanism(io);

    var openCmd = claw.open();
    Scheduler.getDefault().schedule(openCmd);

    for (int i = 0; i < 50; i++) {
      claw.periodic();
      Scheduler.getDefault().run();
      if (claw.atSetpoint()) {
        break;
      }
    }

    assertTrue(claw.atSetpoint(), "Claw should open to setpoint");
  }
}
