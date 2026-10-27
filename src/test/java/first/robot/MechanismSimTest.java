package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.elevator.ElevatorConstants;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderConstants;
import first.robot.mechanisms.extender.ExtenderMechanism;
import first.robot.sim.SimMechanical;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class MechanismSimTest {
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
  void testElevatorMoveToSetpoint() {
    ElevatorMechanism elevator = new ElevatorMechanism();

    double target = ElevatorConstants.ABOVE_TABLE_HEIGHT;
    var cmd = elevator.moveTo(target);
    Scheduler.getDefault().schedule(cmd);

    // Run periodic loop for up to 2 seconds (100 ticks @ 20ms)
    for (int i = 0; i < 100; i++) {
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
    ExtenderMechanism extender = new ExtenderMechanism();

    double target = ExtenderConstants.BALL_EXTENSION;
    var cmd = extender.moveTo(target);
    Scheduler.getDefault().schedule(cmd);

    for (int i = 0; i < 100; i++) {
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
    ClawMechanism claw = new ClawMechanism();

    var openCmd = claw.open();
    Scheduler.getDefault().schedule(openCmd);

    for (int i = 0; i < 50; i++) {
      Scheduler.getDefault().run();
      if (claw.atSetpoint()) {
        break;
      }
    }

    assertTrue(claw.atSetpoint(), "Claw should open to setpoint");
  }
}
