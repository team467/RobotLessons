package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Meters;

import first.robot.sim.SimMechanical;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.extender.ExtenderConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.units.measure.Distance;

public class SubsystemSimTest {
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
    Elevator elevator = new Elevator();

    Distance target = ElevatorConstants.ABOVE_TABLE_HEIGHT;
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
    assertEquals(target.in(Meters), elevator.getPosition().in(Meters), ElevatorConstants.TOLERANCE.in(Meters));
  }

  @Test
  void testExtenderMoveToSetpoint() {
    Extender extender = new Extender();

    Distance target = ExtenderConstants.BALL_EXTENSION;
    var cmd = extender.moveTo(target);
    Scheduler.getDefault().schedule(cmd);

    for (int i = 0; i < 100; i++) {
      Scheduler.getDefault().run();
      if (extender.atSetpoint()) {
        break;
      }
    }

    assertTrue(extender.atSetpoint(), "Extender should reach setpoint");
    assertEquals(target.in(Meters), extender.getPosition().in(Meters), ExtenderConstants.TOLERANCE.in(Meters));
  }

  @Test
  void testClawMoveToSetpoint() {
    Claw claw = new Claw();

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
