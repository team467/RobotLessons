package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Meters;

import first.robot.sim.SimMechanical;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.extender.ExtenderConstants;
import first.robot.subsystems.kicker.Kicker;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class ResetStateTest {
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
  void testResetStateBehavior() {
    Elevator elevator = new Elevator();
    Extender extender = new Extender();
    Claw claw = new Claw();
    Drive drive = new Drive();
    Kicker kicker = new Kicker();
    SimWorld world = new SimWorld(elevator, extender, claw, drive, kicker);

    // 1. Move elevator up, extender out, drive forward
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    Scheduler.getDefault().schedule(extender.moveTo(ExtenderConstants.BALL_EXTENSION));
    Scheduler.getDefault().schedule(drive.driveTo(DriveConstants.TABLE_POSITION));
    for (int i = 0; i < 40; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // 2. Click "Reset State"
    world.requestResetState();
    world.update();
    Scheduler.getDefault().run();

    // Verify positions reset to starting positions
    assertEquals(ElevatorConstants.STARTING_HEIGHT.in(Meters), elevator.getPosition().in(Meters), 0.05,
        "Elevator position should reset to starting height");
    assertEquals(ElevatorConstants.STARTING_HEIGHT.in(Meters), elevator.getSetpoint().in(Meters), 0.01,
        "Elevator setpoint should reset to starting height");
    assertEquals(ExtenderConstants.STARTING_EXTENSION.in(Meters), extender.getPosition().in(Meters), 0.05,
        "Extender position should reset to starting extension");
    assertEquals(ExtenderConstants.STARTING_EXTENSION.in(Meters), extender.getSetpoint().in(Meters), 0.01,
        "Extender setpoint should reset to starting extension");
    assertEquals(DriveConstants.START_POSITION.in(Meters), drive.getPosition().in(Meters), 0.05,
        "Drive position should reset to starting position");

    // Verify default hold commands are active
    assertEquals("Elevator Hold", elevator.getCurrentCommandName(), "Elevator hold command should be active");
    assertEquals("Extender Hold", extender.getCurrentCommandName(), "Extender hold command should be active");
    assertEquals("Drive Hold", drive.getCurrentCommandName(), "Drive hold command should be active");

    // 3. Run 50 ticks to ensure stability (no drift, no limit violations)
    for (int i = 0; i < 50; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    assertEquals(ElevatorConstants.STARTING_HEIGHT.in(Meters), elevator.getPosition().in(Meters), 0.05,
        "Elevator should maintain stable hold at starting height");
    assertEquals(ExtenderConstants.STARTING_EXTENSION.in(Meters), extender.getPosition().in(Meters), 0.05,
        "Extender should maintain stable hold at starting extension");
    assertEquals(DriveConstants.START_POSITION.in(Meters), drive.getPosition().in(Meters), 0.05,
        "Drive should maintain stable hold at starting position");

    // 4. Try commanding elevator motion to verify commands run normally after reset
    Scheduler.getDefault().schedule(elevator.moveTo(ElevatorConstants.ABOVE_TABLE_HEIGHT));
    for (int i = 0; i < 30; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    assertTrue(elevator.getPosition().in(Meters) > ElevatorConstants.STARTING_HEIGHT.in(Meters) + 0.05,
        "Elevator should move up towards target height");
  }
}
