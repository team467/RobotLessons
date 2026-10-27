package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderMechanism;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimWorld;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class TableContactTest {
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
  void testArmHittingTableTopSurface() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw);

    // Raise elevator above table and extend over the table
    Scheduler.getDefault().schedule(elevator.moveTo(first.robot.mechanisms.elevator.ElevatorConstants.ABOVE_TABLE_HEIGHT));
    Scheduler.getDefault().schedule(extender.moveTo(FieldConstants.BALL_START_X - RobotConstants.WRIST_BASE_X));
    for (int i = 0; i < 80; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // Now command elevator down to minimum height (trying to smash through table)
    Scheduler.getDefault().schedule(elevator.moveTo(0.35));
    for (int i = 0; i < 60; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // Table contact must be detected
    assertTrue(world.snapshot().tableContact(), "Table contact should be true when claw reaches table surface");

    // The claw must physically stop on the table top and NOT penetrate through it
    double clawAngle = SimMechanical.getClawMotor().getSimAngle();
    double clawDrop = 0.02 + RobotConstants.FINGER_LENGTH * Math.cos(clawAngle);
    double lowestClawY = elevator.getPosition() - clawDrop;
    assertTrue(lowestClawY >= FieldConstants.TABLE_HEIGHT - 0.02,
        "Claw must not penetrate below table surface (" + lowestClawY + " vs " + FieldConstants.TABLE_HEIGHT + ")");
  }

  @Test
  void testArmHittingTableSideEdge() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw);

    // Keep elevator low (below table top, e.g. 0.50m)
    // Try to extend out all the way into the side of the table
    Scheduler.getDefault().schedule(extender.moveTo(1.10));
    for (int i = 0; i < 80; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // Must detect collision with table side edge
    assertTrue(world.snapshot().tableContact(), "Extending into side of table should trigger table contact");

    // Wrist X must not penetrate through the table left edge (TABLE_LEFT_X = 0.70)
    double wristX = RobotConstants.WRIST_BASE_X + extender.getPosition();
    double clawTipX =
        wristX
            + 0.03
            + RobotConstants.FINGER_LENGTH
                * Math.sin(SimMechanical.getClawMotor().getSimAngle());
    assertTrue(clawTipX <= FieldConstants.TABLE_LEFT_X + 0.02,
        "Claw must not penetrate past table left edge (" + clawTipX + " vs " + FieldConstants.TABLE_LEFT_X + ")");
  }

  @Test
  void testArmHittingTableFromLeftDoesNotJumpVertically() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw);

    // Position elevator at 0.80m (wrist is above table height 0.75m, but claw hangs down to 0.56m)
    Scheduler.getDefault().schedule(elevator.moveTo(0.80));
    for (int i = 0; i < 50; i++) {
      world.update();
      Scheduler.getDefault().run();
    }
    assertEquals(0.80, elevator.getPosition(), 0.02);

    // Now try to extend into the table from the left
    Scheduler.getDefault().schedule(extender.moveTo(1.00));
    for (int i = 0; i < 80; i++) {
      world.update();
      Scheduler.getDefault().run();
    }

    // Must detect table contact
    assertTrue(world.snapshot().tableContact(), "Table contact should be true when hitting table from left");

    // Extender must stop at the left edge
    double wristX = RobotConstants.WRIST_BASE_X + extender.getPosition();
    double clawTipX =
        wristX
            + 0.03
            + RobotConstants.FINGER_LENGTH
                * Math.sin(SimMechanical.getClawMotor().getSimAngle());
    assertTrue(clawTipX <= FieldConstants.TABLE_LEFT_X + 0.02,
        "Claw must stop at the table left edge");

    // Elevator MUST NOT jump up vertically! It should remain around 0.80m
    assertEquals(0.80, elevator.getPosition(), 0.03,
        "Elevator must not jump up vertically when hitting table from left");
  }
}
