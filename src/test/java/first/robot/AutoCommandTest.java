package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.auto.ScoreBallAuto;
import first.robot.orchestration.BallCommands;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimWorld;
import first.robot.mechanisms.claw.ClawMechanism;
import first.robot.mechanisms.drive.DriveMechanism;
import first.robot.mechanisms.elevator.ElevatorMechanism;
import first.robot.mechanisms.extender.ExtenderMechanism;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class AutoCommandTest {
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
  void testGrabBallAutoExecutesAndGrabsBall() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw);

    Command grabBall = BallCommands.grabBallFromTable(elevator, extender, claw);
    Scheduler.getDefault().schedule(grabBall);

    // Simulate for up to 6 seconds (300 ticks @ 20ms)
    boolean grabbed = false;
    for (int i = 0; i < 300; i++) {
      world.update();
      Scheduler.getDefault().run();

      if (claw.hasBall()) {
        grabbed = true;
        break;
      }
    }

    assertTrue(grabbed, "GrabBall command sequence should grab the ball");
    assertTrue(world.snapshot().ballHeld(), "World should register ball is held");
  }

  @Test
  void testScoreBallAutoCompletesAndScores() {
    ElevatorMechanism elevator = new ElevatorMechanism();
    ExtenderMechanism extender = new ExtenderMechanism();
    ClawMechanism claw = new ClawMechanism();
    DriveMechanism drive = new DriveMechanism();

    SimWorld world = new SimWorld(elevator, extender, claw, drive);

    Command scoreBall = ScoreBallAuto.driveAndScoreBall(drive, elevator, extender, claw);
    Scheduler.getDefault().schedule(scoreBall);

    boolean scored = false;
    for (int i = 0; i < 600; i++) {
      world.update();
      Scheduler.getDefault().run();

      if (world.snapshot().scored()) {
        scored = true;
        break;
      }
    }

    assertTrue(scored, "DriveAndScoreBall command sequence should drive to the table and score the ball");
  }
}
