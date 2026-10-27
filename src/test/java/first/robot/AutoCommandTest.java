package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.auto.ScoreBallAuto;
import first.robot.orchestration.BallCommands;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.extender.Extender;
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
    Elevator elevator = new Elevator();
    Extender extender = new Extender();
    Claw claw = new Claw();

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
    Elevator elevator = new Elevator();
    Extender extender = new Extender();
    Claw claw = new Claw();
    Drive drive = new Drive();

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
