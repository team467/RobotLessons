package first.robot;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.auto.ScoreBallAuto;
import first.robot.orchestration.BallCommands;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveIOSim;
import first.robot.subsystems.drive.DriveMechanism;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderIOSim;
import first.robot.subsystems.extender.ExtenderMechanism;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class AutoCommandTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void testGrabBallAutoExecutesAndGrabsBall() {
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);

    SimWorld world = new SimWorld(elevator, extender, claw, clawIO);

    Command grabBall = BallCommands.grabBallFromTable(elevator, extender, claw);
    Scheduler.getDefault().schedule(grabBall);

    // Simulate for up to 6 seconds (300 ticks @ 20ms)
    boolean grabbed = false;
    for (int i = 0; i < 300; i++) {
      elevator.periodic();
      extender.periodic();
      claw.periodic();
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
    ElevatorIOSim elevatorIO = new ElevatorIOSim();
    ExtenderIOSim extenderIO = new ExtenderIOSim();
    ClawIOSim clawIO = new ClawIOSim();
    DriveIOSim driveIO = new DriveIOSim();

    ElevatorMechanism elevator = new ElevatorMechanism(elevatorIO);
    ExtenderMechanism extender = new ExtenderMechanism(extenderIO);
    ClawMechanism claw = new ClawMechanism(clawIO);
    DriveMechanism drive = new DriveMechanism(driveIO);

    SimWorld world = new SimWorld(elevator, elevatorIO, extender, extenderIO, claw, clawIO, drive, driveIO);

    Command scoreBall = ScoreBallAuto.driveAndScoreBall(drive, elevator, extender, claw);
    Scheduler.getDefault().schedule(scoreBall);

    boolean scored = false;
    for (int i = 0; i < 600; i++) {
      drive.periodic();
      elevator.periodic();
      extender.periodic();
      claw.periodic();
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
