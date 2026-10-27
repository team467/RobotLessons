package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Meters;

import first.robot.auto.ShootBothBallsAuto;
import first.robot.sim.SimMechanical;
import first.robot.sim.SimWorld;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.kicker.Kicker;
import first.robot.subsystems.kicker.KickerConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

public class ShootBothBallsAutoTest {
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
  void testKickerPullBackRatchetLockAndSolenoidKick() {
    Kicker kicker = new Kicker();

    // 1. Pull back the kicker against the spring
    Scheduler.getDefault().schedule(kicker.pullBack());
    for (int i = 0; i < 60; i++) {
      Scheduler.getDefault().run();
      if (kicker.isLoaded()) {
        break;
      }
    }

    assertTrue(kicker.isLoaded(), "Kicker should reach loaded pullback position");
    assertTrue(kicker.isRatchetLocked(), "Ratchet should be locked");

    // 2. Let default hold() command run with 0V and verify ratchet holds the spring in place
    for (int i = 0; i < 20; i++) {
      Scheduler.getDefault().run();
    }
    assertEquals(0.0, kicker.getAppliedVolts(), 1e-4, "Hold command should apply 0V");
    assertTrue(
        kicker.getPosition().in(Meters) >= KickerConstants.LOADED_PULLBACK.in(Meters) - KickerConstants.TOLERANCE.in(Meters),
        "Ratchet should hold the plunger pulled back with 0V motor output");

    // 3. Release solenoid to kick
    Scheduler.getDefault().schedule(kicker.kick());
    for (int i = 0; i < 30; i++) {
      Scheduler.getDefault().run();
    }

    assertEquals(
        KickerConstants.MIN_PULLBACK.in(Meters),
        kicker.getPosition().in(Meters),
        KickerConstants.TOLERANCE.in(Meters),
        "Solenoid release should snap the plunger back to 0");
    assertFalse(kicker.isLoaded(), "Kicker should no longer be loaded after kick");
  }

  @Test
  void testShootBothBallsIntoBasketAuto() {
    Elevator elevator = new Elevator();
    Extender extender = new Extender();
    Claw claw = new Claw();
    Drive drive = new Drive();
    Kicker kicker = new Kicker();

    SimWorld world = new SimWorld(elevator, extender, claw, drive, kicker);

    Command auto =
        ShootBothBallsAuto.shootBothBallsRoutine(drive, elevator, extender, claw, kicker);
    Scheduler.getDefault().schedule(auto);

    boolean bothScored = false;
    for (int i = 0; i < 1200; i++) {
      Scheduler.getDefault().run();
      world.update();

      SimWorld.Snapshot s = world.snapshot();
      if (s.scored() && s.ball2Scored() && s.basketScored()) {
        bothScored = true;
        break;
      }
    }

    SimWorld.Snapshot finalSnap = world.snapshot();
    assertTrue(finalSnap.scored(), "Ball 1 should be scored in the basket");
    assertTrue(finalSnap.ball2Scored(), "Ball 2 should be scored in the basket");
    assertTrue(finalSnap.basketScored(), "Basket target should register scored balls");
    assertTrue(bothScored, "Both balls should be shot into the elevated basket during auto");
    assertFalse(finalSnap.tableContact(), "Robot should not hit the table during the routine");
  }

  @Test
  void testKickerDoesNothingWhenBallTooCloseToRobot() {
    Elevator elevator = new Elevator();
    Extender extender = new Extender();
    Claw claw = new Claw();
    Drive drive = new Drive();
    Kicker kicker = new Kicker();

    SimWorld world = new SimWorld(elevator, extender, claw, drive, kicker);

    // Grab Ball 2 at start zone, then retract extender all the way to 0.0m (too close to the robot)
    Command grabAndRetractTooClose =
        Command.noRequirements(
                coroutine -> {
                  coroutine.await(
                      first.robot.orchestration.BallCommands.grabBallFromGround(
                          elevator, extender, claw));
                  coroutine.awaitAll(
                      elevator.moveTo(0.60),
                      extender.moveTo(0.0),
                      kicker.pullBack());
                  coroutine.await(kicker.kick());
                })
            .named("Grab And Retract Too Close");
    Scheduler.getDefault().schedule(grabAndRetractTooClose);

    for (int i = 0; i < 350; i++) {
      Scheduler.getDefault().run();
      world.update();
    }

    assertEquals(0.0, extender.getPosition().in(Meters), 0.03, "Extender should be retracted to 0.0m");
    assertEquals(0.60, elevator.getPosition().in(Meters), 0.03, "Elevator should be at 0.60m");
    assertFalse(kicker.isLoaded(), "Kicker should have released and kicked");
    assertTrue(
        world.snapshot().ball2Held(),
        "Kicking when extender=0.0m (too close to robot) should not launch the held ball");
  }
}
