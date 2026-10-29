package first.robot.sim;

import static first.robot.constants.RobotConstants.WRIST_BASE_X;
import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.claw.ClawConstants;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.extender.Extender;
import first.robot.subsystems.extender.ExtenderConstants;
import first.robot.subsystems.kicker.Kicker;
import first.robot.subsystems.kicker.KickerConstants;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.framework.RobotBase;

/**
 * Glues the simulated mechanisms to the ball physics, and publishes a snapshot of everything for
 * the {@link SimWindow} to draw.
 *
 * <p>{@link #update()} runs on the robot thread. The window reads {@link #snapshot()} on the
 * Swing thread. Snapshots are immutable, so this is thread-safe.
 */
public class SimWorld {
  /** Operating state of a mechanism's motor for visual feedback in the simulator. */
  public enum MotorState {
    IDLE,
    RUNNING,
    STALLED
  }

  /** Everything the window needs to draw one frame. */
  public record Snapshot(
      double elevatorHeight,
      double elevatorSetpoint,
      double elevatorVolts,
      String elevatorCommand,
      double extenderPosition,
      double extenderSetpoint,
      double extenderVolts,
      String extenderCommand,
      double clawAngle,
      double clawSetpoint,
      double clawVolts,
      String clawCommand,
      double ballX,
      double ballY,
      boolean ballHeld,
      boolean scored,
      boolean tableContact,
      boolean enabled,
      String mode,
      double drivePosition,
      double driveSetpoint,
      double driveVolts,
      String driveCommand,
      double robotSideX,
      double ball2SideX,
      double ball2SideY,
      boolean ball2Held,
      boolean ball2Scored,
      double ball1FieldX,
      double ball2FieldX,
      double autoTimeSeconds,
      boolean autoRunning,
      boolean autoComplete,
      MotorState elevatorMotorState,
      MotorState extenderMotorState,
      MotorState clawMotorState,
      MotorState driveMotorState,
      boolean basketScored,
      double kickerPullback,
      double kickerSetpoint,
      double kickerVolts,
      boolean kickerRatchetLocked,
      boolean kickerSolenoidReleased,
      boolean kickerLoaded,
      String kickerCommand,
      MotorState kickerMotorState) {
    public double shooterPullback() { return kickerPullback; }
    public double shooterSetpoint() { return kickerSetpoint; }
    public double shooterVolts() { return kickerVolts; }
    public boolean shooterRatchetLocked() { return kickerRatchetLocked; }
    public boolean shooterSolenoidReleased() { return kickerSolenoidReleased; }
    public boolean shooterLoaded() { return kickerLoaded; }
    public String shooterCommand() { return kickerCommand; }
    public MotorState shooterMotorState() { return kickerMotorState; }
  }

  private final Elevator elevator;
  private final Extender extender;
  private final Claw claw;
  private final Drive drive;
  private final Kicker kicker;
  private final BallSim ball1;
  private final BallSim ball2;
  private final AtomicBoolean resetStateRequested = new AtomicBoolean(false);
  private final AtomicBoolean teleopRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoGrabRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoScoreRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoTwoBallRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoShootBallsRequested = new AtomicBoolean(false);
  private final AtomicBoolean disableRequested = new AtomicBoolean(false);
  private Runnable teleopActivator;

  private double autoTimerSeconds = 0.0;
  private boolean autoRunning = false;
  private boolean autoComplete = false;

  private volatile Snapshot snapshot;

  public SimWorld(
      Elevator elevator,
      Extender extender,
      Claw claw,
      Drive drive,
      Kicker kicker) {
    this.elevator = elevator;
    this.extender = extender;
    this.claw = claw;
    this.drive = drive;
    this.kicker = kicker;

    double tableFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TABLE_LEFT_X.in(Meters);
    double tableFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TABLE_RIGHT_X.in(Meters);
    double targetFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TARGET_LEFT_X.in(Meters);
    double targetFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TARGET_RIGHT_X.in(Meters);
    double tableTargetFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TABLE_TARGET_LEFT_X.in(Meters);
    double tableTargetFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TABLE_TARGET_RIGHT_X.in(Meters);
    double basketFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.BASKET_LEFT_X.in(Meters);
    double basketFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.BASKET_RIGHT_X.in(Meters);
    double basketHeight = first.robot.constants.FieldConstants.BASKET_HEIGHT.in(Meters);

    this.ball1 =
        new BallSim(
            first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
                + first.robot.constants.FieldConstants.BALL1_START_X.in(Meters),
            first.robot.constants.FieldConstants.BALL1_START_Y.in(Meters),
            BallSim.State.ON_TABLE,
            tableFieldLeft,
            tableFieldRight,
            first.robot.constants.FieldConstants.TABLE_HEIGHT.in(Meters),
            targetFieldLeft,
            targetFieldRight,
            0.0);
    this.ball1.setBasketBounds(basketFieldLeft, basketFieldRight, basketHeight);
    this.ball1.setFloorTargetBounds(targetFieldLeft, targetFieldRight);

    this.ball2 =
        new BallSim(
            first.robot.constants.FieldConstants.ROBOT_START_X.in(Meters)
                + first.robot.constants.FieldConstants.BALL2_START_X.in(Meters),
            first.robot.constants.FieldConstants.BALL2_START_Y.in(Meters),
            BallSim.State.ON_FLOOR,
            tableFieldLeft,
            tableFieldRight,
            first.robot.constants.FieldConstants.TABLE_HEIGHT.in(Meters),
            tableTargetFieldLeft,
            tableTargetFieldRight,
            first.robot.constants.FieldConstants.TABLE_HEIGHT.in(Meters));
    this.ball2.setBasketBounds(basketFieldLeft, basketFieldRight, basketHeight);
    this.ball2.setFloorTargetBounds(targetFieldLeft, targetFieldRight);

    snapshot = buildSnapshot(false);
  }

  public SimWorld(
      Elevator elevator,
      Extender extender,
      Claw claw,
      Drive drive) {
    this(elevator, extender, claw, drive, null);
  }

  public SimWorld(
      Elevator elevator,
      Extender extender,
      Claw claw) {
    this(elevator, extender, claw, null, null);
  }

  public void setTeleopActivator(Runnable teleopActivator) {
    this.teleopActivator = teleopActivator;
  }

  /** Steps the ball physics and enforces table collision limits. Call from {@code simulationPeriodic()}. */
  public void update() {
    if (resetStateRequested.getAndSet(false)) {
      ball1.reset();
      ball2.reset();
      autoTimerSeconds = 0.0;
      autoRunning = false;
      autoComplete = false;
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      SimMechanical.resetHardware();
      if (drive != null) {
        drive.resetPosition(DriveConstants.START_POSITION);
        drive.resetSetpoint();
      }
      if (kicker != null) {
        kicker.resetSetpoint();
      }
      elevator.resetSetpoint();
      extender.resetSetpoint();
      claw.resetSetpoint();
      if (RobotBase.isAutonomous()) {
        org.wpilib.simulation.DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.TELEOPERATED);
      }
      if (teleopActivator != null) {
        teleopActivator.run();
      }
    }

    if (disableRequested.getAndSet(false)) {
      autoRunning = false;
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      org.wpilib.simulation.DriverStationSim.setEnabled(false);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
    }

    if (teleopRequested.getAndSet(false)) {
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      long teleopId = 0;
      for (org.wpilib.hardware.hal.OpModeOption opt : org.wpilib.simulation.DriverStationSim.getOpModeOptions()) {
        if ("Teleop".equals(opt.name)) {
          teleopId = opt.id;
          break;
        }
      }
      org.wpilib.simulation.DriverStationSim.setDsAttached(true);
      org.wpilib.simulation.DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.TELEOPERATED);
      if (teleopId != 0) {
        org.wpilib.simulation.DriverStationSim.setOpMode(teleopId);
      }
      org.wpilib.simulation.DriverStationSim.setEnabled(true);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
      if (teleopActivator != null) {
        teleopActivator.run();
      }
    }

    if (autoGrabRequested.getAndSet(false)) {
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      long grabBallId = 0;
      for (org.wpilib.hardware.hal.OpModeOption opt : org.wpilib.simulation.DriverStationSim.getOpModeOptions()) {
        if ("Grab Ball".equals(opt.name)) {
          grabBallId = opt.id;
          break;
        }
      }
      org.wpilib.simulation.DriverStationSim.setDsAttached(true);
      org.wpilib.simulation.DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.AUTONOMOUS);
      if (grabBallId != 0) {
        org.wpilib.simulation.DriverStationSim.setOpMode(grabBallId);
      }
      org.wpilib.simulation.DriverStationSim.setEnabled(true);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
      org.wpilib.command3.Scheduler.getDefault()
          .schedule(first.robot.orchestration.BallCommands.grabBallFromTable(elevator, extender, claw));
    }

    if (autoScoreRequested.getAndSet(false)) {
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      long scoreBallId = 0;
      for (org.wpilib.hardware.hal.OpModeOption opt : org.wpilib.simulation.DriverStationSim.getOpModeOptions()) {
        if ("Score Ball".equals(opt.name)) {
          scoreBallId = opt.id;
          break;
        }
      }
      org.wpilib.simulation.DriverStationSim.setDsAttached(true);
      org.wpilib.simulation.DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.AUTONOMOUS);
      if (scoreBallId != 0) {
        org.wpilib.simulation.DriverStationSim.setOpMode(scoreBallId);
      }
      org.wpilib.simulation.DriverStationSim.setEnabled(true);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
      if (drive != null) {
        org.wpilib.command3.Scheduler.getDefault()
            .schedule(first.robot.auto.ScoreBallAuto.driveAndScoreBall(drive, elevator, extender, claw));
      } else {
        org.wpilib.command3.Scheduler.getDefault()
            .schedule(
                org.wpilib.command3.Command.noRequirements(coroutine -> {
                  coroutine.await(first.robot.orchestration.BallCommands.grabBallFromTable(elevator, extender, claw));
                  coroutine.await(first.robot.orchestration.BallCommands.scoreBallOnGround(elevator, extender, claw));
                }).named("Score Ball"));
      }
    }

    if (autoTwoBallRequested.getAndSet(false)) {
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      long twoBallId = 0;
      for (org.wpilib.hardware.hal.OpModeOption opt : org.wpilib.simulation.DriverStationSim.getOpModeOptions()) {
        if ("Two Ball Routine".equals(opt.name)) {
          twoBallId = opt.id;
          break;
        }
      }
      org.wpilib.simulation.DriverStationSim.setDsAttached(true);
      org.wpilib.simulation.DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.AUTONOMOUS);
      if (twoBallId != 0) {
        org.wpilib.simulation.DriverStationSim.setOpMode(twoBallId);
      }
      org.wpilib.simulation.DriverStationSim.setEnabled(true);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
      if (drive != null) {
        org.wpilib.command3.Scheduler.getDefault()
            .schedule(first.robot.auto.TwoBallAuto.twoBallRoutine(drive, elevator, extender, claw));
      }
    }

    if (autoShootBallsRequested.getAndSet(false)) {
      org.wpilib.command3.Scheduler.getDefault().cancelAll();
      long shootBallsId = 0;
      for (org.wpilib.hardware.hal.OpModeOption opt : org.wpilib.simulation.DriverStationSim.getOpModeOptions()) {
        if ("Shoot Both Balls".equals(opt.name)) {
          shootBallsId = opt.id;
          break;
        }
      }
      org.wpilib.simulation.DriverStationSim.setDsAttached(true);
      org.wpilib.simulation.DriverStationSim.setRobotMode(org.wpilib.hardware.hal.RobotMode.AUTONOMOUS);
      if (shootBallsId != 0) {
        org.wpilib.simulation.DriverStationSim.setOpMode(shootBallsId);
      }
      org.wpilib.simulation.DriverStationSim.setEnabled(true);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
      if (drive != null && kicker != null) {
        org.wpilib.command3.Scheduler.getDefault()
            .schedule(
                first.robot.auto.ShootBothBallsAuto.shootBothBallsRoutine(
                    drive, elevator, extender, claw, kicker));
      }
    }

    double robotFieldX =
        (drive != null) ? drive.getPosition().in(Meters) : first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters);
    double wristExtenderX = WRIST_BASE_X.in(Meters) + extender.getPosition().in(Meters);
    double wristFieldX = robotFieldX + wristExtenderX;
    double wristY = elevator.getPosition().in(Meters);
    double clawAngle = SimMechanical.getClawMotor().getSimAngle();
    boolean ballHeld = ball1.isHeld() || ball2.isHeld();

    // The lowest point of the claw fingers / held ball relative to wristY
    double clawDrop = 0.02 + first.robot.constants.RobotConstants.FINGER_LENGTH.in(Meters) * Math.cos(clawAngle);
    if (ballHeld) {
      clawDrop = Math.max(clawDrop, first.robot.constants.RobotConstants.GRIP_DEPTH.in(Meters) + first.robot.constants.FieldConstants.BALL_RADIUS.in(Meters));
    }
    double clawLowestY = wristY - clawDrop;

    // How far in front of wristSideX the fingers / held ball reach
    double clawFrontReach = 0.03 + first.robot.constants.RobotConstants.FINGER_LENGTH.in(Meters) * Math.sin(clawAngle);
    if (ballHeld) {
      clawFrontReach = Math.max(clawFrontReach, first.robot.constants.FieldConstants.BALL_RADIUS.in(Meters));
    }
    double tableFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TABLE_LEFT_X.in(Meters);
    double tableFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TABLE_RIGHT_X.in(Meters);

    boolean tableContact = false;

    // Check collision with the table in field coordinates:
    // Only applies when robot arm is approaching the table from the left
    if (wristFieldX < tableFieldLeft
        && clawLowestY < first.robot.constants.FieldConstants.TABLE_HEIGHT.in(Meters) - 0.015) {
      if (wristFieldX + clawFrontReach >= tableFieldLeft - 0.015) {
        // Claw tip reaches the table's left edge: stop extender and flag collision
        double maxAllowedWristFieldX = tableFieldLeft - clawFrontReach;
        double maxAllowedExtenderWrist = maxAllowedWristFieldX - robotFieldX;
        double maxExt = Math.max(0.0, maxAllowedExtenderWrist - WRIST_BASE_X.in(Meters));
        SimMechanical.getExtenderMotor().setMaxExtensionLimit(maxExt);
        tableContact = true;
      } else {
        SimMechanical.getExtenderMotor()
            .setMaxExtensionLimit(ExtenderConstants.MAX_EXTENSION.in(Meters));
      }
      SimMechanical.getElevatorMotor()
          .setMinHeightLimit(ElevatorConstants.MIN_HEIGHT.in(Meters));
    } else {
      // Claw is high enough to be over the table, or wrist has moved over the table
      boolean overTable = (wristFieldX + clawFrontReach) >= tableFieldLeft
          && (wristFieldX - 0.06) <= tableFieldRight;

      if (overTable) {
        // Over the table top: elevator cannot penetrate down into the table surface
        double minElevatorHeight = first.robot.constants.FieldConstants.TABLE_HEIGHT.in(Meters) + clawDrop;
        SimMechanical.getElevatorMotor().setMinHeightLimit(minElevatorHeight);
        if (clawLowestY <= first.robot.constants.FieldConstants.TABLE_HEIGHT.in(Meters) + 0.015) {
          tableContact = true;
        }
      } else {
        SimMechanical.getElevatorMotor()
            .setMinHeightLimit(ElevatorConstants.MIN_HEIGHT.in(Meters));
      }
      SimMechanical.getExtenderMotor()
          .setMaxExtensionLimit(ExtenderConstants.MAX_EXTENSION.in(Meters));
    }

    if (kicker != null) {
      double kickStrength = SimMechanical.getKickerMotor().consumeKickStrength();
      if (kickStrength > 0.0) {
        double ext = extender.getPosition().in(Meters);
        boolean armInStrikeZone =
            ext >= SimConstants.KICKER_MIN_STRIKE_EXTENSION
                && ext <= SimConstants.KICKER_MAX_STRIKE_EXTENSION
                && wristY >= SimConstants.KICKER_MIN_STRIKE_WRIST_Y
                && wristY <= SimConstants.KICKER_MAX_STRIKE_WRIST_Y;
        if (armInStrikeZone) {
          double launchVx = SimConstants.KICKER_LAUNCH_VX_MPS * kickStrength;
          double launchVy = SimConstants.KICKER_LAUNCH_VY_MPS * kickStrength;
          if (ball1.isHeld()) {
            ball1.kick(launchVx, launchVy);
          }
          if (ball2.isHeld()) {
            ball2.kick(launchVx, launchVy);
          }
        }
      }
    }

    double robotVelX = (drive != null) ? drive.getVelocity() : 0.0;
    ball1.checkBumperContact(robotFieldX, robotVelX);
    ball2.checkBumperContact(robotFieldX, robotVelX);

    double targetFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TARGET_LEFT_X.in(Meters);
    double targetFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters)
            + first.robot.constants.FieldConstants.TARGET_RIGHT_X.in(Meters);
    boolean ball1InFloorTarget =
        ball1.getState() == BallSim.State.ON_FLOOR
            && ball1.getX() >= targetFieldLeft
            && ball1.getX() <= targetFieldRight;
    boolean ball2InFloorTarget =
        ball2.getState() == BallSim.State.ON_FLOOR
            && ball2.getX() >= targetFieldLeft
            && ball2.getX() <= targetFieldRight;

    ball1.setTableOccupied(
        ball2.getState() == BallSim.State.ON_TABLE && !ball2.isScoredInBasket());
    ball2.setTableOccupied(
        ball1.getState() == BallSim.State.ON_TABLE && !ball1.isScoredInBasket());
    ball1.setFloorTargetOccupied(ball2InFloorTarget);
    ball2.setFloorTargetOccupied(ball1InFloorTarget);

    double gripStop1;
    double gripStop2;
    if (ball1.isHeld()) {
      gripStop1 = ball1.update(LOOP_PERIOD_SECONDS, wristFieldX, wristY, clawAngle);
      gripStop2 = ball2.update(LOOP_PERIOD_SECONDS, Double.NEGATIVE_INFINITY, wristY, clawAngle);
    } else if (ball2.isHeld()) {
      gripStop2 = ball2.update(LOOP_PERIOD_SECONDS, wristFieldX, wristY, clawAngle);
      gripStop1 = ball1.update(LOOP_PERIOD_SECONDS, Double.NEGATIVE_INFINITY, wristY, clawAngle);
    } else if (ball1.isScored() && !ball2.isScored()) {
      gripStop2 = ball2.update(LOOP_PERIOD_SECONDS, wristFieldX, wristY, clawAngle);
      gripStop1 =
          ball1.update(
              LOOP_PERIOD_SECONDS,
              ball2.isHeld() ? Double.NEGATIVE_INFINITY : wristFieldX,
              wristY,
              clawAngle);
    } else {
      gripStop1 = ball1.update(LOOP_PERIOD_SECONDS, wristFieldX, wristY, clawAngle);
      gripStop2 =
          ball2.update(
              LOOP_PERIOD_SECONDS,
              ball1.isHeld() ? Double.NEGATIVE_INFINITY : wristFieldX,
              wristY,
              clawAngle);
    }
    double gripStop = Double.isNaN(gripStop1) ? gripStop2 : gripStop1;

    SimMechanical.getClawMotor().setGripStop(gripStop);
    SimMechanical.getClawBallSensor().setValue(ball1.isHeld() || ball2.isHeld());

    boolean isAutonomous = RobotBase.isAutonomous() && RobotBase.isEnabled();
    if (isAutonomous) {
      if (!autoRunning && !autoComplete) {
        autoRunning = true;
        autoTimerSeconds = 0.0;
      }
    } else {
      autoRunning = false;
    }

    if (autoRunning) {
      if (ball1.isScored() && ball2.isScored()) {
        autoRunning = false;
        autoComplete = true;
      } else {
        autoTimerSeconds += LOOP_PERIOD_SECONDS;
      }
    }

    snapshot = buildSnapshot(tableContact);
  }

  /** Resets the balls, cancels commands, and resets mechanisms to start. */
  public void requestResetState() {
    resetStateRequested.set(true);
  }

  /** Same as requestResetState. */
  public void requestBallReset() {
    requestResetState();
  }

  /** Switches Driver Station to Teleop mode and enables the robot. */
  public void requestTeleop() {
    teleopRequested.set(true);
  }

  /** Switches Driver Station to Autonomous mode and runs the Grab Ball sequence. */
  public void requestAutoGrab() {
    autoGrabRequested.set(true);
  }

  /** Switches Driver Station to Autonomous mode and runs the Score Ball sequence. */
  public void requestAutoScore() {
    autoScoreRequested.set(true);
  }

  /** Switches Driver Station to Autonomous mode and runs the Two Ball Routine. */
  public void requestAutoTwoBall() {
    autoTwoBallRequested.set(true);
  }

  /** Switches Driver Station to Autonomous mode and runs the Shoot Both Balls routine. */
  public void requestAutoShootBalls() {
    autoShootBallsRequested.set(true);
  }

  /** Disables the robot and cancels all commands. */
  public void requestDisable() {
    disableRequested.set(true);
  }

  public Snapshot snapshot() {
    return snapshot;
  }

  private Snapshot buildSnapshot(boolean tableContact) {
    String mode;
    String opMode = RobotBase.getOpMode();
    if (opMode != null && !opMode.isEmpty()) {
      mode = opMode;
    } else if (RobotBase.isAutonomous()) {
      mode = "Autonomous";
    } else if (RobotBase.isTeleop()) {
      mode = "Teleop";
    } else {
      mode = "Disabled";
    }

    double robotFieldX =
        (drive != null) ? drive.getPosition().in(Meters) : first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters);
    double robotSideX = robotFieldX - first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters);

    double ball1SideX = ball1.getX() - first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters);
    double ball2SideX = ball2.getX() - first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters);

    double minElevatorLimit = SimMechanical.getElevatorMotor().getMinHeightLimit();
    double maxExtenderLimit = SimMechanical.getExtenderMotor().getMaxExtensionLimit();
    boolean anyBallHeld = (ball1 != null && ball1.isHeld()) || (ball2 != null && ball2.isHeld()) || claw.hasBall();

    MotorState elevatorState = computeElevatorMotorState(minElevatorLimit, tableContact);
    MotorState extenderState = computeExtenderMotorState(maxExtenderLimit, tableContact);
    MotorState clawState = computeClawMotorState(anyBallHeld);
    MotorState driveState = computeDriveMotorState();
    MotorState kickerState = computeKickerMotorState();

    boolean basketScored = ball1.isScoredInBasket() || ball2.isScoredInBasket();

    return new Snapshot(
        elevator.getPosition().in(Meters),
        elevator.getSetpoint().in(Meters),
        elevator.getAppliedVolts(),
        elevator.getCurrentCommandName(),
        extender.getPosition().in(Meters),
        extender.getSetpoint().in(Meters),
        extender.getAppliedVolts(),
        extender.getCurrentCommandName(),
        claw.getAngle().in(Radians),
        claw.getSetpoint().in(Radians),
        claw.getAppliedVolts(),
        claw.getCurrentCommandName(),
        ball1SideX,
        ball1.getY(),
        ball1.isHeld(),
        ball1.isScored(),
        tableContact,
        RobotBase.isEnabled(),
        mode,
        robotFieldX,
        drive != null ? drive.getSetpoint().in(Meters) : first.robot.constants.FieldConstants.ROBOT_TABLE_X.in(Meters),
        drive != null ? drive.getAppliedVolts() : 0.0,
        drive != null ? drive.getCurrentCommandName() : "(none)",
        robotSideX,
        ball2SideX,
        ball2.getY(),
        ball2.isHeld(),
        ball2.isScored(),
        ball1.getX(),
        ball2.getX(),
        autoTimerSeconds,
        autoRunning,
        autoComplete,
        elevatorState,
        extenderState,
        clawState,
        driveState,
        basketScored,
        kicker != null ? kicker.getPosition().in(Meters) : 0.0,
        kicker != null ? kicker.getSetpoint().in(Meters) : 0.0,
        kicker != null ? kicker.getAppliedVolts() : 0.0,
        kicker == null || kicker.isRatchetLocked(),
        kicker != null && kicker.isSolenoidReleased(),
        kicker != null && kicker.isLoaded(),
        kicker != null ? kicker.getCurrentCommandName() : "(none)",
        kickerState);
  }

  private MotorState computeElevatorMotorState(double minLimit, boolean tableContact) {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = elevator.getVelocity();
    double volts = elevator.getAppliedVolts();
    double pos = elevator.getPosition().in(Meters);

    // Stalled: voltage applied, but cannot move because at limit/stop/obstacle
    boolean atLowerLimit = pos <= (minLimit + 0.02) || tableContact;
    boolean pushingDown = (volts < ElevatorConstants.KG - 0.4);
    boolean atUpperLimit = pos >= (ElevatorConstants.MAX_HEIGHT.in(Meters) - 0.02);
    boolean pushingUp = (volts > ElevatorConstants.KG + 0.8);

    if (Math.abs(vel) < 0.03 && ((atLowerLimit && pushingDown) || (atUpperLimit && pushingUp))) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.015 && Math.abs(volts - ElevatorConstants.KG) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private MotorState computeExtenderMotorState(double maxLimit, boolean tableContact) {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = extender.getVelocity();
    double volts = extender.getAppliedVolts();
    double pos = extender.getPosition().in(Meters);

    // Stalled: voltage applied, but cannot move because at retracted or extended limit
    boolean atMinLimit = pos <= (ExtenderConstants.MIN_EXTENSION.in(Meters) + 0.02);
    boolean pushingIn = (volts < -0.5);
    boolean atMaxLimit = pos >= (maxLimit - 0.02) || (tableContact && volts > 0.5);
    boolean pushingOut = (volts > 0.5);

    if (Math.abs(vel) < 0.03 && ((atMinLimit && pushingIn) || (atMaxLimit && pushingOut))) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.01 && Math.abs(volts) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private MotorState computeClawMotorState(boolean ballHeld) {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = claw.getVelocity();
    double volts = claw.getAppliedVolts();
    double angle = claw.getAngle().in(Radians);

    // Stalled: squeezing a held ball, or driven into closed/open physical stops
    boolean grippingBall = ballHeld && (volts < -0.5);
    boolean atClosedStop = angle <= (ClawConstants.CLOSED_ANGLE.in(Radians) + 0.025) && (volts < -0.5);
    boolean atOpenStop = angle >= (ClawConstants.OPEN_ANGLE.in(Radians) - 0.025) && (volts > 0.5);

    if (Math.abs(vel) < 0.05 && (grippingBall || atClosedStop || atOpenStop)) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.02 && Math.abs(volts) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private MotorState computeDriveMotorState() {
    if (!RobotBase.isEnabled() || drive == null) {
      return MotorState.IDLE;
    }
    double vel = drive.getVelocity();
    double volts = drive.getAppliedVolts();

    // Drivetrain stalled: commanding substantial drive voltage while stationary
    if (Math.abs(volts) > 1.5 && Math.abs(vel) < 0.02) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.02 && Math.abs(volts) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private MotorState computeKickerMotorState() {
    if (!RobotBase.isEnabled() || kicker == null) {
      return MotorState.IDLE;
    }
    double vel = kicker.getVelocity();
    double volts = kicker.getAppliedVolts();
    double pos = kicker.getPosition().in(Meters);

    boolean atMaxPullback = pos >= (KickerConstants.MAX_PULLBACK.in(Meters) - 0.015) && (volts > 0.5);
    boolean againstRatchet = kicker.isRatchetLocked() && (volts < -0.5);

    if (Math.abs(vel) < 0.02 && (atMaxPullback || againstRatchet)) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.01 && Math.abs(volts) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }
}
