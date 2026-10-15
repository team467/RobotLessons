package first.robot.sim;

import static first.robot.constants.RobotConstants.WRIST_BASE_X;

import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.claw.ClawMechanism;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.DriveIOSim;
import first.robot.subsystems.drive.DriveMechanism;
import first.robot.subsystems.elevator.ElevatorMechanism;
import first.robot.subsystems.extender.ExtenderMechanism;
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
      MotorState driveMotorState) {}

  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final ElevatorMechanism elevator;
  private final first.robot.subsystems.elevator.ElevatorIOSim elevatorIO;
  private final ExtenderMechanism extender;
  private final first.robot.subsystems.extender.ExtenderIOSim extenderIO;
  private final ClawMechanism claw;
  private final ClawIOSim clawIO;
  private final DriveMechanism drive;
  private final DriveIOSim driveIO;
  private final BallSim ball1;
  private final BallSim ball2;
  private final AtomicBoolean resetStateRequested = new AtomicBoolean(false);
  private final AtomicBoolean teleopRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoGrabRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoScoreRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoTwoBallRequested = new AtomicBoolean(false);
  private final AtomicBoolean disableRequested = new AtomicBoolean(false);
  private Runnable teleopActivator;

  private double autoTimerSeconds = 0.0;
  private boolean autoRunning = false;
  private boolean autoComplete = false;

  private volatile Snapshot snapshot;

  public SimWorld(
      ElevatorMechanism elevator,
      first.robot.subsystems.elevator.ElevatorIOSim elevatorIO,
      ExtenderMechanism extender,
      first.robot.subsystems.extender.ExtenderIOSim extenderIO,
      ClawMechanism claw,
      ClawIOSim clawIO,
      DriveMechanism drive,
      DriveIOSim driveIO) {
    this.elevator = elevator;
    this.elevatorIO = elevatorIO;
    this.extender = extender;
    this.extenderIO = extenderIO;
    this.claw = claw;
    this.clawIO = clawIO;
    this.drive = drive;
    this.driveIO = driveIO;

    double tableFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TABLE_LEFT_X;
    double tableFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TABLE_RIGHT_X;
    double targetFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TARGET_LEFT_X;
    double targetFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TARGET_RIGHT_X;
    double tableTargetFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TABLE_TARGET_LEFT_X;
    double tableTargetFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TABLE_TARGET_RIGHT_X;

    this.ball1 =
        new BallSim(
            first.robot.constants.FieldConstants.ROBOT_TABLE_X
                + first.robot.constants.FieldConstants.BALL1_START_X,
            first.robot.constants.FieldConstants.BALL1_START_Y,
            BallSim.State.ON_TABLE,
            tableFieldLeft,
            tableFieldRight,
            first.robot.constants.FieldConstants.TABLE_HEIGHT,
            targetFieldLeft,
            targetFieldRight,
            0.0);

    this.ball2 =
        new BallSim(
            first.robot.constants.FieldConstants.ROBOT_START_X
                + first.robot.constants.FieldConstants.BALL2_START_X,
            first.robot.constants.FieldConstants.BALL2_START_Y,
            BallSim.State.ON_FLOOR,
            tableFieldLeft,
            tableFieldRight,
            first.robot.constants.FieldConstants.TABLE_HEIGHT,
            tableTargetFieldLeft,
            tableTargetFieldRight,
            first.robot.constants.FieldConstants.TABLE_HEIGHT);

    snapshot = buildSnapshot(false);
  }

  public SimWorld(
      ElevatorMechanism elevator,
      first.robot.subsystems.elevator.ElevatorIOSim elevatorIO,
      ExtenderMechanism extender,
      first.robot.subsystems.extender.ExtenderIOSim extenderIO,
      ClawMechanism claw,
      ClawIOSim clawIO) {
    this(elevator, elevatorIO, extender, extenderIO, claw, clawIO, null, null);
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
      org.wpilib.simulation.DriverStationSim.setEnabled(false);
      org.wpilib.simulation.DriverStationSim.notifyNewData();
      if (elevatorIO != null) {
        elevatorIO.resetSim();
      }
      if (extenderIO != null) {
        extenderIO.resetSim();
      }
      if (clawIO != null) {
        clawIO.resetSim();
      }
      if (driveIO != null) {
        driveIO.resetPosition();
      }
      if (drive != null) {
        drive.resetPosition(DriveConstants.START_POSITION);
        drive.resetSetpoint();
      }
      elevator.resetSetpoint();
      extender.resetSetpoint();
      claw.resetSetpoint();
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

    double robotFieldX =
        (drive != null) ? drive.getPosition() : first.robot.constants.FieldConstants.ROBOT_TABLE_X;
    double wristExtenderX = WRIST_BASE_X + extender.getPosition();
    double wristFieldX = robotFieldX + wristExtenderX;
    double wristY = elevator.getPosition();
    double clawAngle = clawIO.getSimAngle();
    boolean ballHeld = ball1.isHeld() || ball2.isHeld();

    // The lowest point of the claw fingers / held ball relative to wristY
    double clawDrop = 0.02 + first.robot.constants.RobotConstants.FINGER_LENGTH * Math.cos(clawAngle);
    if (ballHeld) {
      clawDrop = Math.max(clawDrop, first.robot.constants.RobotConstants.GRIP_DEPTH + first.robot.constants.FieldConstants.BALL_RADIUS);
    }
    double clawLowestY = wristY - clawDrop;

    // How far in front of wristSideX the fingers / held ball reach
    double clawFrontReach = 0.03 + first.robot.constants.RobotConstants.FINGER_LENGTH * Math.sin(clawAngle);
    if (ballHeld) {
      clawFrontReach = Math.max(clawFrontReach, first.robot.constants.FieldConstants.BALL_RADIUS);
    }
    double tableFieldLeft =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TABLE_LEFT_X;
    double tableFieldRight =
        first.robot.constants.FieldConstants.ROBOT_TABLE_X
            + first.robot.constants.FieldConstants.TABLE_RIGHT_X;

    boolean tableContact = false;

    // Check collision with the table in field coordinates:
    // Only applies when robot arm is approaching the table from the left
    if (wristFieldX < tableFieldLeft
        && clawLowestY < first.robot.constants.FieldConstants.TABLE_HEIGHT - 0.015) {
      if (wristFieldX + clawFrontReach >= tableFieldLeft - 0.015) {
        // Claw tip reaches the table's left edge: stop extender and flag collision
        double maxAllowedWristFieldX = tableFieldLeft - clawFrontReach;
        double maxAllowedExtenderWrist = maxAllowedWristFieldX - robotFieldX;
        double maxExt = Math.max(0.0, maxAllowedExtenderWrist - WRIST_BASE_X);
        if (extenderIO != null) {
          extenderIO.setMaxExtensionLimit(maxExt);
        }
        tableContact = true;
      } else {
        if (extenderIO != null) {
          extenderIO.setMaxExtensionLimit(first.robot.subsystems.extender.ExtenderConstants.MAX_EXTENSION);
        }
      }
      if (elevatorIO != null) {
        elevatorIO.setMinHeightLimit(first.robot.subsystems.elevator.ElevatorConstants.MIN_HEIGHT);
      }
    } else {
      // Claw is high enough to be over the table, or wrist has moved over the table
      boolean overTable = (wristFieldX + clawFrontReach) >= tableFieldLeft
          && (wristFieldX - 0.06) <= tableFieldRight;

      if (overTable) {
        // Over the table top: elevator cannot penetrate down into the table surface
        double minElevatorHeight = first.robot.constants.FieldConstants.TABLE_HEIGHT + clawDrop;
        if (elevatorIO != null) {
          elevatorIO.setMinHeightLimit(minElevatorHeight);
        }
        if (clawLowestY <= first.robot.constants.FieldConstants.TABLE_HEIGHT + 0.015) {
          tableContact = true;
        }
      } else {
        if (elevatorIO != null) {
          elevatorIO.setMinHeightLimit(first.robot.subsystems.elevator.ElevatorConstants.MIN_HEIGHT);
        }
      }
      if (extenderIO != null) {
        extenderIO.setMaxExtensionLimit(first.robot.subsystems.extender.ExtenderConstants.MAX_EXTENSION);
      }
    }

    double gripStop1 = ball1.update(LOOP_PERIOD_SECONDS, wristFieldX, wristY, clawAngle);
    double gripStop2 = ball2.update(LOOP_PERIOD_SECONDS, wristFieldX, wristY, clawAngle);
    double gripStop = Double.isNaN(gripStop1) ? gripStop2 : gripStop1;

    clawIO.setGripStop(gripStop);
    clawIO.setBallDetected(ball1.isHeld() || ball2.isHeld());

    // Step simulation physics for all mechanisms
    if (elevatorIO != null) {
      elevatorIO.updateSim();
    }
    if (extenderIO != null) {
      extenderIO.updateSim();
    }
    if (clawIO != null) {
      clawIO.updateSim();
    }
    if (driveIO != null) {
      driveIO.updateSim();
    }

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
        (drive != null) ? drive.getPosition() : first.robot.constants.FieldConstants.ROBOT_TABLE_X;
    double robotSideX = robotFieldX - first.robot.constants.FieldConstants.ROBOT_TABLE_X;

    double ball1SideX = ball1.getX() - first.robot.constants.FieldConstants.ROBOT_TABLE_X;
    double ball2SideX = ball2.getX() - first.robot.constants.FieldConstants.ROBOT_TABLE_X;

    double minElevatorLimit =
        (elevatorIO != null) ? elevatorIO.getMinHeightLimit() : first.robot.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;
    double maxExtenderLimit =
        (extenderIO != null) ? extenderIO.getMaxExtensionLimit() : first.robot.subsystems.extender.ExtenderConstants.MAX_EXTENSION;
    boolean anyBallHeld = (ball1 != null && ball1.isHeld()) || (ball2 != null && ball2.isHeld()) || claw.hasBall();

    MotorState elevatorState = computeElevatorMotorState(minElevatorLimit, tableContact);
    MotorState extenderState = computeExtenderMotorState(maxExtenderLimit, tableContact);
    MotorState clawState = computeClawMotorState(anyBallHeld);
    MotorState driveState = computeDriveMotorState();

    return new Snapshot(
        elevator.getPosition(),
        elevator.getSetpoint(),
        elevator.getAppliedVolts(),
        elevator.getCurrentCommandName(),
        extender.getPosition(),
        extender.getSetpoint(),
        extender.getAppliedVolts(),
        extender.getCurrentCommandName(),
        claw.getAngle(),
        claw.getSetpoint(),
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
        drive != null ? drive.getSetpoint() : first.robot.constants.FieldConstants.ROBOT_TABLE_X,
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
        driveState);
  }

  private MotorState computeElevatorMotorState(double minLimit, boolean tableContact) {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = elevator.getVelocity();
    double volts = elevator.getAppliedVolts();
    double pos = elevator.getPosition();

    // Stalled: voltage applied, but cannot move because at limit/stop/obstacle
    boolean atLowerLimit = pos <= (minLimit + 0.02) || tableContact;
    boolean pushingDown = (volts < first.robot.subsystems.elevator.ElevatorConstants.KG - 0.4);
    boolean atUpperLimit = pos >= (first.robot.subsystems.elevator.ElevatorConstants.MAX_HEIGHT - 0.02);
    boolean pushingUp = (volts > first.robot.subsystems.elevator.ElevatorConstants.KG + 0.8);

    if (Math.abs(vel) < 0.03 && ((atLowerLimit && pushingDown) || (atUpperLimit && pushingUp))) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.015 && Math.abs(volts - first.robot.subsystems.elevator.ElevatorConstants.KG) > 0.2) {
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
    double pos = extender.getPosition();

    // Stalled: voltage applied, but cannot move because at retracted or extended limit
    boolean atMinLimit = pos <= (first.robot.subsystems.extender.ExtenderConstants.MIN_EXTENSION + 0.02);
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
    double angle = claw.getAngle();

    // Stalled: squeezing a held ball, or driven into closed/open physical stops
    boolean grippingBall = ballHeld && (volts < -0.5);
    boolean atClosedStop = angle <= (first.robot.subsystems.claw.ClawConstants.CLOSED_ANGLE + 0.025) && (volts < -0.5);
    boolean atOpenStop = angle >= (first.robot.subsystems.claw.ClawConstants.OPEN_ANGLE - 0.025) && (volts > 0.5);

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
}
