package first.robot.sim;

import static first.robot.constants.RobotConstants.WRIST_BASE_X;
import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.simulation.DriverStationSim;

/**
 * Glues the simulated mechanisms to the ball physics, and publishes a snapshot of everything for
 * the {@link SimWindow} to draw.
 *
 * <p>{@link #update()} runs on the robot thread. The window reads {@link #snapshot()} on the
 * Swing thread. Snapshots are immutable, so this is thread-safe.
 */
public class SimWorld {
  // ---- Local Hardware Port Constants (kept local so sim classes do not reference Schematic) ----
  static final int DRIVE_CAN_ID = 1;
  static final int ELEVATOR_CAN_ID = 2;
  static final int EXTENDER_CAN_ID = 3;
  static final int CLAW_CAN_ID = 4;
  static final int KICKER_CAN_ID = 5;
  static final int CLAW_BALL_SENSOR_DIO = 0;
  static final int KICKER_SOLENOID_CHANNEL = 1;

  // ---- Local Mechanism Constants (kept local so learners can create subsystem constants) ----
  static final double DRIVE_START_POSITION_METERS = FieldConstants.ROBOT_START_X.in(Meters);
  static final double ELEVATOR_MIN_HEIGHT_METERS = 0.25;
  static final double ELEVATOR_MAX_HEIGHT_METERS = 1.20;
  static final double ELEVATOR_STARTING_HEIGHT_METERS = 0.50;
  static final double ELEVATOR_KG = 0.70;
  static final double EXTENDER_MIN_EXTENSION_METERS = 0.0;
  static final double EXTENDER_MAX_EXTENSION_METERS = 1.10;
  static final double EXTENDER_STARTING_EXTENSION_METERS = 0.33;
  static final double CLAW_CLOSED_ANGLE_RAD = 0.0;
  static final double CLAW_OPEN_ANGLE_RAD = Math.toRadians(60.0);
  static final double CLAW_STARTING_ANGLE_RAD = 0.0;
  static final double KICKER_MIN_PULLBACK_METERS = 0.0;
  static final double KICKER_MAX_PULLBACK_METERS = 0.25;
  static final double KICKER_LOADED_PULLBACK_METERS = 0.22;
  static final double KICKER_STARTING_PULLBACK_METERS = 0.0;
  static final double KICKER_TOLERANCE_METERS = 0.015;

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
      MotorState kickerMotorState) {}

  private final boolean driveInitialized;
  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;
  private final SparkMax extenderMotor;
  private final RelativeEncoder extenderEncoder;
  private final SparkMax clawMotor;
  private final RelativeEncoder clawEncoder;
  private final DigitalInput clawBallSensor;
  private final SparkMax kickerMotor;
  private final RelativeEncoder kickerEncoder;
  private final DigitalOutput kickerSolenoid;

  // Epilogue telemetry subscribers (populated via @Logged on Robot and subsystems)
  private final DoubleSubscriber elevatorSetpointSub;
  private final StringSubscriber elevatorCommandSub;
  private final DoubleSubscriber extenderSetpointSub;
  private final StringSubscriber extenderCommandSub;
  private final DoubleSubscriber clawSetpointSub;
  private final StringSubscriber clawCommandSub;
  private final DoubleSubscriber driveSetpointSub;
  private final StringSubscriber driveCommandSub;
  private final DoubleSubscriber kickerSetpointSub;
  private final StringSubscriber kickerCommandSub;
  private final BooleanSubscriber kickerRatchetLockedSub;
  private final BooleanSubscriber kickerLoadedSub;

  private final BallSim ball1;
  private final BallSim ball2;
  private final AtomicBoolean resetStateRequested = new AtomicBoolean(false);
  private final AtomicBoolean teleopRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoScoreRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoTwoBallRequested = new AtomicBoolean(false);
  private final AtomicBoolean autoShootBallsRequested = new AtomicBoolean(false);
  private final AtomicBoolean disableRequested = new AtomicBoolean(false);
  private Runnable teleopActivator;

  private double autoTimerSeconds = 0.0;
  private boolean autoRunning = false;
  private boolean autoComplete = false;

  private volatile Snapshot snapshot;

  public SimWorld() {
    this.driveInitialized = SimMechanical.isMotorInitialized(DRIVE_CAN_ID);
    this.driveMotor = SimMechanical.createSparkMax(DRIVE_CAN_ID, MotorType.kBrushless);
    this.driveEncoder = driveMotor.getEncoder();
    if (!driveInitialized) {
      SimMechanical.setPosition(DRIVE_CAN_ID, FieldConstants.ROBOT_TABLE_X.in(Meters));
    }

    this.elevatorMotor = SimMechanical.createSparkMax(ELEVATOR_CAN_ID, MotorType.kBrushless);
    this.elevatorEncoder = elevatorMotor.getEncoder();

    this.extenderMotor = SimMechanical.createSparkMax(EXTENDER_CAN_ID, MotorType.kBrushless);
    this.extenderEncoder = extenderMotor.getEncoder();

    this.clawMotor = SimMechanical.createSparkMax(CLAW_CAN_ID, MotorType.kBrushless);
    this.clawEncoder = clawMotor.getEncoder();
    this.clawBallSensor = SimMechanical.createDigitalInput(CLAW_BALL_SENSOR_DIO);

    this.kickerMotor = SimMechanical.createSparkMax(KICKER_CAN_ID, MotorType.kBrushless);
    this.kickerEncoder = kickerMotor.getEncoder();
    this.kickerSolenoid = SimMechanical.createDigitalOutput(KICKER_SOLENOID_CHANNEL);

    NetworkTable robotTelemetry =
        NetworkTableInstance.getDefault().getTable("Telemetry").getSubTable("Robot");
    this.elevatorSetpointSub =
        robotTelemetry
            .getSubTable("elevator")
            .getDoubleTopic("getSetpoint")
            .subscribe(ELEVATOR_STARTING_HEIGHT_METERS);
    this.elevatorCommandSub =
        robotTelemetry
            .getSubTable("elevator")
            .getStringTopic("getCurrentCommandName")
            .subscribe("(none)");
    this.extenderSetpointSub =
        robotTelemetry
            .getSubTable("extender")
            .getDoubleTopic("getSetpoint")
            .subscribe(EXTENDER_STARTING_EXTENSION_METERS);
    this.extenderCommandSub =
        robotTelemetry
            .getSubTable("extender")
            .getStringTopic("getCurrentCommandName")
            .subscribe("(none)");
    this.clawSetpointSub =
        robotTelemetry
            .getSubTable("claw")
            .getDoubleTopic("getSetpoint")
            .subscribe(CLAW_STARTING_ANGLE_RAD);
    this.clawCommandSub =
        robotTelemetry
            .getSubTable("claw")
            .getStringTopic("getCurrentCommandName")
            .subscribe("(none)");
    this.driveSetpointSub =
        robotTelemetry
            .getSubTable("drive")
            .getDoubleTopic("getSetpoint")
            .subscribe(DRIVE_START_POSITION_METERS);
    this.driveCommandSub =
        robotTelemetry
            .getSubTable("drive")
            .getStringTopic("getCurrentCommandName")
            .subscribe("(none)");
    this.kickerSetpointSub =
        robotTelemetry
            .getSubTable("kicker")
            .getDoubleTopic("getSetpoint")
            .subscribe(KICKER_STARTING_PULLBACK_METERS);
    this.kickerCommandSub =
        robotTelemetry
            .getSubTable("kicker")
            .getStringTopic("getCurrentCommandName")
            .subscribe("(none)");
    this.kickerRatchetLockedSub =
        robotTelemetry
            .getSubTable("kicker")
            .getBooleanTopic("isRatchetLocked")
            .subscribe(true);
    this.kickerLoadedSub =
        robotTelemetry
            .getSubTable("kicker")
            .getBooleanTopic("isLoaded")
            .subscribe(false);

    double tableFieldLeft =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TABLE_LEFT_X.in(Meters);
    double tableFieldRight =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TABLE_RIGHT_X.in(Meters);
    double targetFieldLeft =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TARGET_LEFT_X.in(Meters);
    double targetFieldRight =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TARGET_RIGHT_X.in(Meters);
    double tableTargetFieldLeft =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TABLE_TARGET_LEFT_X.in(Meters);
    double tableTargetFieldRight =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TABLE_TARGET_RIGHT_X.in(Meters);
    double basketFieldLeft =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.BASKET_LEFT_X.in(Meters);
    double basketFieldRight =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.BASKET_RIGHT_X.in(Meters);
    double basketHeight = FieldConstants.BASKET_HEIGHT.in(Meters);

    this.ball1 =
        new BallSim(
            FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.BALL1_START_X.in(Meters),
            FieldConstants.BALL1_START_Y.in(Meters),
            BallSim.State.ON_TABLE,
            tableFieldLeft,
            tableFieldRight,
            FieldConstants.TABLE_HEIGHT.in(Meters),
            targetFieldLeft,
            targetFieldRight,
            0.0);
    this.ball1.setBasketBounds(basketFieldLeft, basketFieldRight, basketHeight);
    this.ball1.setFloorTargetBounds(targetFieldLeft, targetFieldRight);

    this.ball2 =
        new BallSim(
            FieldConstants.ROBOT_START_X.in(Meters) + FieldConstants.BALL2_START_X.in(Meters),
            FieldConstants.BALL2_START_Y.in(Meters),
            BallSim.State.ON_FLOOR,
            tableFieldLeft,
            tableFieldRight,
            FieldConstants.TABLE_HEIGHT.in(Meters),
            tableTargetFieldLeft,
            tableTargetFieldRight,
            FieldConstants.TABLE_HEIGHT.in(Meters));
    this.ball2.setBasketBounds(basketFieldLeft, basketFieldRight, basketHeight);
    this.ball2.setFloorTargetBounds(targetFieldLeft, targetFieldRight);

    snapshot = buildSnapshot(false);
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
      Scheduler.getDefault().cancelAll();
      SimMechanical.resetHardware();
      if (!driveInitialized) {
        SimMechanical.setPosition(DRIVE_CAN_ID, FieldConstants.ROBOT_TABLE_X.in(Meters));
      }
      if (RobotBase.isAutonomous()) {
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
      }
      if (teleopActivator != null) {
        teleopActivator.run();
      }
    }

    if (disableRequested.getAndSet(false)) {
      autoRunning = false;
      Scheduler.getDefault().cancelAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }

    if (teleopRequested.getAndSet(false)) {
      Scheduler.getDefault().cancelAll();
      long teleopId = 0;
      for (OpModeOption opt : DriverStationSim.getOpModeOptions()) {
        if ("Teleop".equals(opt.name)) {
          teleopId = opt.id;
          break;
        }
      }
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
      if (teleopId != 0) {
        DriverStationSim.setOpMode(teleopId);
      }
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      if (teleopActivator != null) {
        teleopActivator.run();
      }
    }

    if (autoScoreRequested.getAndSet(false)) {
      Scheduler.getDefault().cancelAll();
      long scoreBallId = 0;
      for (OpModeOption opt : DriverStationSim.getOpModeOptions()) {
        if ("Score Ball".equals(opt.name)) {
          scoreBallId = opt.id;
          break;
        }
      }
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
      if (scoreBallId != 0) {
        DriverStationSim.setOpMode(scoreBallId);
      }
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }

    if (autoTwoBallRequested.getAndSet(false)) {
      Scheduler.getDefault().cancelAll();
      long twoBallId = 0;
      for (OpModeOption opt : DriverStationSim.getOpModeOptions()) {
        if ("Two Ball Routine".equals(opt.name)) {
          twoBallId = opt.id;
          break;
        }
      }
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
      if (twoBallId != 0) {
        DriverStationSim.setOpMode(twoBallId);
      }
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }

    if (autoShootBallsRequested.getAndSet(false)) {
      Scheduler.getDefault().cancelAll();
      long shootBallsId = 0;
      for (OpModeOption opt : DriverStationSim.getOpModeOptions()) {
        if ("Shoot Both Balls".equals(opt.name)) {
          shootBallsId = opt.id;
          break;
        }
      }
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
      if (shootBallsId != 0) {
        DriverStationSim.setOpMode(shootBallsId);
      }
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }

    double robotFieldX = getEncoderPosition(driveEncoder, DRIVE_START_POSITION_METERS);
    double extenderPos = getEncoderPosition(extenderEncoder, EXTENDER_STARTING_EXTENSION_METERS);
    double wristExtenderX = WRIST_BASE_X.in(Meters) + extenderPos;
    double wristFieldX = robotFieldX + wristExtenderX;
    double wristY = getEncoderPosition(elevatorEncoder, ELEVATOR_STARTING_HEIGHT_METERS);
    double clawAngle = SimMechanical.getClawMotor().getSimAngle();
    boolean ballHeld = ball1.isHeld() || ball2.isHeld();

    // The lowest point of the claw fingers / held ball relative to wristY
    double clawDrop = 0.02 + RobotConstants.FINGER_LENGTH.in(Meters) * Math.cos(clawAngle);
    if (ballHeld) {
      clawDrop =
          Math.max(
              clawDrop,
              RobotConstants.GRIP_DEPTH.in(Meters) + FieldConstants.BALL_RADIUS.in(Meters));
    }
    double clawLowestY = wristY - clawDrop;

    // The highest point of the wrist housing / claw motor relative to wristY
    double clawTopRise = 0.13;
    double clawHighestY = wristY + clawTopRise;

    // How far in front of wristSideX the fingers / held ball reach
    double clawFrontReach = 0.03 + RobotConstants.FINGER_LENGTH.in(Meters) * Math.sin(clawAngle);
    if (ballHeld) {
      clawFrontReach = Math.max(clawFrontReach, FieldConstants.BALL_RADIUS.in(Meters));
    }
    double tableFieldLeft =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TABLE_LEFT_X.in(Meters);
    double tableFieldRight =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TABLE_RIGHT_X.in(Meters);
    double tableTopY = FieldConstants.TABLE_HEIGHT.in(Meters);
    double tableBottomY = tableTopY - 0.045; // tabletop slab thickness (4.5 cm), legs do not collide

    boolean tableContact = false;

    // Check collision with the tabletop slab in field coordinates (legs do not collide):
    // 1. Approaching the tabletop slab from the left while vertically overlapping [tableBottomY, tableTopY]
    boolean verticallyOverlapsTabletop =
        clawLowestY < (tableTopY - 0.015) && clawHighestY > (tableBottomY + 0.015);
    if (wristFieldX < tableFieldLeft && verticallyOverlapsTabletop) {
      if (wristFieldX + clawFrontReach >= tableFieldLeft - 0.015) {
        // Claw reaches the tabletop's left edge: stop extender and flag collision
        double maxAllowedWristFieldX = tableFieldLeft - clawFrontReach;
        double maxAllowedExtenderWrist = maxAllowedWristFieldX - robotFieldX;
        double maxExt = Math.max(0.0, maxAllowedExtenderWrist - WRIST_BASE_X.in(Meters));
        SimMechanical.getExtenderMotor().setMaxExtensionLimit(maxExt);
        tableContact = true;
      } else {
        SimMechanical.getExtenderMotor().setMaxExtensionLimit(EXTENDER_MAX_EXTENSION_METERS);
      }
      SimMechanical.getElevatorMotor().setMinHeightLimit(ELEVATOR_MIN_HEIGHT_METERS);
      SimMechanical.getElevatorMotor().setMaxHeightLimit(ELEVATOR_MAX_HEIGHT_METERS);
    } else {
      // Arm is either above the tabletop, below the tabletop (under the table), or horizontally aligned with it
      boolean horizontallyAtTable =
          (wristFieldX + clawFrontReach) >= tableFieldLeft && (wristFieldX - 0.06) <= tableFieldRight;

      if (horizontallyAtTable) {
        if (wristY >= tableBottomY) {
          // Above the tabletop: elevator cannot penetrate down through the top surface
          double minElevatorHeight = tableTopY + clawDrop;
          SimMechanical.getElevatorMotor().setMinHeightLimit(minElevatorHeight);
          SimMechanical.getElevatorMotor().setMaxHeightLimit(ELEVATOR_MAX_HEIGHT_METERS);
          if (clawLowestY <= tableTopY + 0.015) {
            tableContact = true;
          }
        } else {
          // Under the tabletop: elevator cannot penetrate up through the underside of the tabletop
          double maxElevatorHeight = tableBottomY - clawTopRise;
          SimMechanical.getElevatorMotor().setMinHeightLimit(ELEVATOR_MIN_HEIGHT_METERS);
          SimMechanical.getElevatorMotor().setMaxHeightLimit(maxElevatorHeight);
          if (clawHighestY >= tableBottomY - 0.015) {
            tableContact = true;
          }
        }
      } else {
        SimMechanical.getElevatorMotor().setMinHeightLimit(ELEVATOR_MIN_HEIGHT_METERS);
        SimMechanical.getElevatorMotor().setMaxHeightLimit(ELEVATOR_MAX_HEIGHT_METERS);
      }
      SimMechanical.getExtenderMotor().setMaxExtensionLimit(EXTENDER_MAX_EXTENSION_METERS);
    }

    double kickStrength = SimMechanical.getKickerMotor().consumeKickStrength();
    if (kickStrength > 0.0) {
      boolean armInStrikeZone =
          extenderPos >= SimConstants.KICKER_MIN_STRIKE_EXTENSION
              && extenderPos <= SimConstants.KICKER_MAX_STRIKE_EXTENSION
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

    double robotVelX = getEncoderVelocity(driveEncoder);
    ball1.checkBumperContact(robotFieldX, robotVelX);
    ball2.checkBumperContact(robotFieldX, robotVelX);

    double targetFieldLeft =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TARGET_LEFT_X.in(Meters);
    double targetFieldRight =
        FieldConstants.ROBOT_TABLE_X.in(Meters) + FieldConstants.TARGET_RIGHT_X.in(Meters);
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

  /** Switches Driver Station to Teleop mode and enables the robot. */
  public void requestTeleop() {
    teleopRequested.set(true);
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
    NetworkTableInstance.getDefault().flushLocal();

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

    double robotFieldX = getEncoderPosition(driveEncoder, DRIVE_START_POSITION_METERS);
    double robotSideX = robotFieldX - FieldConstants.ROBOT_TABLE_X.in(Meters);

    double ball1SideX = ball1.getX() - FieldConstants.ROBOT_TABLE_X.in(Meters);
    double ball2SideX = ball2.getX() - FieldConstants.ROBOT_TABLE_X.in(Meters);

    double minElevatorLimit = SimMechanical.getElevatorMotor().getMinHeightLimit();
    double maxElevatorLimit = SimMechanical.getElevatorMotor().getMaxHeightLimit();
    double maxExtenderLimit = SimMechanical.getExtenderMotor().getMaxExtensionLimit();
    boolean anyBallHeld =
        (ball1 != null && ball1.isHeld())
            || (ball2 != null && ball2.isHeld())
            || clawBallSensor.get();

    MotorState elevatorState =
        computeElevatorMotorState(minElevatorLimit, maxElevatorLimit, tableContact);
    MotorState extenderState = computeExtenderMotorState(maxExtenderLimit, tableContact);
    MotorState clawState = computeClawMotorState(anyBallHeld);
    MotorState driveState = computeDriveMotorState();
    MotorState kickerState = computeKickerMotorState();

    boolean basketScored = ball1.isScoredInBasket() || ball2.isScoredInBasket();

    return new Snapshot(
        getEncoderPosition(elevatorEncoder, ELEVATOR_STARTING_HEIGHT_METERS),
        elevatorSetpointSub.get(),
        getAppliedVolts(elevatorMotor),
        elevatorCommandSub.get(),
        getEncoderPosition(extenderEncoder, EXTENDER_STARTING_EXTENSION_METERS),
        extenderSetpointSub.get(),
        getAppliedVolts(extenderMotor),
        extenderCommandSub.get(),
        getEncoderPosition(clawEncoder, CLAW_STARTING_ANGLE_RAD),
        clawSetpointSub.get(),
        getAppliedVolts(clawMotor),
        clawCommandSub.get(),
        ball1SideX,
        ball1.getY(),
        ball1.isHeld(),
        ball1.isScored(),
        tableContact,
        RobotBase.isEnabled(),
        mode,
        robotFieldX,
        driveSetpointSub.get(),
        getAppliedVolts(driveMotor),
        driveCommandSub.get(),
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
        getEncoderPosition(kickerEncoder, KICKER_STARTING_PULLBACK_METERS),
        kickerSetpointSub.get(),
        getAppliedVolts(kickerMotor),
        kickerRatchetLockedSub.get(),
        kickerSolenoid.get(),
        kickerLoadedSub.get(),
        kickerCommandSub.get(),
        kickerState);
  }

  private MotorState computeElevatorMotorState(
      double minLimit, double maxLimit, boolean tableContact) {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = getEncoderVelocity(elevatorEncoder);
    double volts = getAppliedVolts(elevatorMotor);
    double pos = getEncoderPosition(elevatorEncoder, ELEVATOR_STARTING_HEIGHT_METERS);

    // Stalled: voltage applied, but cannot move because at limit/stop/obstacle
    boolean atLowerLimit =
        pos <= (minLimit + 0.02) || (tableContact && volts < ELEVATOR_KG - 0.4);
    boolean pushingDown = (volts < ELEVATOR_KG - 0.4);
    boolean atUpperLimit =
        pos >= (maxLimit - 0.02) || (tableContact && volts > ELEVATOR_KG + 0.8);
    boolean pushingUp = (volts > ELEVATOR_KG + 0.8);

    if (Math.abs(vel) < 0.03 && ((atLowerLimit && pushingDown) || (atUpperLimit && pushingUp))) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.015 && Math.abs(volts - ELEVATOR_KG) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private MotorState computeExtenderMotorState(double maxLimit, boolean tableContact) {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = getEncoderVelocity(extenderEncoder);
    double volts = getAppliedVolts(extenderMotor);
    double pos = getEncoderPosition(extenderEncoder, EXTENDER_STARTING_EXTENSION_METERS);

    // Stalled: voltage applied, but cannot move because at retracted or extended limit
    boolean atMinLimit = pos <= (EXTENDER_MIN_EXTENSION_METERS + 0.02);
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
    double vel = getEncoderVelocity(clawEncoder);
    double volts = getAppliedVolts(clawMotor);
    double angle = getEncoderPosition(clawEncoder, CLAW_STARTING_ANGLE_RAD);

    // Stalled: squeezing a held ball, or driven into closed/open physical stops
    boolean grippingBall = ballHeld && (volts < -0.5);
    boolean atClosedStop = angle <= (CLAW_CLOSED_ANGLE_RAD + 0.025) && (volts < -0.5);
    boolean atOpenStop = angle >= (CLAW_OPEN_ANGLE_RAD - 0.025) && (volts > 0.5);

    if (Math.abs(vel) < 0.05 && (grippingBall || atClosedStop || atOpenStop)) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.02 && Math.abs(volts) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private MotorState computeDriveMotorState() {
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = getEncoderVelocity(driveEncoder);
    double volts = getAppliedVolts(driveMotor);

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
    if (!RobotBase.isEnabled()) {
      return MotorState.IDLE;
    }
    double vel = getEncoderVelocity(kickerEncoder);
    double volts = getAppliedVolts(kickerMotor);
    double pos = getEncoderPosition(kickerEncoder, KICKER_STARTING_PULLBACK_METERS);

    boolean atMaxPullback = pos >= (KICKER_MAX_PULLBACK_METERS - 0.015) && (volts > 0.5);
    boolean ratchetLocked = !kickerSolenoid.get() && pos > KICKER_MIN_PULLBACK_METERS + 1e-3;
    boolean againstRatchet = ratchetLocked && (volts < -0.5);

    if (Math.abs(vel) < 0.02 && (atMaxPullback || againstRatchet)) {
      return MotorState.STALLED;
    }

    if (Math.abs(vel) > 0.01 && Math.abs(volts) > 0.2) {
      return MotorState.RUNNING;
    }

    return MotorState.IDLE;
  }

  private static double getEncoderPosition(RelativeEncoder encoder, double defaultValue) {
    Double pos = encoder.getPosition().get();
    return pos != null ? pos : defaultValue;
  }

  private static double getEncoderVelocity(RelativeEncoder encoder) {
    Double vel = encoder.getVelocity().get();
    return vel != null ? vel : 0.0;
  }

  private static double getAppliedVolts(SparkMax motor) {
    if (!RobotBase.isEnabled()) {
      return 0.0;
    }
    Double val = motor.getAppliedOutput().get();
    return val != null ? val * 12.0 : 0.0;
  }
}
