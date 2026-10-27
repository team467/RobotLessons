package first.robot.sim;

import static first.robot.Schematic.*;
import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.constants.FieldConstants;
import first.robot.mechanisms.claw.ClawConstants;
import first.robot.mechanisms.drive.DriveConstants;
import first.robot.mechanisms.elevator.ElevatorConstants;
import first.robot.mechanisms.extender.ExtenderConstants;
import first.robot.mechanisms.shooter.ShooterConstants;
import java.util.HashMap;
import java.util.Map;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.hardware.discrete.DigitalOutput;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.DIOSim;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Central mechanical physics simulation, SparkMaxSim registry, and DIO simulation registry
 * for the robot.
 *
 * <p>Instantiates WPILib physics models ({@link ElevatorSim}, {@link SingleJointedArmSim})
 * and connects them to REV {@link SparkMaxSim} and WPILib {@link DIOSim} instances so that
 * mechanisms can instantiate real {@link SparkMax}, {@link DigitalInput}, and {@link DigitalOutput}
 * hardware objects directly.
 */
public final class SimMechanical {

  /** Drivetrain motor entry (CAN ID 1). */
  public static final class DriveMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(2),
            SimConstants.DRIVE_GEARING,
            SimConstants.DRIVE_ROBOT_MASS_KG,
            SimConstants.DRIVE_WHEEL_RADIUS_METERS,
            -1.0,
            FieldConstants.FIELD_LENGTH_METERS + 1.0,
            false,
            DriveConstants.START_POSITION);
    private SparkMax motor;
    private SparkMaxSim sparkSim;

    public void attachSparkMax(SparkMax sparkMax) {
      if (this.motor == sparkMax && this.sparkSim != null) {
        return;
      }
      this.motor = sparkMax;
      this.sparkSim = new SparkMaxSim(sparkMax, DCMotor.getNEO(2));
      this.sparkSim.setBusVoltage(12.0);
      this.sparkSim.enable();
      this.sparkSim.setPosition(sim.getPosition());
      this.sparkSim.setVelocity(sim.getVelocity());
      this.sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
      this.sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
    }

    public SparkMax getSparkMax() {
      return motor;
    }

    public SparkMaxSim getSparkMaxSim() {
      return sparkSim;
    }

    public double getPosition() {
      return sim.getPosition();
    }

    public double getVelocity() {
      return sim.getVelocity();
    }

    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    public double getAppliedVolts() {
      if (!RobotBase.isEnabled()) {
        return 0.0;
      }
      return sparkSim != null ? sparkSim.getSetpoint() : 0.0;
    }

    public void setPosition(double position) {
      sim.setState(position, 0.0);
      if (sparkSim != null) {
        sparkSim.setPosition(position);
        sparkSim.setVelocity(0.0);
        sparkSim.getRelativeEncoderSim().setPosition(position);
        sparkSim.getRelativeEncoderSim().setVelocity(0.0);
      }
    }

    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (sparkSim != null) {
        sparkSim.setBusVoltage(12.0);
        sparkSim.setPosition(sim.getPosition());
        sparkSim.setVelocity(sim.getVelocity());
        sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
        sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
        sparkSim.setMotorCurrent(Math.abs(sim.getCurrentDraw()));
        sparkSim.setAppliedOutput(volts / 12.0);
      }
    }

    public void reset() {
      setPosition(DriveConstants.START_POSITION);
      if (sparkSim != null) {
        sparkSim.setAppliedOutput(0.0);
      }
      if (motor != null) {
        motor.setVoltage(0.0);
      }
    }
  }

  /** Elevator motor entry (CAN ID 2). */
  public static final class ElevatorMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            SimConstants.ELEVATOR_GEARING,
            SimConstants.ELEVATOR_CARRIAGE_MASS_KG,
            SimConstants.ELEVATOR_DRUM_RADIUS_METERS,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            true, // simulate gravity
            ElevatorConstants.STARTING_HEIGHT);
    private SparkMax motor;
    private SparkMaxSim sparkSim;
    private double minHeightLimit = ElevatorConstants.MIN_HEIGHT;

    public void attachSparkMax(SparkMax sparkMax) {
      if (this.motor == sparkMax && this.sparkSim != null) {
        return;
      }
      this.motor = sparkMax;
      this.sparkSim = new SparkMaxSim(sparkMax, DCMotor.getNEO(1));
      this.sparkSim.setBusVoltage(12.0);
      this.sparkSim.enable();
      this.sparkSim.setPosition(sim.getPosition());
      this.sparkSim.setVelocity(sim.getVelocity());
      this.sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
      this.sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
    }

    public SparkMax getSparkMax() {
      return motor;
    }

    public SparkMaxSim getSparkMaxSim() {
      return sparkSim;
    }

    public void setMinHeightLimit(double minHeight) {
      this.minHeightLimit = minHeight;
    }

    public double getMinHeightLimit() {
      return minHeightLimit;
    }

    public double getPosition() {
      return sim.getPosition();
    }

    public double getVelocity() {
      return sim.getVelocity();
    }

    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    public double getAppliedVolts() {
      if (!RobotBase.isEnabled()) {
        return 0.0;
      }
      return sparkSim != null ? sparkSim.getSetpoint() : 0.0;
    }

    public void setPosition(double position) {
      sim.setState(position, 0.0);
      if (sparkSim != null) {
        sparkSim.setPosition(position);
        sparkSim.setVelocity(0.0);
        sparkSim.getRelativeEncoderSim().setPosition(position);
        sparkSim.getRelativeEncoderSim().setVelocity(0.0);
      }
    }

    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (sim.getPosition() < minHeightLimit) {
        sim.setState(minHeightLimit, Math.max(0.0, sim.getVelocity()));
      }
      if (sparkSim != null) {
        sparkSim.setBusVoltage(12.0);
        sparkSim.setPosition(sim.getPosition());
        sparkSim.setVelocity(sim.getVelocity());
        sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
        sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
        sparkSim.setMotorCurrent(Math.abs(sim.getCurrentDraw()));
        sparkSim.setAppliedOutput(volts / 12.0);
      }
    }

    public void reset() {
      minHeightLimit = ElevatorConstants.MIN_HEIGHT;
      setPosition(ElevatorConstants.STARTING_HEIGHT);
      if (sparkSim != null) {
        sparkSim.setAppliedOutput(0.0);
      }
      if (motor != null) {
        motor.setVoltage(0.0);
      }
    }
  }

  /** Extender motor entry (CAN ID 3). */
  public static final class ExtenderMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            SimConstants.EXTENDER_GEARING,
            SimConstants.EXTENDER_SLIDE_MASS_KG,
            SimConstants.EXTENDER_PULLEY_RADIUS_METERS,
            ExtenderConstants.MIN_EXTENSION,
            ExtenderConstants.MAX_EXTENSION,
            false, // horizontal: no gravity
            ExtenderConstants.STARTING_EXTENSION);
    private SparkMax motor;
    private SparkMaxSim sparkSim;
    private double maxExtensionLimit = ExtenderConstants.MAX_EXTENSION;

    public void attachSparkMax(SparkMax sparkMax) {
      if (this.motor == sparkMax && this.sparkSim != null) {
        return;
      }
      this.motor = sparkMax;
      this.sparkSim = new SparkMaxSim(sparkMax, DCMotor.getNEO(1));
      this.sparkSim.setBusVoltage(12.0);
      this.sparkSim.enable();
      this.sparkSim.setPosition(sim.getPosition());
      this.sparkSim.setVelocity(sim.getVelocity());
      this.sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
      this.sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
    }

    public SparkMax getSparkMax() {
      return motor;
    }

    public SparkMaxSim getSparkMaxSim() {
      return sparkSim;
    }

    public void setMaxExtensionLimit(double maxExtension) {
      this.maxExtensionLimit = maxExtension;
    }

    public double getMaxExtensionLimit() {
      return maxExtensionLimit;
    }

    public double getPosition() {
      return sim.getPosition();
    }

    public double getVelocity() {
      return sim.getVelocity();
    }

    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    public double getAppliedVolts() {
      if (!RobotBase.isEnabled()) {
        return 0.0;
      }
      return sparkSim != null ? sparkSim.getSetpoint() : 0.0;
    }

    public void setPosition(double position) {
      sim.setState(position, 0.0);
      if (sparkSim != null) {
        sparkSim.setPosition(position);
        sparkSim.setVelocity(0.0);
        sparkSim.getRelativeEncoderSim().setPosition(position);
        sparkSim.getRelativeEncoderSim().setVelocity(0.0);
      }
    }

    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (sim.getPosition() > maxExtensionLimit) {
        sim.setState(maxExtensionLimit, Math.min(0.0, sim.getVelocity()));
      }
      if (sparkSim != null) {
        sparkSim.setBusVoltage(12.0);
        sparkSim.setPosition(sim.getPosition());
        sparkSim.setVelocity(sim.getVelocity());
        sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
        sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
        sparkSim.setMotorCurrent(Math.abs(sim.getCurrentDraw()));
        sparkSim.setAppliedOutput(volts / 12.0);
      }
    }

    public void reset() {
      maxExtensionLimit = ExtenderConstants.MAX_EXTENSION;
      setPosition(ExtenderConstants.STARTING_EXTENSION);
      if (sparkSim != null) {
        sparkSim.setAppliedOutput(0.0);
      }
      if (motor != null) {
        motor.setVoltage(0.0);
      }
    }
  }

  /** Claw motor entry (CAN ID 4). */
  public static final class ClawMotorEntry {
    private final SingleJointedArmSim sim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            SimConstants.CLAW_GEARING,
            SimConstants.CLAW_FINGERS_MOI_KG_M2,
            SimConstants.CLAW_FINGER_LENGTH_METERS,
            ClawConstants.CLOSED_ANGLE,
            ClawConstants.OPEN_ANGLE,
            false, // no gravity
            ClawConstants.STARTING_ANGLE);
    private SparkMax motor;
    private SparkMaxSim sparkSim;
    private double gripStopAngle = Double.NaN;

    public void attachSparkMax(SparkMax sparkMax) {
      if (this.motor == sparkMax && this.sparkSim != null) {
        return;
      }
      this.motor = sparkMax;
      this.sparkSim = new SparkMaxSim(sparkMax, DCMotor.getNeo550(1));
      this.sparkSim.setBusVoltage(12.0);
      this.sparkSim.enable();
      this.sparkSim.setPosition(sim.getAngle());
      this.sparkSim.setVelocity(sim.getVelocity());
      this.sparkSim.getRelativeEncoderSim().setPosition(sim.getAngle());
      this.sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
    }

    public SparkMax getSparkMax() {
      return motor;
    }

    public SparkMaxSim getSparkMaxSim() {
      return sparkSim;
    }

    public void setGripStop(double angleRadians) {
      this.gripStopAngle = angleRadians;
    }

    public double getSimAngle() {
      return sim.getAngle();
    }

    public double getPosition() {
      return sim.getAngle();
    }

    public double getVelocity() {
      return sim.getVelocity();
    }

    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    public double getAppliedVolts() {
      if (!RobotBase.isEnabled()) {
        return 0.0;
      }
      return sparkSim != null ? sparkSim.getSetpoint() : 0.0;
    }

    public void setPosition(double position) {
      sim.setState(position, 0.0);
      if (sparkSim != null) {
        sparkSim.setPosition(position);
        sparkSim.setVelocity(0.0);
        sparkSim.getRelativeEncoderSim().setPosition(position);
        sparkSim.getRelativeEncoderSim().setVelocity(0.0);
      }
    }

    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (!Double.isNaN(gripStopAngle) && sim.getAngle() < gripStopAngle) {
        sim.setState(gripStopAngle, 0.0);
      }
      if (sparkSim != null) {
        sparkSim.setBusVoltage(12.0);
        sparkSim.setPosition(sim.getAngle());
        sparkSim.setVelocity(sim.getVelocity());
        sparkSim.getRelativeEncoderSim().setPosition(sim.getAngle());
        sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
        sparkSim.setMotorCurrent(Math.abs(sim.getCurrentDraw()));
        sparkSim.setAppliedOutput(volts / 12.0);
      }
    }

    public void reset() {
      gripStopAngle = Double.NaN;
      setPosition(ClawConstants.STARTING_ANGLE);
      if (sparkSim != null) {
        sparkSim.setAppliedOutput(0.0);
      }
      if (motor != null) {
        motor.setVoltage(0.0);
      }
    }
  }

  /** Shooter motor entry (CAN ID 5). */
  public static final class ShooterMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            SimConstants.SHOOTER_GEARING,
            SimConstants.SHOOTER_PLUNGER_MASS_KG,
            SimConstants.SHOOTER_DRUM_RADIUS_METERS,
            ShooterConstants.MIN_PULLBACK,
            ShooterConstants.MAX_PULLBACK,
            false,
            ShooterConstants.STARTING_PULLBACK);
    private SparkMax motor;
    private SparkMaxSim sparkSim;
    private double ratchetFloor = ShooterConstants.STARTING_PULLBACK;
    private double pendingKickStrength = 0.0;

    public void attachSparkMax(SparkMax sparkMax) {
      if (this.motor == sparkMax && this.sparkSim != null) {
        return;
      }
      this.motor = sparkMax;
      this.sparkSim = new SparkMaxSim(sparkMax, DCMotor.getNEO(1));
      this.sparkSim.setBusVoltage(12.0);
      this.sparkSim.enable();
      this.sparkSim.setPosition(sim.getPosition());
      this.sparkSim.setVelocity(sim.getVelocity());
      this.sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
      this.sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
    }

    public SparkMax getSparkMax() {
      return motor;
    }

    public SparkMaxSim getSparkMaxSim() {
      return sparkSim;
    }

    public double consumeKickStrength() {
      double strength = pendingKickStrength;
      pendingKickStrength = 0.0;
      return strength;
    }

    public double getSimPullback() {
      return sim.getPosition();
    }

    public double getPosition() {
      return sim.getPosition();
    }

    public double getVelocity() {
      return sim.getVelocity();
    }

    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    public double getAppliedVolts() {
      if (!RobotBase.isEnabled()) {
        return 0.0;
      }
      return sparkSim != null ? sparkSim.getSetpoint() : 0.0;
    }

    public void setPosition(double position) {
      sim.setState(position, 0.0);
      if (sparkSim != null) {
        sparkSim.setPosition(position);
        sparkSim.setVelocity(0.0);
        sparkSim.getRelativeEncoderSim().setPosition(position);
        sparkSim.getRelativeEncoderSim().setVelocity(0.0);
      }
    }

    public void update(double dtSeconds) {
      boolean enabled = RobotBase.isEnabled();
      boolean activeRelease = enabled && shooterSolenoidSim.getValue();

      if (activeRelease) {
        double currentPos = sim.getPosition();
        if (ratchetFloor >= ShooterConstants.LOADED_PULLBACK - ShooterConstants.TOLERANCE_METERS) {
          pendingKickStrength = 1.0;
        } else if (ratchetFloor > 0.05) {
          pendingKickStrength = Math.clamp(currentPos / ShooterConstants.LOADED_PULLBACK, 0.0, 1.0);
        }
        ratchetFloor = ShooterConstants.MIN_PULLBACK;

        double nextPos =
            Math.max(
                ShooterConstants.MIN_PULLBACK,
                currentPos - SimConstants.SHOOTER_SPRING_RELEASE_SPEED_MPS * dtSeconds);
        double nextVel =
            (nextPos > ShooterConstants.MIN_PULLBACK)
                ? -SimConstants.SHOOTER_SPRING_RELEASE_SPEED_MPS
                : 0.0;
        sim.setInputVoltage(0.0);
        sim.setState(nextPos, nextVel);
      } else {
        double volts = getAppliedVolts();
        if (volts > 0.0) {
          sim.setInputVoltage(volts);
          sim.update(dtSeconds);
          if (sim.getPosition() > ratchetFloor) {
            ratchetFloor = sim.getPosition();
          }
          if (sim.getPosition() >= ShooterConstants.MAX_PULLBACK) {
            ratchetFloor = ShooterConstants.MAX_PULLBACK;
            sim.setState(ShooterConstants.MAX_PULLBACK, 0.0);
          }
        } else {
          sim.setInputVoltage(volts);
          sim.setState(ratchetFloor, 0.0);
        }
      }
      if (sparkSim != null) {
        sparkSim.setBusVoltage(12.0);
        sparkSim.setPosition(sim.getPosition());
        sparkSim.setVelocity(sim.getVelocity());
        sparkSim.getRelativeEncoderSim().setPosition(sim.getPosition());
        sparkSim.getRelativeEncoderSim().setVelocity(sim.getVelocity());
        sparkSim.setMotorCurrent(Math.abs(sim.getCurrentDraw()));
        sparkSim.setAppliedOutput(getAppliedVolts() / 12.0);
      }
    }

    public void reset() {
      sim.setState(ShooterConstants.STARTING_PULLBACK, 0.0);
      ratchetFloor = ShooterConstants.STARTING_PULLBACK;
      pendingKickStrength = 0.0;
      shooterSolenoidSim.setValue(false);
      setPosition(ShooterConstants.STARTING_PULLBACK);
      if (sparkSim != null) {
        sparkSim.setAppliedOutput(0.0);
      }
      if (motor != null) {
        motor.setVoltage(0.0);
      }
    }
  }

  private static final DriveMotorEntry driveEntry = new DriveMotorEntry();
  private static final ElevatorMotorEntry elevatorEntry = new ElevatorMotorEntry();
  private static final ExtenderMotorEntry extenderEntry = new ExtenderMotorEntry();
  private static final ClawMotorEntry clawEntry = new ClawMotorEntry();
  private static final ShooterMotorEntry shooterEntry = new ShooterMotorEntry();

  private static final DIOSim clawBallSensorSim = new DIOSim(CLAW_BALL_SENSOR_DIO);
  private static final DIOSim shooterSolenoidSim = new DIOSim(SHOOTER_SOLENOID_CHANNEL);

  private static final Map<Integer, SparkMax> activeMotors = new HashMap<>();
  private static final Map<Integer, DigitalInput> activeDigitalInputs = new HashMap<>();
  private static final Map<Integer, DigitalOutput> activeDigitalOutputs = new HashMap<>();

  private static final Runnable physicsUpdate =
      () -> {
        driveEntry.update(LOOP_PERIOD_SECONDS);
        elevatorEntry.update(LOOP_PERIOD_SECONDS);
        extenderEntry.update(LOOP_PERIOD_SECONDS);
        clawEntry.update(LOOP_PERIOD_SECONDS);
        shooterEntry.update(LOOP_PERIOD_SECONDS);
      };

  static {
    Scheduler.getDefault().getDefaultEventLoop().bind(physicsUpdate);
  }

  private SimMechanical() {}

  /**
   * Creates a {@link SparkMax} for simulation, safely closing any previous instance on the same
   * CAN ID so that tests can re-instantiate mechanisms without native CAN conflicts.
   */
  public static SparkMax createSparkMax(int canId, MotorType type) {
    SparkMax existing = activeMotors.get(canId);
    if (existing != null) {
      return existing;
    }
    SparkMax motor = new SparkMax(CANPort.CAN_S0, canId, type);
    activeMotors.put(canId, motor);
    register(motor);
    return motor;
  }

  /**
   * Creates a {@link DigitalInput} for simulation, reusing any existing instance on the same DIO
   * channel.
   */
  public static DigitalInput createDigitalInput(int channel) {
    DigitalInput existing = activeDigitalInputs.get(channel);
    if (existing != null) {
      return existing;
    }
    DigitalInput input = new DigitalInput(channel);
    activeDigitalInputs.put(channel, input);
    return input;
  }

  /**
   * Creates a {@link DigitalOutput} for simulation, reusing any existing instance on the same DIO
   * channel.
   */
  public static DigitalOutput createDigitalOutput(int channel) {
    DigitalOutput existing = activeDigitalOutputs.get(channel);
    if (existing != null) {
      return existing;
    }
    DigitalOutput output = new DigitalOutput(channel);
    activeDigitalOutputs.put(channel, output);
    return output;
  }

  /**
   * Registers a mechanism's {@link SparkMax} motor controller with the simulation system,
   * attaching a {@link SparkMaxSim} and syncing physics to the motor encoder.
   */
  public static void register(SparkMax motor) {
    int canId = motor.getDeviceId();
    activeMotors.put(canId, motor);
    switch (canId) {
      case DRIVE_CAN_ID -> driveEntry.attachSparkMax(motor);
      case ELEVATOR_CAN_ID -> elevatorEntry.attachSparkMax(motor);
      case EXTENDER_CAN_ID -> extenderEntry.attachSparkMax(motor);
      case CLAW_CAN_ID -> clawEntry.attachSparkMax(motor);
      case SHOOTER_CAN_ID -> shooterEntry.attachSparkMax(motor);
      default -> throw new IllegalArgumentException("Unknown motor CAN ID: " + canId);
    }
  }

  /** Sets the simulated position of the motor specified by its CAN ID. */
  public static void setPosition(int canId, double pos) {
    switch (canId) {
      case DRIVE_CAN_ID -> driveEntry.setPosition(pos);
      case ELEVATOR_CAN_ID -> elevatorEntry.setPosition(pos);
      case EXTENDER_CAN_ID -> extenderEntry.setPosition(pos);
      case CLAW_CAN_ID -> clawEntry.setPosition(pos);
      case SHOOTER_CAN_ID -> shooterEntry.setPosition(pos);
      default -> throw new IllegalArgumentException("Unknown motor CAN ID: " + canId);
    }
  }

  public static DriveMotorEntry getDriveMotor() {
    return driveEntry;
  }

  public static ElevatorMotorEntry getElevatorMotor() {
    return elevatorEntry;
  }

  public static ExtenderMotorEntry getExtenderMotor() {
    return extenderEntry;
  }

  public static ClawMotorEntry getClawMotor() {
    return clawEntry;
  }

  public static ShooterMotorEntry getShooterMotor() {
    return shooterEntry;
  }

  public static DIOSim getClawBallSensor() {
    return clawBallSensorSim;
  }

  public static DIOSim getShooterSolenoid() {
    return shooterSolenoidSim;
  }

  /** Clears all scheduled commands, event loops, triggers, and default command bindings. */
  public static void resetScheduler() {
    Scheduler scheduler = Scheduler.getDefault();
    scheduler.cancelAll();
    scheduler.getDefaultEventLoop().clear();
    clearSchedulerCollection(scheduler, "m_defaultCommandBindings");
    clearSchedulerCollection(scheduler, "m_activeBindings");
    clearSchedulerCollection(scheduler, "m_boundTriggers");
    clearSchedulerCollection(scheduler, "m_queuedToRun");
    clearSchedulerCollection(scheduler, "m_runningCommands");
    clearSchedulerCollection(scheduler, "m_currentCommandAncestry");
    clearSchedulerCollection(scheduler, "m_periodicCallbacks");
    clearSchedulerCollection(scheduler, "m_eventListeners");
    scheduler.getDefaultEventLoop().bind(physicsUpdate);
  }

  private static void clearSchedulerCollection(Scheduler scheduler, String fieldName) {
    try {
      java.lang.reflect.Field field = Scheduler.class.getDeclaredField(fieldName);
      field.setAccessible(true);
      Object obj = field.get(scheduler);
      if (obj instanceof java.util.Collection<?> col) {
        col.clear();
      } else if (obj instanceof java.util.Map<?, ?> map) {
        map.clear();
      }
    } catch (Exception ignored) {
    }
  }

  /** Resets all active simulated motors and DIOs to their initial starting positions. */
  public static void resetAll() {
    resetScheduler();
    driveEntry.reset();
    elevatorEntry.reset();
    extenderEntry.reset();
    clawEntry.reset();
    shooterEntry.reset();
    clawBallSensorSim.setValue(false);
    shooterSolenoidSim.setValue(false);
  }
}
