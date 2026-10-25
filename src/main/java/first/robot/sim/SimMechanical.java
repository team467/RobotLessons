package first.robot.sim;

import static first.robot.Schematic.*;
import static first.robot.constants.SimulationConstants.LOOP_PERIOD_SECONDS;

import first.robot.constants.FieldConstants;
import first.robot.mechanisms.claw.ClawConstants;
import first.robot.mechanisms.drive.DriveConstants;
import first.robot.mechanisms.elevator.ElevatorConstants;
import first.robot.mechanisms.extender.ExtenderConstants;
import first.robot.mechanisms.shooter.ShooterConstants;
import java.util.HashMap;
import java.util.Map;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Central mechanical physics simulation and CAN/DIO hardware registry for the simulated robot.
 *
 * <p>Instantiates WPILib physics simulation models ({@link ElevatorSim}, {@link
 * SingleJointedArmSim}) and simulated sensors for each mechanism, registering them by CAN ID or DIO
 * port so that {@code *IOSim} classes can instantiate {@link FakeSparkMax} and {@link
 * FakeDigitalInput} devices just like real hardware.
 */
public final class SimMechanical {
  /** Common interface for a simulated motor entry in the CAN registry. */
  public interface SimMotorEntry {
    void setVoltage(double volts);

    double getAppliedVolts();

    double getPosition();

    double getVelocity();

    double getCurrentAmps();

    void setPosition(double position);

    void update(double dtSeconds);

    void reset();
  }

  /** Common interface for a simulated digital input (limit switch / beam break) in the DIO registry. */
  public static final class SimDigitalInputEntry {
    private boolean value = false;

    public boolean get() {
      return value;
    }

    public void set(boolean value) {
      this.value = value;
    }

    public void reset() {
      this.value = false;
    }
  }

  // ---- Drivetrain motor entry (CAN ID 1) ----
  public static final class DriveMotorEntry implements SimMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(2),
            DriveConstants.GEARING,
            DriveConstants.ROBOT_MASS_KG,
            DriveConstants.WHEEL_RADIUS_METERS,
            -1.0,
            FieldConstants.FIELD_LENGTH_METERS + 1.0,
            false,
            DriveConstants.START_POSITION);
    private double requestedVolts = 0.0;

    @Override
    public void setVoltage(double volts) {
      requestedVolts = volts;
    }

    @Override
    public double getAppliedVolts() {
      return RobotBase.isEnabled() ? requestedVolts : 0.0;
    }

    @Override
    public double getPosition() {
      return sim.getPosition();
    }

    @Override
    public double getVelocity() {
      return sim.getVelocity();
    }

    @Override
    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    @Override
    public void setPosition(double position) {
      sim.setState(position, 0.0);
      requestedVolts = 0.0;
    }

    @Override
    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
    }

    @Override
    public void reset() {
      setPosition(DriveConstants.START_POSITION);
    }
  }

  // ---- Elevator motor entry (CAN ID 2) ----
  public static final class ElevatorMotorEntry implements SimMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            ElevatorConstants.GEARING,
            ElevatorConstants.CARRIAGE_MASS_KG,
            ElevatorConstants.DRUM_RADIUS_METERS,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            true, // simulate gravity
            ElevatorConstants.STARTING_HEIGHT);
    private double requestedVolts = 0.0;
    private double minHeightLimit = ElevatorConstants.MIN_HEIGHT;

    public void setMinHeightLimit(double minHeight) {
      this.minHeightLimit = minHeight;
    }

    public double getMinHeightLimit() {
      return minHeightLimit;
    }

    @Override
    public void setVoltage(double volts) {
      requestedVolts = volts;
    }

    @Override
    public double getAppliedVolts() {
      return RobotBase.isEnabled() ? requestedVolts : 0.0;
    }

    @Override
    public double getPosition() {
      return sim.getPosition();
    }

    @Override
    public double getVelocity() {
      return sim.getVelocity();
    }

    @Override
    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    @Override
    public void setPosition(double position) {
      sim.setState(position, 0.0);
      requestedVolts = 0.0;
    }

    @Override
    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (sim.getPosition() < minHeightLimit) {
        sim.setState(minHeightLimit, Math.max(0.0, sim.getVelocity()));
      }
    }

    @Override
    public void reset() {
      sim.setState(ElevatorConstants.STARTING_HEIGHT, 0.0);
      requestedVolts = 0.0;
      minHeightLimit = ElevatorConstants.MIN_HEIGHT;
    }
  }

  // ---- Extender motor entry (CAN ID 3) ----
  public static final class ExtenderMotorEntry implements SimMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            ExtenderConstants.GEARING,
            ExtenderConstants.SLIDE_MASS_KG,
            ExtenderConstants.PULLEY_RADIUS_METERS,
            ExtenderConstants.MIN_EXTENSION,
            ExtenderConstants.MAX_EXTENSION,
            false, // horizontal: no gravity
            ExtenderConstants.STARTING_EXTENSION);
    private double requestedVolts = 0.0;
    private double maxExtensionLimit = ExtenderConstants.MAX_EXTENSION;

    public void setMaxExtensionLimit(double maxExtension) {
      this.maxExtensionLimit = maxExtension;
    }

    public double getMaxExtensionLimit() {
      return maxExtensionLimit;
    }

    @Override
    public void setVoltage(double volts) {
      requestedVolts = volts;
    }

    @Override
    public double getAppliedVolts() {
      return RobotBase.isEnabled() ? requestedVolts : 0.0;
    }

    @Override
    public double getPosition() {
      return sim.getPosition();
    }

    @Override
    public double getVelocity() {
      return sim.getVelocity();
    }

    @Override
    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    @Override
    public void setPosition(double position) {
      sim.setState(position, 0.0);
      requestedVolts = 0.0;
    }

    @Override
    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (sim.getPosition() > maxExtensionLimit) {
        sim.setState(maxExtensionLimit, Math.min(0.0, sim.getVelocity()));
      }
    }

    @Override
    public void reset() {
      sim.setState(ExtenderConstants.STARTING_EXTENSION, 0.0);
      requestedVolts = 0.0;
      maxExtensionLimit = ExtenderConstants.MAX_EXTENSION;
    }
  }

  // ---- Claw motor entry (CAN ID 4) ----
  public static final class ClawMotorEntry implements SimMotorEntry {
    private final SingleJointedArmSim sim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            ClawConstants.GEARING,
            ClawConstants.FINGERS_MOI_KG_M2,
            ClawConstants.FINGER_LENGTH_METERS,
            ClawConstants.CLOSED_ANGLE,
            ClawConstants.OPEN_ANGLE,
            false, // no gravity
            ClawConstants.STARTING_ANGLE);
    private double requestedVolts = 0.0;
    private double gripStopAngle = Double.NaN;

    public void setGripStop(double angleRadians) {
      this.gripStopAngle = angleRadians;
    }

    public double getSimAngle() {
      return sim.getAngle();
    }

    @Override
    public void setVoltage(double volts) {
      requestedVolts = volts;
    }

    @Override
    public double getAppliedVolts() {
      return RobotBase.isEnabled() ? requestedVolts : 0.0;
    }

    @Override
    public double getPosition() {
      return sim.getAngle();
    }

    @Override
    public double getVelocity() {
      return sim.getVelocity();
    }

    @Override
    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    @Override
    public void setPosition(double position) {
      sim.setState(position, 0.0);
      requestedVolts = 0.0;
    }

    @Override
    public void update(double dtSeconds) {
      double volts = getAppliedVolts();
      sim.setInputVoltage(volts);
      sim.update(dtSeconds);
      if (!Double.isNaN(gripStopAngle) && sim.getAngle() < gripStopAngle) {
        sim.setState(gripStopAngle, 0.0);
      }
    }

    @Override
    public void reset() {
      sim.setState(ClawConstants.STARTING_ANGLE, 0.0);
      requestedVolts = 0.0;
      gripStopAngle = Double.NaN;
    }
  }

  // ---- Shooter motor entry (CAN ID 5) ----
  public static final class ShooterMotorEntry implements SimMotorEntry {
    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            ShooterConstants.GEARING,
            ShooterConstants.PLUNGER_MASS_KG,
            ShooterConstants.DRUM_RADIUS_METERS,
            ShooterConstants.MIN_PULLBACK,
            ShooterConstants.MAX_PULLBACK,
            false,
            ShooterConstants.STARTING_PULLBACK);
    private double requestedVolts = 0.0;
    private boolean solenoidReleased = false;
    private double ratchetFloor = ShooterConstants.STARTING_PULLBACK;
    private double pendingKickStrength = 0.0;

    public void setSolenoidReleased(boolean released) {
      this.solenoidReleased = released;
    }

    public boolean isSolenoidReleased() {
      return solenoidReleased;
    }

    public double consumeKickStrength() {
      double strength = pendingKickStrength;
      pendingKickStrength = 0.0;
      return strength;
    }

    public double getSimPullback() {
      return sim.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
      requestedVolts = volts;
    }

    @Override
    public double getAppliedVolts() {
      return RobotBase.isEnabled() ? requestedVolts : 0.0;
    }

    @Override
    public double getPosition() {
      return sim.getPosition();
    }

    @Override
    public double getVelocity() {
      return sim.getVelocity();
    }

    @Override
    public double getCurrentAmps() {
      return Math.abs(sim.getCurrentDraw());
    }

    @Override
    public void setPosition(double position) {
      sim.setState(position, 0.0);
      requestedVolts = 0.0;
    }

    @Override
    public void update(double dtSeconds) {
      boolean enabled = RobotBase.isEnabled();
      boolean activeRelease = enabled && solenoidReleased;

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
                currentPos - ShooterConstants.SPRING_RELEASE_SPEED_MPS * dtSeconds);
        double nextVel =
            (nextPos > ShooterConstants.MIN_PULLBACK)
                ? -ShooterConstants.SPRING_RELEASE_SPEED_MPS
                : 0.0;
        sim.setInputVoltage(0.0);
        sim.setState(nextPos, nextVel);
      } else {
        double volts = getAppliedVolts();
        // With the one-way ratchet engaged, only positive pullback voltage can advance the plunger
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
    }

    @Override
    public void reset() {
      sim.setState(ShooterConstants.STARTING_PULLBACK, 0.0);
      requestedVolts = 0.0;
      solenoidReleased = false;
      ratchetFloor = ShooterConstants.STARTING_PULLBACK;
      pendingKickStrength = 0.0;
    }
  }

  private static final Map<Integer, SimMotorEntry> registry = new HashMap<>();
  private static final Map<Integer, SimDigitalInputEntry> dioRegistry = new HashMap<>();

  static {
    resetAll();
  }

  private SimMechanical() {}

  /**
   * Creates and registers a fresh simulated motor entry in the CAN registry for {@code canId}.
   * Calling this when {@link FakeSparkMax} is constructed ensures each newly constructed {@code
   * *IOSim} gets its own fresh simulation instance while remaining accessible by CAN ID in the
   * registry.
   */
  public static SimMotorEntry createMotor(int canId) {
    SimMotorEntry entry =
        switch (canId) {
          case DRIVE_CAN_ID -> new DriveMotorEntry();
          case ELEVATOR_CAN_ID -> new ElevatorMotorEntry();
          case EXTENDER_CAN_ID -> new ExtenderMotorEntry();
          case CLAW_CAN_ID -> new ClawMotorEntry();
          case SHOOTER_CAN_ID -> new ShooterMotorEntry();
          default ->
              throw new IllegalArgumentException(
                  "No simulated motor configured in SimMechanical for CAN ID: " + canId);
        };
    registry.put(canId, entry);
    return entry;
  }

  /** Looks up the active registered simulated motor by its CAN ID. */
  public static SimMotorEntry getMotor(int canId) {
    SimMotorEntry motor = registry.get(canId);
    if (motor == null) {
      return createMotor(canId);
    }
    return motor;
  }

  /** Creates and registers a fresh simulated digital input in the DIO registry for {@code dioPort}. */
  public static SimDigitalInputEntry createDigitalInput(int dioPort) {
    SimDigitalInputEntry entry =
        switch (dioPort) {
          case CLAW_BALL_SENSOR_DIO -> new SimDigitalInputEntry();
          default ->
              throw new IllegalArgumentException(
                  "No simulated digital input configured in SimMechanical for DIO port: " + dioPort);
        };
    dioRegistry.put(dioPort, entry);
    return entry;
  }

  /** Looks up the active registered simulated digital input by its DIO port. */
  public static SimDigitalInputEntry getDigitalInput(int dioPort) {
    SimDigitalInputEntry entry = dioRegistry.get(dioPort);
    if (entry == null) {
      return createDigitalInput(dioPort);
    }
    return entry;
  }

  public static DriveMotorEntry getDriveMotor() {
    return (DriveMotorEntry) getMotor(DRIVE_CAN_ID);
  }

  public static ElevatorMotorEntry getElevatorMotor() {
    return (ElevatorMotorEntry) getMotor(ELEVATOR_CAN_ID);
  }

  public static ExtenderMotorEntry getExtenderMotor() {
    return (ExtenderMotorEntry) getMotor(EXTENDER_CAN_ID);
  }

  public static ClawMotorEntry getClawMotor() {
    return (ClawMotorEntry) getMotor(CLAW_CAN_ID);
  }

  public static ShooterMotorEntry getShooterMotor() {
    return (ShooterMotorEntry) getMotor(SHOOTER_CAN_ID);
  }

  /** Steps a specific motor's simulation by {@link SimulationConstants#LOOP_PERIOD_SECONDS}. */
  public static void updateMotor(int canId) {
    getMotor(canId).update(LOOP_PERIOD_SECONDS);
  }

  /** Resets all registered motors and digital inputs to fresh instances. */
  public static void resetAll() {
    createMotor(DRIVE_CAN_ID);
    createMotor(ELEVATOR_CAN_ID);
    createMotor(EXTENDER_CAN_ID);
    createMotor(CLAW_CAN_ID);
    createMotor(SHOOTER_CAN_ID);
    createDigitalInput(CLAW_BALL_SENSOR_DIO);
  }
}
