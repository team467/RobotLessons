package first.robot.sim;

/**
 * Physical simulation parameters (motor gearing, masses, drum radii, inertia, and launch physics)
 * used exclusively by the simulation subsystem.
 */
public final class SimConstants {
  private SimConstants() {}

  // ---- Drivetrain Simulation ----
  public static final double DRIVE_WHEEL_RADIUS_METERS = 0.076;
  public static final double DRIVE_ROBOT_MASS_KG = 25.0;
  public static final double DRIVE_CHASSIS_LENGTH_METERS = 0.70;
  public static final double DRIVE_CHASSIS_WIDTH_METERS = 0.60;
  public static final double DRIVE_GEARING = 8.45;
  public static final double DRIVE_MAX_SPEED_MPS = 3.5;

  // ---- Elevator Simulation ----
  public static final double ELEVATOR_GEARING = 10.0;
  public static final double ELEVATOR_CARRIAGE_MASS_KG = 6.0;
  public static final double ELEVATOR_DRUM_RADIUS_METERS = 0.025;

  // ---- Extender Simulation ----
  public static final double EXTENDER_GEARING = 8.0;
  public static final double EXTENDER_SLIDE_MASS_KG = 3.0;
  public static final double EXTENDER_PULLEY_RADIUS_METERS = 0.02;

  // ---- Claw Simulation ----
  public static final double CLAW_GEARING = 100.0;
  public static final double CLAW_FINGERS_MOI_KG_M2 = 0.005;
  public static final double CLAW_FINGER_LENGTH_METERS = 0.22;

  // ---- Shooter Simulation ----
  public static final double SHOOTER_GEARING = 10.0;
  public static final double SHOOTER_PLUNGER_MASS_KG = 1.5;
  public static final double SHOOTER_DRUM_RADIUS_METERS = 0.015;
  public static final double SHOOTER_SPRING_RELEASE_SPEED_MPS = 5.0;

  // ---- Shooter Strike Zone & Launch Physics ----
  public static final double SHOOTER_MIN_STRIKE_EXTENSION = 0.17;
  public static final double SHOOTER_MAX_STRIKE_EXTENSION = 0.29;
  public static final double SHOOTER_MIN_STRIKE_WRIST_Y = 0.53;
  public static final double SHOOTER_MAX_STRIKE_WRIST_Y = 0.67;
  public static final double SHOOTER_LAUNCH_VX_MPS = 1.44;
  public static final double SHOOTER_LAUNCH_VY_MPS = 5.10;
}
