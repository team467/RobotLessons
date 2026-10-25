package first.robot;

/** Electrical schematic constants (CAN IDs and hardware port assignments) for the robot. */
public final class Schematic {
  private Schematic() {}

  // ---- Motor Controller CAN IDs ----
  public static final int DRIVE_CAN_ID = 1;
  public static final int ELEVATOR_CAN_ID = 2;
  public static final int EXTENDER_CAN_ID = 3;
  public static final int CLAW_CAN_ID = 4;
  public static final int SHOOTER_CAN_ID = 5;

  // ---- Digital I/O (DIO) Ports ----
  public static final int CLAW_BALL_SENSOR_DIO = 0;
}
