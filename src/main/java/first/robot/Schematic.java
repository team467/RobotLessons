package first.robot;

/** Electrical schematic constants (CAN IDs and hardware port assignments) for the robot. */
public final class Schematic {
  private Schematic() {}

  // ---- USB Ports ----
  public static final int DRIVER_CONTROLLER_PORT = 1;

  // ---- Motor Controller CAN IDs ----
  public static final int DRIVE_CAN_ID = 0;
}
