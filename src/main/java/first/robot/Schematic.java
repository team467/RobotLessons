package first.robot;

/** Electrical schematic constants (CAN IDs and hardware port assignments) for the robot. */
public final class Schematic {
  private Schematic() {}

  // ---- USB Ports ----
  public static final int DRIVER_CONTROLLER_PORT = 1;

  // ---- Motor Controller CAN IDs ----
  public static final int DRIVE_CAN_ID = 1;
  public static final int Elevator_spark_max_CAN_ID = 2;
  public static final int Claw_spark_max_CAN_ID = 4;
  public static final int Kicker__spark_max_CAN_ID = 5;
  public static final int Extender_spark_max_CAN_ID = 3;

  // ---- Digital I/O ---- 
  public static final int Claw_ball_sensor_PORT = 1;
  public static final int Kicker_solenoid_PORT = 2;
  public static final int LEDs_PORT = 5;
}
