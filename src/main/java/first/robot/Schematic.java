package first.robot;

/** Electrical schematic constants (CAN IDs and hardware port assignments) for the robot. */
public final class Schematic {
  private Schematic() {}

  // ---- USB Ports ----
  public static final int DRIVER_CONTROLLER_PORT = 1;

  // ---- Motor Controller CAN IDs ----
  public static final int DRIVE_CAN_ID = 1;
  public static final int ELEVATOR_NEO_ID = 2;
  public static final int CLAW_SPARK_MAX_ID = 4;
  public static final int KICKER_SPARK_MAX_ID = 5;
  public static final int DRIVE_SPARK_MAX_ID = 1;
  public static final int EXTENDER_SPARK_MAX_ID = 3;
  public static final int KICKER_SOLENOID_DIO = 2;
  public static final int CLAW_BALL_SENSOR_DIO = 1;
  public static final int LEDS_DIO = 5;
  public static final int DRIVER_CONTROLLER_USB = 1;
  public static final int OPERATOR_CONTROLLER = 2;

}
