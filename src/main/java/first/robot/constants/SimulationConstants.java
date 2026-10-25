package first.robot.constants;

/**
 * Constants governing simulation time stepping and physics updates.
 */
public final class SimulationConstants {
  private SimulationConstants() {}

  /** Standard simulation and robot loop update period in seconds (20ms, 50Hz). */
  public static final double LOOP_PERIOD_SECONDS = 0.02;
}
