package first.robot.sim;

import first.robot.sim.SimMechanical.SimSolenoidEntry;

/**
 * A simulated pneumatic solenoid that connects by channel to a solenoid entry in {@link
 * SimMechanical}.
 */
public class FakeSolenoid {
  private final int channel;
  private final SimSolenoidEntry entry;

  public FakeSolenoid(int channel) {
    this.channel = channel;
    this.entry = SimMechanical.createSolenoid(channel);
  }

  public int getChannel() {
    return channel;
  }

  public void set(boolean on) {
    entry.set(on);
  }

  public boolean get() {
    return entry.get();
  }
}
