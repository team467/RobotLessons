package first.robot.sim;

import first.robot.sim.SimMechanical.SimDigitalInputEntry;

/**
 * A simulated WPILib {@code DigitalInput} sensor (such as a limit switch or beam-break sensor) that
 * connects by DIO channel to a digital input entry in {@link SimMechanical}.
 *
 * <p>Allows {@code *IOSim} hardware classes to be written just like real robot hardware IO classes
 * (instantiating a digital input by DIO port and calling {@link #get()} in {@code updateInputs}).
 */
public class FakeDigitalInput {
  private final int channel;
  private final SimDigitalInputEntry entry;

  /**
   * Creates a {@link FakeDigitalInput} attached to the simulated DIO channel {@code channel} in
   * {@link SimMechanical}.
   */
  public FakeDigitalInput(int channel) {
    this.channel = channel;
    this.entry = SimMechanical.createDigitalInput(channel);
  }

  /** Returns the DIO channel number this digital input is attached to. */
  public int getChannel() {
    return channel;
  }

  /** Gets the value from the digital input (true when triggered/pressed). */
  public boolean get() {
    return entry.get();
  }

  /** Sets the simulated state of the digital input (used by the simulation world). */
  public void set(boolean value) {
    entry.set(value);
  }

  /** Resets the digital input back to its initial untriggered state. */
  public void resetSim() {
    entry.reset();
  }
}
