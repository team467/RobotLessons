package first.lib.state;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;

/**
 * Wraps a {@link StateMachine} whose states are identified by a key, and remembers which state is
 * active and which one a caller has asked for. {@link #setGoal} turns a request into a {@link
 * Command} that finishes once the machine has entered the requested state and then left it again.
 *
 * @param <G> the key type identifying states (typically an enum)
 */
public class TrackedStateMachine<G> {
  private final String name;
  private final StateMachine machine;
  private final Map<G, StateMachine.State> states = new HashMap<>();

  private G requested;
  private G active;
  private int entries;

  public TrackedStateMachine(String name) {
    this.name = name;
    this.machine = new StateMachine(name);
  }

  /**
   * Adds a state to the machine.
   *
   * @param key identifies the state in {@link #state}, {@link #isRequested} and {@link #setGoal}
   * @param command the command to run while the state is active
   */
  public StateMachine.State addState(G key, Command command) {
    StateMachine.State state = machine.addState(command);
    state.onEnter(() -> {
      active = key;
      entries++;
      requested = null;
    });
    states.put(key, state);
    return state;
  }

  /**
   * Adds a transition from {@code from} to {@code to} when {@code condition} is true.
   *
   * <p>Automatically evaluates to {@code false} on the pre-yield check immediately following state
   * entry so that {@link StateMachine}'s internal rising-edge detector resets cleanly even if
   * {@code condition} was already true when entering {@code from}.
   */
  public void transition(G from, G to, BooleanSupplier condition) {
    int[] entryOnFirstCheck = {-1};
    state(from)
        .switchTo(state(to))
        .when(
            () -> {
              if (entryOnFirstCheck[0] != entries) {
                entryOnFirstCheck[0] = entries;
                return false;
              }
              return condition.getAsBoolean();
            });
  }

  /**
   * Adds a transition from {@code from} to {@code to} when {@code from}'s command completes.
   */
  public void transitionWhenComplete(G from, G to) {
    state(from).switchTo(state(to)).whenComplete();
  }

  /** Returns the state added with the given key, for declaring transitions. */
  public StateMachine.State state(G key) {
    return states.get(key);
  }

  public void setInitialState(G key) {
    machine.setInitialState(states.get(key));
  }

  /**
   * Returns a low-priority command that requires {@code mechanism} and continuously runs this state
   * machine, suitable for {@link Mechanism#setDefaultCommand(Command)}.
   */
  public Command runOn(Mechanism mechanism) {
    return mechanism
        .run(coroutine -> coroutine.await(machine))
        .withPriority(Command.LOWEST_PRIORITY)
        .named(name + " Hold");
  }

  /** A condition, for use in transitions, that is true while {@code key} has been requested. */
  public BooleanSupplier isRequested(G key) {
    return () -> requested == key;
  }

  /** Requests {@code goal}; the command finishes after the machine enters it and leaves it. */
  public Command setGoal(G goal) {
    return Command.noRequirements(coroutine -> {
      int start = entries;
      requested = goal;
      coroutine.waitUntil(() -> entries > start && active != goal);
    }).named(name + " " + goal);
  }
}
