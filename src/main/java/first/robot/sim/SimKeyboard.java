package first.robot.sim;

import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.wpilib.command3.Trigger;

/**
 * Turns keyboard keys into command {@link Trigger}s, just like gamepad buttons.
 *
 * <p>Keys are only seen while the simulator window (or another window in this program) has focus.
 * Key events arrive on the Swing thread and are read on the robot thread, so the pressed-key set
 * is thread-safe.
 */
public class SimKeyboard {
  private final Set<Integer> pressed = ConcurrentHashMap.newKeySet();
  private boolean installed = false;

  /**
   * A trigger that is true while {@code keyCode} is held down.
   *
   * @param keyCode a {@link KeyEvent} constant such as {@code KeyEvent.VK_UP}
   */
  public Trigger key(int keyCode) {
    return new Trigger(() -> pressed.contains(keyCode));
  }

  /** Returns whether a key is currently held down. */
  public boolean isPressed(int keyCode) {
    return pressed.contains(keyCode);
  }

  /** Sets whether a key is held down (useful for testing and simulated input). */
  public void setPressed(int keyCode, boolean isPressed) {
    if (isPressed) {
      pressed.add(keyCode);
    } else {
      pressed.remove(keyCode);
    }
  }

  /** Forgets all held keys (for example when the window loses focus). */
  public void releaseAll() {
    pressed.clear();
  }

  /** Starts listening to key events from every window in this program. Call on the Swing thread. */
  void install() {
    if (installed) {
      return;
    }
    installed = true;
    KeyboardFocusManager.getCurrentKeyboardFocusManager()
        .addKeyEventDispatcher(
            event -> {
              if (event.getID() == KeyEvent.KEY_PRESSED) {
                pressed.add(event.getKeyCode());
              } else if (event.getID() == KeyEvent.KEY_RELEASED) {
                pressed.remove(event.getKeyCode());
              }
              return false; // let buttons etc. still see the event
            });
  }
}
