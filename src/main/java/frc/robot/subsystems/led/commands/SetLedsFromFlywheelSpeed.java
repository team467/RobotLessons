package frc.robot.subsystems.led.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.Led.COLORS_467;

public class SetLedsFromFlywheelSpeed extends Command {

  private final Led led;
  private final Flywheel flywheel;

  public SetLedsFromFlywheelSpeed(Led led, Flywheel flywheel) {
      this.led = led;
      this.flywheel = flywheel;

      // Only add led as a requirement, not flywheel because we don't want to interrupt the flywheel
      addRequirements(led);
  }

/** The initial subroutine of a command. Called once when the command is initially scheduled. */
public void initialize() {}

/** The main body of a command. Called repeatedly while the command is scheduled. */
public void execute() {
  double speed = flywheel.appliedVoltage() / 12.0;
  if (speed > 0.0) {
    led.setColorMovingUp(COLORS_467.Gold, COLORS_467.Black, speed);
  } else {
    led.setColorMovingDown(COLORS_467.Blue, COLORS_467.Black, speed);
  }

}

/**
 * Whether the command has finished. Once a command finishes, the scheduler will call its end()
 * method and un-schedule it.
 *
 * @return whether the command has finished.
 */
public boolean isFinished() {
  return false;
}
    
}
