package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.FlyWheelIO.FlyWheelIOInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlyWheel extends SubsystemBase {
  private final FlyWheelIO io;
  private final FlyWheelIOInputs inputs = new FlyWheelIOInputs();

  public FlyWheel(FlyWheelIO io) {
    this.io = io;
  }

  public void periodic() {
    updateInputsAndLog();
  }

  private void updateInputsAndLog() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("FlyWheel", (LoggableInputs) inputs);
  }

  public void start() {
    System.out.println("Flywheel has started.");
    io.setVoltage(10.0);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  // Commands 101

  public Command startCommand() {
    return Commands.runEnd(() -> this.start(), this::stop, this);
  }

  public Command stopCommand() {
    return Commands.runEnd(() -> this.stop(), this::stop, this);
  }
}
