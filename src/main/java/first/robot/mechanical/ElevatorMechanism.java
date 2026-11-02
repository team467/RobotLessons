package first.robot.mechanical;

import static first.robot.subsystems.elevator.ElevatorConstants.KD;
import static first.robot.subsystems.elevator.ElevatorConstants.KG;
import static first.robot.subsystems.elevator.ElevatorConstants.KP;
import static first.robot.subsystems.elevator.ElevatorConstants.MAX_HEIGHT;
import static first.robot.subsystems.elevator.ElevatorConstants.MAX_VOLTS;
import static first.robot.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;
import static first.robot.subsystems.elevator.ElevatorConstants.STARTING_HEIGHT;
import static first.robot.subsystems.elevator.ElevatorConstants.TOLERANCE;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.units.measure.Distance;

/**
 * Linear position mechanism tailored for the elevator, preconfigured with elevator
 * PID gains, feedforward, travel limits, and voltage limits.
 */
public class ElevatorMechanism extends LinearPositionMechanism {
  public ElevatorMechanism(
      Consumer<Double> setVoltage,
      Supplier<Distance> getPosition,
      DoubleSupplier getVelocity) {
    super(
        "Elevator",
        setVoltage,
        getPosition,
        getVelocity,
        KP,
        KD,
        KG,
        MAX_VOLTS,
        TOLERANCE,
        MIN_HEIGHT,
        MAX_HEIGHT,
        STARTING_HEIGHT);
  }
}
