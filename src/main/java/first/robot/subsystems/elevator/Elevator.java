package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import first.lib.mechanism.LinearPositionMechanism;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import org.wpilib.command3.Command;
import org.wpilib.epilogue.Logged;

/** The elevator moves the whole arm up and down (the y-axis). */
@Logged
public class Elevator extends LinearPositionMechanism {
  public Elevator() {
    super(
        "Elevator",
        Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless),
        KP,
        KD,
        KG,
        MAX_VOLTS,
        TOLERANCE,
        MIN_HEIGHT,
        MAX_HEIGHT,
        STARTING_HEIGHT);
  }

  public Command up() {
    return jog("Elevator Up", MANUAL_SPEED_MPS * 0.02);
  }

  public Command down() {
    return jog("Elevator Down", -MANUAL_SPEED_MPS * 0.02);
  }
}
