package first.robot.subsystems.extender;

import static first.robot.subsystems.extender.ExtenderConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import first.lib.mechanism.LinearPositionMechanism;
import first.robot.Schematic;
import first.robot.mechanical.Mechanical;
import org.wpilib.command3.Command;
import org.wpilib.epilogue.Logged;

/** The extender slides the claw in and out on a pulley (the x-axis). */
@Logged
public class Extender extends LinearPositionMechanism {
  public Extender() {
    super(
        "Extender",
        Mechanical.createSparkMax(Schematic.EXTENDER_CAN_ID, MotorType.kBrushless),
        KP,
        KD,
        MAX_VOLTS,
        TOLERANCE,
        MIN_EXTENSION,
        MAX_EXTENSION,
        STARTING_EXTENSION);
  }

  public Command out() {
    return jog("Extender Out", MANUAL_SPEED_MPS * 0.02);
  }

  public Command in() {
    return jog("Extender In", -MANUAL_SPEED_MPS * 0.02);
  }
}
