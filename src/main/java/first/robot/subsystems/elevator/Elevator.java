package first.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import first.robot.Schematic;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import first.robot.mechanical.Mechanical;

import com.revrobotics.RelativeEncoder;

@Logged
public class Elevator implements Mechanism {

    private final SparkMax motor = Mechanical.createSparkMax(Schematic.ELEVATOR_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    public Elevator() {
        setDefaultCommand(hold());
    }

    public Command up() {
        return run(coroutine -> {
            while (encoder.getPosition().get() < 1.15) {
                motor.setVoltage(6.0);
                coroutine.yield();
            }
        }).named("Elevator Up");
    }

    public Command down() {
        return run(coroutine -> {
            while (encoder.getPosition().get() > 0.30) {
                motor.setVoltage(-6.0);
                coroutine.yield();
            }
        }).named("Elevator Down");
    }

    public Command hold() {
        return run(coroutine -> {
            while (true) {
                motor.setVoltage(0.0);
                coroutine.yield();
            }
        }).named("hold");
    }
}