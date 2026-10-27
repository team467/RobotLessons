package first.robot.mechanical;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.sim.SimMechanical;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.hardware.discrete.DigitalOutput;

/**
 * Factory and hardware adapter for creating and managing motors and DIO peripherals across both
 * real hardware and simulation environments transparently.
 */
public final class Mechanical {
  private Mechanical() {}

  /**
   * Creates a {@link SparkMax} motor controller, automatically hooking up physics simulation
   * when running in simulation mode.
   */
  public static SparkMax createSparkMax(int canId, MotorType type) {
    if (RobotBase.isSimulation()) {
      return SimMechanical.createSparkMax(canId, type);
    }
    return new SparkMax(CANPort.CAN_S0, canId, type);
  }

  /**
   * Creates a {@link DigitalInput} sensor, automatically backed by simulated DIO when in simulation.
   */
  public static DigitalInput createDigitalInput(int channel) {
    if (RobotBase.isSimulation()) {
      return SimMechanical.createDigitalInput(channel);
    }
    return new DigitalInput(channel);
  }

  /**
   * Creates a {@link DigitalOutput} channel, automatically backed by simulated DIO when in simulation.
   */
  public static DigitalOutput createDigitalOutput(int channel) {
    if (RobotBase.isSimulation()) {
      return SimMechanical.createDigitalOutput(channel);
    }
    return new DigitalOutput(channel);
  }

  /**
   * Registers a motor controller with the simulation system if running in simulation.
   */
  public static void register(SparkMax motor) {
    if (RobotBase.isSimulation()) {
      SimMechanical.register(motor);
    }
  }

  /**
   * Sets the position of the encoder, transparently syncing either simulated physics or real
   * hardware encoder registers.
   */
  public static void setEncoderPosition(SparkMax motor, RelativeEncoder encoder, double pos) {
    if (RobotBase.isSimulation()) {
      SimMechanical.setPosition(motor.getDeviceId(), pos);
    } else {
      encoder.setPosition(pos);
    }
  }
}
