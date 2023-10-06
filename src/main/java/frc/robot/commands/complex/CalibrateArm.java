package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.extender.ArmExtender;

public class CalibrateArm extends CommandBase {
  private ArmExtender extender;

  public CalibrateArm(ArmExtender extender) {
    this.extender = extender;
    addRequirements(extender);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    extender.manual(1.5);
  }

  @Override
  public void end(boolean interrupted) {
    extender.hold();
  }
}
