package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class SpinFlywheelToSpeed extends Command {

    private final Flywheel flywheel;

    public SpinFlywheelToSpeed(Flywheel flywheel) {
        this.flywheel = flywheel;
        addRequirements(this.flywheel);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        flywheel.spinToSpeed(25).execute();
    }


    @Override
    public boolean isFinished() {
        // Run until interrupted
        return false;
    }
    
}
