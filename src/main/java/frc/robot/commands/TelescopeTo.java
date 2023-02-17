package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class TelescopeTo extends CommandBase {

    private final Telescope sys_telescope;
    private final double setPoint;

    public TelescopeTo(Telescope telescope, double destination) {
        sys_telescope = telescope;
        setPoint = destination;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_telescope);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_telescope.extend(setPoint);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (sys_telescope.getSwitch().getActiveLimitSwitch()) {
            return true;
        }
        
        //else if (Math.abs(setPoint - sys_telescope.getDistance()) <= 1) {
        //     return true;
        // }
        return false;
    }

}