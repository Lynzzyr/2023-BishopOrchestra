package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class AutoCloseClaw extends CommandBase {

    private final Claw m_claw;

    private final double setpoint;

    private boolean finished = false;
    private int closeDistance;

    public AutoCloseClaw(Claw claw, double setpoint, int closeDistance) {
        m_claw = claw;
        this.setpoint = setpoint;
        this.closeDistance = closeDistance;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_claw);   
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_claw.rollingToFAvg(m_claw.getLeftToF()) <= closeDistance) {
            m_claw.setSetpoint(setpoint);
            m_claw.enable();
            finished = true;
        } else if (m_claw.rollingToFAvg(m_claw.getRightToF()) <= closeDistance) {
            m_claw.setSetpoint(setpoint);
            m_claw.enable();
            finished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_claw.setSetpoint(setpoint);
        // m_claw.enable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        
        return finished;
    }

}