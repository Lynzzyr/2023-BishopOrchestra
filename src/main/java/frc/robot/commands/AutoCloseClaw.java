package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewClaw;

public class AutoCloseClaw extends CommandBase {

    private final NewClaw m_claw;

    private final double setpoint;

    private boolean finished = false;
    private int closeDistance;

    public AutoCloseClaw(NewClaw claw, double setpoint, int closeDistance) {
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
        if (m_claw.rollingToFAvg() <= closeDistance) {
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