package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.NewClaw;

public class ClawMovement extends CommandBase {

    private final NewClaw m_claw;
    private double m_setPoint; // private final

    public ClawMovement(NewClaw claw, double setPoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;
        this.m_setPoint = setPoint;

        addRequirements(m_claw);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_claw.setSetpoint(m_setPoint);
        m_claw.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_claw.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_claw.getDutyPosition() - m_setPoint) <= kClaw.encoderTolerance;
    }

}