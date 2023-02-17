package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase {

    private final Claw m_claw;

    public OpenClaw(Claw claw) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;
        
        addRequirements(m_claw);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_claw.setPIDF(kClaw.kP, 0, 0, 0);
        m_claw.clawGoTo(kClaw.openPosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_claw.stopMot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return 
        Math.abs(kClaw.openPosition - m_claw.getEncoderPosition()) <= kClaw.encoderOffset
        && 
        (m_claw.getDistanceFromClaw() <= kClaw.objectRange && m_claw.getDistanceFromClaw() != 0);
    }

}