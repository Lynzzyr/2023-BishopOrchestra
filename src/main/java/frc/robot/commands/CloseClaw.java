package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase {

    private final Claw m_claw;

    public CloseClaw(Claw claw) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;

        addRequirements(m_claw);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_claw.setPIDF(kClaw.kP, kClaw.kI, kClaw.kD, kClaw.kF);
        m_claw.clawGoTo(kClaw.closePosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_claw.stopMot();
        m_claw.setPIDF(kClaw.kP, kClaw.kI, kClaw.kD, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return Math.abs(m_claw.getEncoderPosition()) >= 17500;
        // return m_claw.getDistanceFromClaw() <= (kClaw.objectRange + 50) && m_claw.getDistanceFromClaw() != 0;
        return Math.abs(kClaw.closePosition - m_claw.getEncoderPosition()) <= kClaw.encoderOffset;
    }

}