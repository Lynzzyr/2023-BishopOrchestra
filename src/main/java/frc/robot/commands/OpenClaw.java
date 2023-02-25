package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase {

    private final Claw m_claw;
    private final boolean isAuto;

    public OpenClaw(Claw claw, boolean auto) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;
        isAuto = auto;
        
        addRequirements(m_claw);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_claw.openClaw();
    }

    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!isAuto) {
            m_claw.stopMot();
        }
    }

    @Override
    public boolean isFinished() {
        // return (!isAuto) &&  || m_claw.getDistanceFromClaw() <= kClaw.objectRange;
        if (isAuto) {
            return m_claw.getDistanceFromClaw() <= kClaw.objectRange;
        } else {
            return Math.abs(m_claw.getEncoderPosition() - kClaw.openPosition) <= kClaw.encoderOffset;
        }
    }

}