package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase {

    private final Claw m_claw;
    private final double position;

    private int stallTimer = 0;

    public CloseClaw(Claw claw, double pos) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;
        position = pos;

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stallTimer = 0;
        m_claw.clawGoTo(position);
    }

    @Override
    public void execute() {
        if (m_claw.isStalled()) {
            stallTimer++;
        } else {
            stallTimer = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_claw.stopMot();
    }

    @Override 
    public boolean isFinished() {
        return Math.abs(m_claw.getEncoderPosition() - position) <= kClaw.encoderOffset || stallTimer >= kClaw.stallTime;
    }

}