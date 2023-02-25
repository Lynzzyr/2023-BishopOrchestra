package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase {

    private final Claw m_claw;
    private final boolean isAuto;
    private final double position;

    private boolean hasClosed = false;

    private int stallTimer = 0;

    public CloseClaw(Claw claw, boolean auto, double pos) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;
        isAuto = auto;
        position = pos;

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stallTimer = 0;
        hasClosed = false;
        if (!isAuto) {
            m_claw.clawGoTo(position);
        }
    }

    @Override
    public void execute() {
        if (isAuto) {
            if (m_claw.getDistanceFromClaw() <= kClaw.objectRange && m_claw.getDistanceFromClaw() != 0) {
                if (!hasClosed) {
                    m_claw.clawGoTo(position);
                    if (m_claw.isStalled()) {
                        stallTimer++;
                        if (stallTimer == kClaw.stallTime) {
                            hasClosed = true;
                        }
                    }
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_claw.stopMot();
    }

    @Override 
    public boolean isFinished() {
        if (!isAuto) {
            return Math.abs(m_claw.getEncoderPosition() - position) <= kClaw.encoderOffset || m_claw.isStalled();
        } else {
            return hasClosed && (Math.abs(m_claw.getEncoderPosition() - position) <= kClaw.encoderOffset || m_claw.isStalled());
        }
    }

}