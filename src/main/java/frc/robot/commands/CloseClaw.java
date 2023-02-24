package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase {

    private final Claw m_claw;
    private final boolean isAuto;

    private boolean hasClosed = false;

    public CloseClaw(Claw claw, boolean auto) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_claw = claw;
        isAuto = auto;

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasClosed = false;
        if (!isAuto) {
            m_claw.closeClaw();
        }
    }

    @Override
    public void execute() {
        if (m_claw.getDistanceFromClaw() <= kClaw.objectRange) {
            if (!hasClosed) {
                m_claw.closeClaw();
                if (m_claw.isStalled()) {
                    Timer.delay(0.5);
                    hasClosed = true;
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
            return Math.abs(m_claw.getEncoderPosition() - kClaw.closePosition) <= kClaw.encoderOffset || m_claw.isStalled();
        } else {
            return hasClosed && (Math.abs(m_claw.getEncoderPosition() - kClaw.closePosition) <= kClaw.encoderOffset || m_claw.isStalled());
        }
    }

}