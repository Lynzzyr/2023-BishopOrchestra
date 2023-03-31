package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kCANBus;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kTelescope;
import frc.robot.subsystems.NewClaw;

public class AutoCloseClaw extends CommandBase {

    private final NewClaw m_claw;

    private final double setpoint;

    private boolean finished = false;
    private int closeDistance;

    private boolean cubeOrCone;
    private boolean groundPickup;

    public AutoCloseClaw(NewClaw claw, double setpoint, int closeDistance, boolean groundPickup, boolean cubeOrCone) {
        m_claw = claw;
        this.setpoint = setpoint;
        this.closeDistance = closeDistance;
        this.cubeOrCone = cubeOrCone;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_claw);   
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (cubeOrCone && groundPickup) {
            m_claw.setOutputLimit(kClaw.cubeOutputLimit);
        } else {
            m_claw.setOutputLimit(kClaw.coneOutputLimit);
        }
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