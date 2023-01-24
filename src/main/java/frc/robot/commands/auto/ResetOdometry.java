package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends InstantCommand {

    private final Drivetrain sys_drivetrain;
    private final Pose2d m_initialPose;

    public ResetOdometry(Drivetrain drivetrain, Pose2d initialPose) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);   
        
        sys_drivetrain = drivetrain;
        m_initialPose = initialPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_drivetrain.resetOdometry(m_initialPose);
    }
}