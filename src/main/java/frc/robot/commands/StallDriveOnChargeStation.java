package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kBalancing;
import frc.robot.Constants.kStallDriveOnChargeStation;
import frc.robot.subsystems.Drivetrain;

/**
 * This command will stall the robot's drive motors
 * on the charge station, when the robot is on an angle.
 */
public class StallDriveOnChargeStation extends CommandBase {

    private final Drivetrain m_drivetrain;

    public StallDriveOnChargeStation(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // m_drivetrain.setPLoop(kP);
    }

    @Override
    public void execute() {
        if (Math.abs(m_drivetrain.getPitch()) < kBalancing.angleTolerance)
            m_drivetrain.arcadeDrive(0, 0);
        else if (m_drivetrain.getPitch() > 0)
            m_drivetrain.arcadeDrive(kStallDriveOnChargeStation.kBackwardSpeed, 0);
        else if (m_drivetrain.getPitch() < 0)
            m_drivetrain.arcadeDrive(kStallDriveOnChargeStation.kForwardSpeed, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
