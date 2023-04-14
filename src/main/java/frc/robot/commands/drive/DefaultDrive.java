package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrivetrain.kSlew;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final CommandXboxController m_controller;
    private final SlewRateLimiter m_forwardSlewLimiter;
    private final SlewRateLimiter m_sidewaysSlewLimiter;

    /**
     * Default drive command
     * 
     * Controller:
     * - right trigger: accelerate
     * - left trigger: decelerate / backwards
     * - left stick: turning
     * 
     * @param drivetrain
     * @param controller
     */
    public DefaultDrive(Drivetrain drivetrain, CommandXboxController controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        m_drivetrain = drivetrain;
        m_controller = controller;

        m_forwardSlewLimiter = new SlewRateLimiter(kSlew.kForwardSlew);
        m_sidewaysSlewLimiter = new SlewRateLimiter(kSlew.kSidewaysSlew);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        double zRotation = -m_controller.getLeftX();

        m_drivetrain.arcadeDrive(m_forwardSlewLimiter.calculate(xSpeed),
                                m_sidewaysSlewLimiter.calculate(zRotation));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}