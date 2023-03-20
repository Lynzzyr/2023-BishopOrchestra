package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kTelescope;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Telescope;

public class ArmToTopCube extends CommandBase {

    private final ArmPIDSubsystem sys_arm;;
    private final Telescope sys_telescope;

    private boolean extended = false;

    public ArmToTopCube(ArmPIDSubsystem sys_arm, Telescope sys_telescope) {
        this.sys_arm = sys_arm;
        this.sys_telescope = sys_telescope;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.sys_arm, this.sys_telescope);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        extended = false;
        sys_telescope.extend(kTelescope.kDestinations.kRetracted);
        sys_arm.setSetpoint(kArmSubsystem.kSetpoints.kToTop);
        sys_arm.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!extended && Math.abs(sys_arm.getSetpoint() - sys_arm.getMeasurement()) < 0.10) {
            extended = true;
            sys_telescope.extend(kTelescope.kDestinations.kExtended);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        extended = false;
        sys_arm.setPrevPos(kArmSubsystem.kSetpoints.kToTop);
        sys_telescope.setPrevPos(kTelescope.kDestinations.kExtended);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(sys_telescope.getDistance() - kTelescope.kDestinations.kExtended) < 0.2;
    }

}