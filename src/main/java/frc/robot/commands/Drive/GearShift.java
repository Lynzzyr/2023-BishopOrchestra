package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.kDrivetrain.kDriveteam;
import frc.robot.Constants.kDrivetrain.kDriveteam.GearState;
import frc.robot.subsystems.Drivetrain;

public class GearShift extends InstantCommand {

    private final double forwardSpeed;
    private final double turningSpeed;

    private final Drivetrain m_drivetrain;
    private final GearState m_gearState;

    /**
     * Code gear shifting to increase, decrease speed
     * @param gearState kSlow, kDefault, kBoost
     * @param drivetrain Drivetrain subsystem
     */

    public GearShift(GearState gearState, Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.

        m_gearState = gearState;

        
        switch(m_gearState) {
            case kSlow:
                forwardSpeed = kDriveteam.slowSpeed;
                this.turningSpeed = kDriveteam.slowTurn;
                break;
            case kDefault:
                forwardSpeed = kDriveteam.defaultSpeedMultiplier;
                this.turningSpeed = kDriveteam.defaultTurningMultiplier;
                break;
            case kBoost:
                forwardSpeed = kDriveteam.boostSpeed;
                this.turningSpeed = kDriveteam.boostTurningSpeed;
                break;
            default:
                forwardSpeed = kDriveteam.defaultSpeedMultiplier;
                this.turningSpeed = kDriveteam.defaultTurningMultiplier;
                break;
        }

        m_drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //sets the speed
        m_drivetrain.setSpeed(forwardSpeed, turningSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

}
