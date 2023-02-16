package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kDrivetrain.kDriveteam;
import frc.robot.Constants.kDrivetrain.kDriveteam.GearState;
import frc.robot.subsystems.Drivetrain;

public class GearShift extends CommandBase {

    private final double forwardSpeed;
    private final double turningSpeed;

    private final Drivetrain m_drivetrain;
    private final GearState m_gearState;

    private int timer;

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
        //sets the speed and ramprate
        m_drivetrain.setSpeed(forwardSpeed, turningSpeed);
        m_drivetrain.rampRate(kDriveteam.kChangeRamp);
        timer = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //counts down then turns the ramp rate down
        timer++;
        if (timer == kDriveteam.timerLength) {
            m_drivetrain.rampRate(kDriveteam.rampRate);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.rampRate(kDriveteam.rampRate);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer >= kDriveteam.timerLength;
    }

}
