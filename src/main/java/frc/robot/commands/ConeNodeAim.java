// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ConeNodeAim extends CommandBase {

    private final Limelight sys_limelight;
    private final Drivetrain sys_drivetrain;
    private final CommandXboxController m_joystick;

    double forwardSpeed, dirInRad, turning;

    /** Creates a new TargetAim. */
    public ConeNodeAim(Limelight limelight, Drivetrain drivetrain, CommandXboxController joystick) {

        sys_limelight = limelight;
        sys_drivetrain = drivetrain;
        m_joystick = joystick;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_drivetrain, sys_limelight);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_limelight.turnOn();
        sys_limelight.setData("pipeline", 1);
        // System.out.println("Initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        forwardSpeed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();

        if (!sys_limelight.isVisible()) {
            dirInRad = sys_limelight.getTurningDir() * (Math.PI / 180); // dir on the controller converted to radians
            sys_drivetrain.arcadeDrive(forwardSpeed, m_joystick.getLeftX()); // Lets the driver drive around
        } else {
            dirInRad = sys_limelight.getXOffset() * (Math.PI / 180);
            sys_drivetrain.arcadeDrive(forwardSpeed, (-turning));
        }

        if (dirInRad != 0) {
            turning = (Math.pow(Math.E, (Math.abs(dirInRad))) - 1);
            if (turning < Constants.kLimelight.KretroTargetFF) {
                turning = Constants.kLimelight.KretroTargetFF;
            }
            turning *= ((Math.abs(dirInRad) / dirInRad));
        }

        /*
         * Older code:
         * dir = sys_limelightR.getXOffset() / Math.abs(sys_limelightR.getXOffset());
         * dir returns -1 or 1 depending on if it's positive or if it's negative
         * 
         * Since the X Offset keeps decreasing, the turning speed will decrease
         * turning = dir * Constants.kDrivetrain.kCNodeTargetSpeed;
         */

        // System.out.println("Turn Rate: " + turning);
        // System.out.println("Executed");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sys_drivetrain.arcadeDrive(0, 0);
        sys_limelight.turnOff();
        // System.out.println("Ended");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(sys_limelight.getXOffset()) <= Constants.kLimelight.KretroTargetTolerance && sys_limelight.isVisible());
    }
}
