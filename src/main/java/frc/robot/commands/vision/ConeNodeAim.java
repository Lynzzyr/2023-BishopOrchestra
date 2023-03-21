// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.kLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ConeNodeAim extends CommandBase {

    private final Limelight sys_limelight;
    private final Drivetrain sys_drivetrain;
    private final CommandXboxController m_joystick;

    boolean debugMode = false; //---DO NOT ENABLE DURING COMP IT WILL MAKE THE ROBOT CRAAAASHHH---//

    private ShuffleboardTab sb_coneNodeAim;
    private GenericEntry nt_kP, nt_kI, nt_kD;

    private final PIDController m_pidController;

    double xSpeed, dirInRad, turning, calculatedOutput;

    /** Creates a new ConeNodeAim. */
    public ConeNodeAim(Limelight limelight, Drivetrain drivetrain, CommandXboxController joystick) {

        sys_limelight = limelight;
        sys_drivetrain = drivetrain;
        m_joystick = joystick;

        m_pidController = new PIDController(kLimelight.kConeNodeAim.kP, kLimelight.kConeNodeAim.kI, kLimelight.kConeNodeAim.kD);
        m_pidController.setSetpoint(0);
        m_pidController.setTolerance(kLimelight.kConeNodeAim.KretroTargetTolerance);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_drivetrain, sys_limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // sys_drivetrain.resetEncoders(); // IDK WHY
        sys_limelight.turnOn();
        sys_limelight.setData("pipeline", 0);
        // System.out.println("Initialized");

        if (debugMode) {
            sb_coneNodeAim = Shuffleboard.getTab("Cone node aim");
            nt_kP = sb_coneNodeAim.add("kP", kLimelight.kConeNodeAim.kP).getEntry();
            nt_kI = sb_coneNodeAim.add("kI", kLimelight.kConeNodeAim.kI).getEntry();
            nt_kD = sb_coneNodeAim.add("kD", kLimelight.kConeNodeAim.kD).getEntry();
            m_pidController.setPID(nt_kP.getDouble(0), nt_kI.getDouble(0), nt_kD.getDouble(0));
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        calculatedOutput = m_pidController.calculate(sys_limelight.getXOffset());
        xSpeed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();

        //Applying feat forward and tolerance
        if (calculatedOutput >= Constants.kLimelight.kConeNodeAim.KretroTargetTolerance){
            calculatedOutput += Constants.kLimelight.kConeNodeAim.KretroTargetFF;
        } else if (calculatedOutput < -Constants.kLimelight.kConeNodeAim.KretroTargetTolerance){
            calculatedOutput -= Constants.kLimelight.kConeNodeAim.KretroTargetFF;
        }

        sys_drivetrain.autoTurnDrive(xSpeed, calculatedOutput);

        if (debugMode){
            System.out.println(calculatedOutput);
        }


        /*
         * Older code:
         * dir = sys_limelightR.getXOffset() / Math.abs(sys_limelightR.getXOffset());
         * dir returns -1 or 1 depending on if it's positive or if it's negative
         * 
         * Since the X Offset keeps decreasing, the turning speed will decrease
         * turning = dir * Constants.kDrivetrain.kCNodeTargetSpeed;
         */

         /* if (!sys_limelight.isVisible()) {
            dirInRad = sys_limelight.getTurningDir() * (Math.PI / 180); // dir on the controller converted to radians
            sys_drivetrain.arcadeDrive(xSpeed, m_joystick.getLeftX()); // Lets the driver drive around
        } else {
            dirInRad = sys_limelight.getXOffset() * (Math.PI / 180);
            sys_drivetrain.arcadeDrive(xSpeed, (-turning));
        }

        if (dirInRad != 0) {
            turning = (Math.pow(Math.E, (Math.abs(dirInRad))) - 1);
            if (turning < kLimelight.KretroTargetFF) {
                turning = kLimelight.KretroTargetFF;
            }
            turning *= ((Math.abs(dirInRad) / dirInRad));
        } */
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
        // return (Math.abs(sys_limelight.getXOffset()) <= kLimelight.KretroTargetTolerance && sys_limelight.isVisible());
        return false;
    }
}