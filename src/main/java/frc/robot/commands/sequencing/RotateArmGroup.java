package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.kTelescope;

import frc.robot.commands.ArmRotation;
import frc.robot.commands.TelescopeTo;

import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Telescope;

public class RotateArmGroup extends ParallelCommandGroup{


    public RotateArmGroup(
        Telescope sys_telescope, 
        ArmPIDSubsystem sys_arm,  
        double armDirection
    ) {


        // Use addRequirements() here to declare subsystem dependencies.
        super(
            new ArmRotation(sys_arm, armDirection),
            new TelescopeTo(
                sys_telescope, 
                kTelescope.kDestinations.kRetracted
            )
        );
    }
}