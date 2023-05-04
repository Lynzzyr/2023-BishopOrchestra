package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.kTelescope;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class RotateArmGroup extends ParallelCommandGroup{


    public RotateArmGroup(
        Telescope sys_telescope, 
        Arm sys_arm,  
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

        addRequirements(sys_arm, sys_telescope);
    }
}