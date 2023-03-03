package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TelescopeTo;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Telescope;

public class ArmToPos extends SequentialCommandGroup {

    public ArmToPos(
        Telescope sys_telescope, 
        ArmPIDSubsystem sys_arm, 
        double armPosition, 
        double telescopePos
    ) {

        super(
            new RotateArmGroup(sys_telescope, sys_arm, armPosition),
            new TelescopeTo(sys_telescope, telescopePos)
        );
    }
}