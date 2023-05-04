package frc.robot.commands.disabled;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeWrist;

public class DisablePIDSubsystems extends InstantCommand {

    private final Arm sys_arm;

    private final IntakeWrist sys_wrist;
    private final IntakePivot sys_pivot;

    private final Claw sys_claw;

    public DisablePIDSubsystems(IntakeWrist sys_wrist, IntakePivot sys_pivot, Arm sys_arm, Claw sys_claw) {
        this.sys_arm = sys_arm;
        this.sys_wrist = sys_wrist;
        this.sys_pivot = sys_pivot;
        this.sys_claw = sys_claw;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.sys_arm, this.sys_wrist, this.sys_pivot);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_arm.disable();
        sys_pivot.disable();
        sys_wrist.disable();
        sys_claw.disable();
    }


    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}