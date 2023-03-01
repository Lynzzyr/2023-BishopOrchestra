package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakeWrist;

public class DisablePIDSubsystems extends InstantCommand {

    private final ArmPIDSubsystem sys_arm;

    private final IntakeWrist sys_wrist;
    private final IntakePivot sys_pivot;

    public DisablePIDSubsystems(IntakeWrist sys_wrist, IntakePivot sys_pivot, ArmPIDSubsystem sys_arm) {
        this.sys_arm = sys_arm;
        this.sys_wrist = sys_wrist;
        this.sys_pivot = sys_pivot;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.sys_arm, this.sys_wrist, this.sys_pivot);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_arm.disable();
        sys_pivot.disable();
        sys_wrist.disable();
    }


    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}