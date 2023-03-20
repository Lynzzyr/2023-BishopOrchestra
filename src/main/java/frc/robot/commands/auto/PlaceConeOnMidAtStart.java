// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kClaw;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.NewClaw;
import frc.robot.subsystems.Telescope;

public class PlaceConeOnMidAtStart extends SequentialCommandGroup {
    
    public PlaceConeOnMidAtStart(
            ArmPIDSubsystem sys_armPIDSubsystem,
            Telescope sys_telescope,
            NewClaw sys_claw) {
                
        addCommands(
                new ClawMovement(sys_claw, kClaw.coneClosePosition).withTimeout(1),
                new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kToMid).withTimeout(1),
                new ClawMovement(sys_claw, kClaw.openPosition).withTimeout(1),
                new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kBalancing).withTimeout(0.1)
        );
    }
}
