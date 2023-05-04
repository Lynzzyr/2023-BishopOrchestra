// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.task;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kClaw;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Telescope;

public class PlaceConeOnMidAtStart extends SequentialCommandGroup {
    
    public PlaceConeOnMidAtStart(
            Arm sys_arm,
            Telescope sys_telescope,
            Claw sys_claw) {
                
        addCommands(
                new ClawMovement(sys_claw, kClaw.coneClosePosition).withTimeout(1)
                .alongWith(
                    Commands.waitSeconds(0.3).andThen(
                    new ArmRotation(sys_arm, kArmSubsystem.kSetpoints.kToMid).withTimeout(1))
                ),
                Commands.waitSeconds(0.5),
                new ClawMovement(sys_claw, kClaw.openPosition).withTimeout(1),
                new ArmRotation(sys_arm, kArmSubsystem.kSetpoints.kGroundPickupCone).withTimeout(0.1)
        );
    }
}
