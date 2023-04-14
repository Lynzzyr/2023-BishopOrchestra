// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kTelescope;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.commands.auto.task.AutoPathPlanning;
import frc.robot.commands.auto.task.CloseClawInAuto;
import frc.robot.commands.auto.task.PlaceConeOnMidAtStart;
import frc.robot.commands.vision.ConeNodeAim;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NewClaw;
import frc.robot.subsystems.Telescope;

public class ConePlacePickupPlaceAuto extends SequentialCommandGroup {

    public ConePlacePickupPlaceAuto(
            Drivetrain sys_drivetrain,
            ArmPIDSubsystem sys_armPIDSubsystem,
            Telescope sys_telescope,
            NewClaw sys_claw,
            Candle sys_LEDs,
            Limelight sys_limelight,
            List<PathPlannerTrajectory> pathGroup) {

        addCommands(
                Commands.runOnce(() -> sys_drivetrain.resetOdometry(pathGroup.get(0).getInitialPose())), // Reset odometry

                new PlaceConeOnMidAtStart(sys_armPIDSubsystem, sys_telescope, sys_claw),
                Commands.waitSeconds(0.5),
                

                new AutoPathPlanning(sys_drivetrain, pathGroup.get(0))
                    .alongWith(
                        // Ready to grab cone
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kGroundBack) // USE kAutoGroundBack IF GOING OVER CHARGET STATION
                    ),

                new CloseClawInAuto(sys_claw, sys_LEDs),

                // Drive to node
                new AutoPathPlanning(sys_drivetrain, pathGroup.get(1))
                    .alongWith(
                        // Cone grabbed
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted).withTimeout(0.5),
                        new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kAutoDrivingWithCone).withTimeout(1)
                    ),

                // Place cone
                new PlaceConeOnMidAtStart(sys_armPIDSubsystem, sys_telescope, sys_claw)
                .alongWith(
                    // Lineup using Limelight
                    new ConeNodeAim(sys_limelight, sys_telescope, sys_drivetrain).withTimeout(0.75)
                ),

                // Drive to next location
                new AutoPathPlanning(sys_drivetrain, pathGroup.get(2))
                .alongWith(
                    Commands.runOnce(() -> sys_drivetrain.setNeutralMode(NeutralMode.Coast))
                )
        );
    }
}
