// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kTelescope;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.commands.auto.task.AutoPathPlanning;
import frc.robot.commands.auto.task.BalancingChargeStation;
import frc.robot.commands.auto.task.CloseClawInAuto;
import frc.robot.commands.auto.task.PlaceConeOnMidAtStart;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Telescope;

public class OneConeOnePickupConeAuto extends SequentialCommandGroup {

    public OneConeOnePickupConeAuto(
            Drivetrain sys_drivetrain,
            Arm sys_arm,
            Telescope sys_telescope,
            Claw sys_claw,
            Candle sys_LEDs,
            List<PathPlannerTrajectory> pathGroup) {

        addCommands(
                Commands.runOnce(() -> sys_drivetrain.resetOdometry(pathGroup.get(0).getInitialPose())), // Reset odometry

                new PlaceConeOnMidAtStart(sys_arm, sys_telescope, sys_claw),
                Commands.waitSeconds(0.5),

                new AutoPathPlanning(sys_drivetrain, pathGroup.get(0))
                    .alongWith(
                        // Ready to grab cone
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kAutoGroundBack)
                    ),
                    
                new CloseClawInAuto(sys_claw, sys_LEDs),

                // Drive to charge station
                new AutoPathPlanning(sys_drivetrain, pathGroup.get(1))
                    .alongWith(
                        // Cone grabbed
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted).withTimeout(0.5),
                        new ArmRotation(sys_arm, kArmSubsystem.kSetpoints.kAutoDrivingWithCone).withTimeout(1)
                    ),

                new BalancingChargeStation(sys_drivetrain, sys_LEDs)
        );
    }
}
