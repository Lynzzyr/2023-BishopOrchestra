// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kTelescope;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.Constants.kCANdle.LEDColorType;
import frc.robot.commands.LEDs.BlinkLEDs;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NewClaw;
import frc.robot.subsystems.Telescope;

public class OneConeOnePickupConeAuto extends SequentialCommandGroup {

    public OneConeOnePickupConeAuto(
            Drivetrain sys_drivetrain,
            ArmPIDSubsystem sys_armPIDSubsystem,
            Telescope sys_telescope,
            NewClaw sys_claw,
            Candle sys_LEDs,
            List<PathPlannerTrajectory> pathGroup) {

        addCommands(
                Commands.runOnce(() -> sys_drivetrain.resetOdometry(pathGroup.get(0).getInitialPose())), // Reset odometry

                new PlaceConeOnMidAtStart(sys_armPIDSubsystem, sys_telescope, sys_claw),
                Commands.waitSeconds(1),

                new AutoPathPlanning(sys_drivetrain, pathGroup.get(0))
                    .alongWith(
                        // Ready to grab cone
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kAutoGroundBack)
                    ),
                //blinks the LEDs
                Commands.runOnce(
                    () -> sys_LEDs.setAnimation(
                        AnimationTypes.Static,
                        kCANdle.kColors.cone[0],
                        kCANdle.kColors.cone[1],
                        kCANdle.kColors.cone[2],
                        LEDColorType.Cone
                    )
                ).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(0.05),
                        new BlinkLEDs(sys_LEDs, 255, 0, 0, kCANdle.kColors.blinkSpeed, kCANdle.kColors.blinkTime)
                    )
                //closes the claw
                ).alongWith(
                    new ClawMovement(sys_claw, kClaw.coneClosePosition).withTimeout(1)
                ),

                // Drive to charge station
                new AutoPathPlanning(sys_drivetrain, pathGroup.get(1))
                    .alongWith(
                        // Cone grabbed
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted).withTimeout(0.5),
                        new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kAutoDrivingWithCone).withTimeout(1)
                    ),

                new BalancingChargeStation(sys_drivetrain, sys_LEDs)
        );
    }
}
