package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NewClaw;
import frc.robot.subsystems.Telescope;

public class OneConeAuto extends SequentialCommandGroup {

    public OneConeAuto(
            Drivetrain sys_drivetrain,
            ArmPIDSubsystem sys_armPIDSubsystem,
            Telescope sys_telescope,
            NewClaw sys_claw,
            Candle sys_LEDs,
            PathPlannerTrajectory trajectory) {

        addCommands(
                Commands.runOnce(() -> sys_drivetrain.resetOdometry(trajectory.getInitialPose())), // Reset odometry

                new PlaceConeOnMidAtStart(sys_armPIDSubsystem, sys_telescope, sys_claw),
                Commands.waitSeconds(1),
                new AutoPathPlanning(sys_drivetrain, trajectory),
                new BalancingChargeStation(sys_drivetrain, sys_LEDs)
        );
    }

}