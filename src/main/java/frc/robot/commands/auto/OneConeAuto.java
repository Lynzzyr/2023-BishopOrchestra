package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.task.AutoPathPlanning;
import frc.robot.commands.auto.task.BalancingChargeStation;
import frc.robot.commands.auto.task.PlaceConeOnMidAtStart;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Telescope;

public class OneConeAuto extends SequentialCommandGroup {

    public OneConeAuto(
            Drivetrain sys_drivetrain,
            Arm sys_arm,
            Telescope sys_telescope,
            Claw sys_claw,
            Candle sys_LEDs,
            PathPlannerTrajectory trajectory) {

        addCommands(
                Commands.runOnce(() -> sys_drivetrain.resetOdometry(trajectory.getInitialPose())), // Reset odometry

                new PlaceConeOnMidAtStart(sys_arm, sys_telescope, sys_claw),
                Commands.waitSeconds(1),
                new AutoPathPlanning(sys_drivetrain, trajectory),
                new BalancingChargeStation(sys_drivetrain, sys_LEDs)
        );
    }

}