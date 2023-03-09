package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kTelescope.kDestinations;
import frc.robot.commands.arm.ArmRotation;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NewClaw;
import frc.robot.subsystems.Telescope;

public class TopConeAuto extends SequentialCommandGroup {

    public TopConeAuto(
                Drivetrain sys_drivetrain,
                ArmPIDSubsystem sys_armPIDSubsystem,
                Telescope sys_telescope,
                NewClaw sys_claw,
                PathPlannerTrajectory trajectory) {
        super(
            new ClawMovement(sys_claw, kClaw.coneClosePosition).withTimeout(kClaw.timeout),
            new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kToTop),
            new TelescopeTo(sys_telescope, kDestinations.kExtended),
            Commands.waitSeconds(0.5),
            new ClawMovement(sys_claw, kClaw.openPosition),
            new TelescopeTo(sys_telescope, kDestinations.kRetracted),
            new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kRestingOnIntake),
            new AutoPathPlanning(sys_drivetrain, trajectory),
            new BalancingChargeStation(sys_drivetrain)
        );

        DriverStation.reportWarning("DO NOT RUN TOP CONE AUTO: NOT WORKING YET", null);
    }

}