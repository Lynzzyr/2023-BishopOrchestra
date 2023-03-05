package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kTelescope;
import frc.robot.commands.ArmRotation;
import frc.robot.commands.ClawMovement;
import frc.robot.commands.TelescopeTo;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NewClaw;
import frc.robot.subsystems.Telescope;

public class MidConeAuto extends SequentialCommandGroup {

    public MidConeAuto(
                Drivetrain sys_drivetrain,
                ArmPIDSubsystem sys_armPIDSubsystem,
                Telescope sys_telescope,
                NewClaw sys_claw,
                PathPlannerTrajectory trajectory) {
        super(
            new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted),
            new ClawMovement(sys_claw, kClaw.coneClosePosition).withTimeout(1),
            new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kToMid),
            Commands.waitSeconds(0.5),
            new ClawMovement(sys_claw, kClaw.openPosition),
            new ArmRotation(sys_armPIDSubsystem, kArmSubsystem.kSetpoints.kRestingOnIntake).withTimeout(1),
            new AutoPathPlanning(sys_drivetrain, trajectory),
            new BalancingChargeStation(sys_drivetrain)
        );
    }

}