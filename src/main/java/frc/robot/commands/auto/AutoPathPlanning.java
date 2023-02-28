package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.subsystems.Drivetrain;

public class AutoPathPlanning extends SequentialCommandGroup {

    public AutoPathPlanning(Drivetrain sys_drivetrain, PathPlannerTrajectory trajectory) {
        super(
            new PPRamseteCommand(
                trajectory,
                sys_drivetrain::getPose2d,
                new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
                new SimpleMotorFeedforward(kDrivetrain.ksVolts, kDrivetrain.kvVolts, kDrivetrain.kaVolts),
                kDrivetrain.kDriveKinematics,
                sys_drivetrain::getWheelSpeeds,
                new PIDController(kDrivetrain.kPDriveVel, 0, 0),
                new PIDController(kDrivetrain.kPDriveVel, 0, 0),
                sys_drivetrain::tankDriveVoltages,
                sys_drivetrain)
        );
    }

}