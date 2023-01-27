package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.subsystems.Drivetrain;

public class AutoPathPlanning extends SequentialCommandGroup {

    private final static DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                kDrivetrain.ksVolts,
                kDrivetrain.kvVolts,
                kDrivetrain.kaVolts),
            kDrivetrain.kDriveKinematics,
            kDrivetrain.kAuto.kMaxVolts);

    private final static TrajectoryConfig config =
        new TrajectoryConfig(
                kDrivetrain.kAuto.kMaxSpeed,
                kDrivetrain.kAuto.kMaxAcceleration)
            .setKinematics(kDrivetrain.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        
    private final static Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );

    public AutoPathPlanning(Drivetrain sys_drivetrain, Trajectory trajectory) {
        super(
            new RamseteCommand(
                exampleTrajectory,
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

    public Trajectory getTrajectory() {
        return exampleTrajectory;
    }

}