// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kOperator;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.auto.AutoPathPlanning;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Driver controllers
    private final CommandXboxController joystickMain;
    private final CommandXboxController joystickSecondary;

    // Subsystems
    private final ExampleSubsystem sys_exampleSubsystem;
    public final Drivetrain sys_drivetrain;

    // Commands
    private final DefaultDrive cmd_defaultDrive;
    private final AutoPathPlanning cmd_autoPath;

    // Trajectory
    private Trajectory m_trajectory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Trajectory trajectory) {

        // Driver controllers
        joystickMain = new CommandXboxController(kOperator.port_joystickMain);
        joystickSecondary = new CommandXboxController(kOperator.port_joystickSecondary);

        // Subsystems
        sys_exampleSubsystem = new ExampleSubsystem();
        sys_drivetrain = new Drivetrain();

        // Commands
        cmd_defaultDrive = new DefaultDrive(sys_drivetrain, joystickMain);
        cmd_autoPath = new AutoPathPlanning(sys_drivetrain, trajectory);

        // Set default drive as drivetrain's default command
        sys_drivetrain.setDefaultCommand(cmd_defaultDrive);

        // Trajectory path
        m_trajectory = trajectory;

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous

        DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    kDrivetrain.ksVolts,
                    kDrivetrain.kvVolts,
                    kDrivetrain.kaVolts),
                kDrivetrain.kDriveKinematics,
                kDrivetrain.kAuto.kMaxVolts);

        TrajectoryConfig config =
            new TrajectoryConfig(
                    kDrivetrain.kAuto.kMaxSpeed,
                    kDrivetrain.kAuto.kMaxAcceleration)
                .setKinematics(kDrivetrain.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);
            
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),    
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config
            );
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(new Translation2d(3, 0)),
        //         new Pose2d(3, 0, new Rotation2d(0)),
        //         config
        //     );
        
        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            sys_drivetrain::getPose2d,
            new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
            new SimpleMotorFeedforward(kDrivetrain.ksVolts, kDrivetrain.kvVolts, kDrivetrain.kaVolts),
            kDrivetrain.kDriveKinematics,
            sys_drivetrain::getWheelSpeeds,
            new PIDController(kDrivetrain.kPDriveVel, 0, 0),
            new PIDController(kDrivetrain.kPDriveVel, 0, 0),
            sys_drivetrain::tankDriveVoltages,
            sys_drivetrain);

        sys_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

        return ramseteCommand.andThen(() -> sys_drivetrain.tankDriveVoltages(0, 0));
    }
}
