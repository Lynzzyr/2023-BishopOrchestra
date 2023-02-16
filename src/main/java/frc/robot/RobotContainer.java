// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.kOperator;
import frc.robot.Constants.kDrivetrain.kDriveteam;
import frc.robot.Constants.kDrivetrain.kDriveteam.GearState;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.GearShift;
import frc.robot.commands.auto.Auto;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.commands.ArmRotation;
import frc.robot.subsystems.Drivetrain;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    public final Drivetrain sys_drivetrain;
    public final ArmPIDSubsystem sys_ArmPIDSubsystem;

    // Commands
    private final DefaultDrive cmd_defaultDrive;
    
    private final GearShift cmd_lowSpeed;
    private final GearShift cmd_midSpeed;
    private final GearShift cmd_highSpeed;

    // Trajectory
    private PathPlannerTrajectory m_trajectory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(PathPlannerTrajectory trajectory) {

        // Driver controllers
        joystickMain = new CommandXboxController(kOperator.port_joystickMain);
        joystickSecondary = new CommandXboxController(kOperator.port_joystickSecondary);

        // Trajectory paths
        m_trajectory = trajectory;

        // Subsystems
        sys_drivetrain = new Drivetrain();
        sys_ArmPIDSubsystem = new ArmPIDSubsystem();

        // Commands
        cmd_defaultDrive = new DefaultDrive(sys_drivetrain, joystickMain);

        cmd_lowSpeed = new GearShift(GearState.kSlow, sys_drivetrain);
        cmd_midSpeed = new GearShift(GearState.kDefault, sys_drivetrain);
        cmd_highSpeed = new GearShift(GearState.kBoost, sys_drivetrain);

        // Set default drive as drivetrain's default command
        sys_drivetrain.setDefaultCommand(cmd_defaultDrive);

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

        joystickMain.leftBumper()
            .onTrue(cmd_lowSpeed);

        joystickMain.leftBumper()
            .onFalse(cmd_midSpeed);

        joystickMain.rightBumper()
            .onTrue(cmd_highSpeed);

        joystickMain.rightBumper()
            .onFalse(cmd_midSpeed);
        

        joystickSecondary.leftBumper()
            .onTrue(cmd_lowSpeed);
        
        joystickSecondary.leftBumper()
            .onFalse(cmd_midSpeed);


        joystickSecondary.rightBumper()
            .onTrue(cmd_highSpeed);

        joystickSecondary.rightBumper()
            .onFalse(cmd_midSpeed);

        // joystickSecondary.x().onTrue(new ArmRotation(sys_ArmPIDSubsystem, 0.55)); // intake back
        // joystickSecondary.b().onTrue(new ArmRotation(sys_ArmPIDSubsystem, -.06)); // intake front
        // joystickSecondary.y().onTrue(new ArmRotation(sys_ArmPIDSubsystem, .057)); // placement forward
        // joystickSecondary.a().onTrue(new ArmRotation(sys_ArmPIDSubsystem, 0.44)); // placement back
      //  joystickSecondary.leftBumper().onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kIdlepos));
        joystickSecondary.b().onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kfront)); // pickup from loading station
        joystickSecondary.x().onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kback)); // pickup from floor
       // joystickSecondary.y().onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kplacehigh));
       // joystickSecondary.a().onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kplacelow));
    }

    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous

        // Disable ramp rate
        sys_drivetrain.rampRate(0);
        // Reset odometry
        sys_drivetrain.resetOdometry(m_trajectory.getInitialPose());
        // Run auto path, then stop and re-set ramp rate
        return new Auto(sys_drivetrain, m_trajectory)
            .andThen(() -> sys_drivetrain.tankDriveVoltages(0, 0))

            .andThen(() -> sys_drivetrain.rampRate(kDriveteam.rampRate));
    }

}


