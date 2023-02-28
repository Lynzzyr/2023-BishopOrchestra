// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kDriveteam.GearState;
import frc.robot.Constants.kIntake.kSetpoints.kPivotSetpoints;
import frc.robot.Constants.kOperator;
import frc.robot.Constants.kClaw.kClawState;
import frc.robot.commands.ArmRotation;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.ConeNodeAim;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.GearShift;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.PivotManualMove;
import frc.robot.commands.TelescopeTo;
import frc.robot.commands.Intake.IntakeHandoffSequence;
import frc.robot.commands.Intake.IntakePickupSequence;
import frc.robot.commands.Intake.PivotMove;
import frc.robot.commands.auto.Auto;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakeRoller;
import frc.robot.subsystems.Intake.IntakeWrist;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    // Driver controllers
    private final CommandXboxController joystickMain;
    private final CommandXboxController joystickSecondary;

    // Subsystems
    public final Drivetrain sys_drivetrain;
    private final IntakePivot sys_intakePivot;
    private final IntakeWrist sys_intakeWrist;
    private final IntakeRoller sys_intakeRoller;

    // Commands
    private final DefaultDrive cmd_defaultDrive;
    private final PivotManualMove cmd_pivotManualUp;
    private final PivotManualMove cmd_pivotManualDown;
    private final Limelight sys_limelight;
    private final ConeNodeAim cmd_coneNodeAim;
    private final PivotMove cmd_pivotTestA;
    private final PivotMove cmd_pivotTestB;

    // Sequential commands
    private final IntakePickupSequence seq_intakePickup;
    private final IntakeHandoffSequence seq_intakeHandoff;
    public final Claw sys_claw;
    public final Candle sys_candle;
    public final ArmPIDSubsystem sys_ArmPIDSubsystem;
    public final Telescope sys_telescope;


    private final GearShift cmd_lowSpeed;
    private final GearShift cmd_midSpeed;
    private final GearShift cmd_highSpeed;

    // // Trajectory
    private PathPlannerTrajectory m_trajectory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(PathPlannerTrajectory trajectory) {

        // Driver controllers
        joystickMain = new CommandXboxController(kOperator.port_joystickMain);
        joystickSecondary = new CommandXboxController(kOperator.port_joystickSecondary);

        // // Trajectory paths
        m_trajectory = trajectory;

        // Subsystems
        sys_drivetrain = new Drivetrain();
        sys_intakePivot = new IntakePivot();
        sys_intakeWrist = new IntakeWrist();
        sys_intakeRoller = new IntakeRoller();

        // Sequential commands
        seq_intakePickup = new IntakePickupSequence(sys_intakePivot, sys_intakeWrist, sys_intakeRoller);
        seq_intakeHandoff = new IntakeHandoffSequence(sys_intakePivot, sys_intakeWrist, sys_intakeRoller);
        
        sys_claw = new Claw();
        sys_candle = new Candle();
        sys_ArmPIDSubsystem = new ArmPIDSubsystem();
        sys_telescope = new Telescope();

        // Commands
        cmd_defaultDrive = new DefaultDrive(sys_drivetrain, joystickMain);
        cmd_lowSpeed = new GearShift(GearState.kSlow, sys_drivetrain);
        cmd_midSpeed = new GearShift(GearState.kDefault, sys_drivetrain);
        cmd_highSpeed = new GearShift(GearState.kBoost, sys_drivetrain);
        cmd_pivotManualUp = new PivotManualMove(sys_intakePivot, 3);
        cmd_pivotManualDown = new PivotManualMove(sys_intakePivot, -3);
        sys_limelight = new Limelight(joystickMain);
        cmd_coneNodeAim = new ConeNodeAim(sys_limelight, sys_drivetrain, joystickMain);
        cmd_pivotTestA = new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotTestA);
        cmd_pivotTestB = new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotTestB);

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
    /**
     * y-intake pivot ascend
     * a-intake pivot descend
     * x-intake roller roll backwards
     * b-intake roller roll forwards */

     /*
      * starting with - 20% of 12 = 2.4
      */


    private void configureBindings() {

        joystickMain.a()
            .whileTrue(seq_intakePickup)
            .onFalse(seq_intakeHandoff);

        // joystickMain.x()
        //     .onTrue(new CloseClaw(sys_claw, kClaw.coneClosePosition))
        //     .onFalse(new OpenClaw(sys_claw, false));

        joystickMain.x()
            .onTrue(Commands.either(
                new CloseClaw(sys_claw, kClaw.coneClosePosition),
                new OpenClaw(sys_claw, false),
                () -> sys_claw.getState() == kClawState.kOpen)
            );
        
        // joystickMain.y()
        //     .onTrue(new CloseClaw(sys_claw, kClaw.cubeClosePosition))
        //     .onFalse(new OpenClaw(sys_claw, false));
        
        joystickMain.y()
            .onTrue(Commands.either(
                new CloseClaw(sys_claw, kClaw.cubeClosePosition),
                new OpenClaw(sys_claw, false),
                () -> sys_claw.getState() == kClawState.kOpen)
            );

        joystickMain.leftBumper()
            .onTrue(cmd_lowSpeed)
            .onFalse(cmd_midSpeed);
        // joystickMain.leftBumper()
        //     .toggleOnTrue(Commands.startEnd(cmd_lowSpeed, cmd_highSpeed, sys_drivetrain));

        joystickMain.rightBumper()
            .onTrue(cmd_highSpeed)
            .onFalse(cmd_midSpeed);
        
        joystickMain.povUp()
            .onTrue(cmd_pivotTestA);
            // .whileTrue(cmd_pivotManualUp);
        
        joystickMain.povDown()
            .onTrue(cmd_pivotTestB);
            // .whileTrue(cmd_pivotManualDown);

        joystickSecondary.povUp()
            .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kExtended));
        // joystickSecondary.povLeft()
            // .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kMid));
        joystickSecondary.povDown()
            .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kRetracted));

        // joystickSecondary.povUp()
        //     .onTrue(new ArmToPos(sys_telescope, sys_ArmPIDSubsystem, kArmSubsystem.kSetpoints.kToTop, kTelescope.kDestinations.kExtended));
        // joystickSecondary.povRight()
        //     .onTrue(new ArmToPos(sys_telescope, sys_ArmPIDSubsystem, kArmSubsystem.kSetpoints.kToMid, kTelescope.kDestinations.kMid));
        // joystickSecondary.povDown()
        //     .onTrue(new ArmToPos(sys_telescope, sys_ArmPIDSubsystem, kArmSubsystem.kSetpoints.kToHandoff, 0));

        joystickSecondary.x()
            .onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kToLoadingRamp)); // pickup from loading station
        joystickSecondary.b()
            .onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kToLoadingshoulder)); // pickup from floor
            joystickSecondary.a()
            .onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kToMid)); // pickup from floor
            joystickSecondary.y()
            .onTrue(new ArmRotation(sys_ArmPIDSubsystem,Constants.kArmSubsystem.kSetpoints.kToTop)); // pickup from floor
        joystickSecondary.leftBumper().onTrue(new ArmRotation(sys_ArmPIDSubsystem, Constants.kArmSubsystem.kSetpoints.kIdling));

        // joystickSecondary.x().onTrue(new RotateArmGroup(sys_telescope, sys_ArmPIDSubsystem, kArmSubsystem.kSetpoints.kfront));
        // joystickSecondary.b().onTrue(new RotateArmGroup(sys_telescope, sys_ArmPIDSubsystem, kArmSubsystem.kSetpoints.kback));

        joystickMain.b()
            .whileTrue(cmd_coneNodeAim); // Cone node auto-alignment command
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
            .andThen(() -> sys_drivetrain.rampRate(kDrivetrain.kDriveteam.rampRate));
    }

}