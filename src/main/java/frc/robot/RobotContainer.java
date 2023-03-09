// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.Constants.kDrivetrain.kDriveteam.GearState;
import frc.robot.Constants.kIntake.kSetpoints.kPivotSetpoints;
import frc.robot.Constants.kIntake.kSetpoints.kWristSetpoints;
import frc.robot.Constants.kOperator;
import frc.robot.Constants.kTelescope;
import frc.robot.Constants.kTrajectoryPath;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.commands.AutoCloseClaw;
import frc.robot.commands.ClawMovement;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.GearShift;
import frc.robot.commands.Intake.IntakeHandoffSequence;
import frc.robot.commands.Intake.IntakePickupSequence;
import frc.robot.commands.Intake.PivotMove;
import frc.robot.commands.Intake.RollerMove;
import frc.robot.commands.Intake.WristMove;
import frc.robot.commands.Intake.Manual.PivotManualMove;
import frc.robot.commands.arm.MoveArmManual;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.commands.auto.MidConeAuto;
import frc.robot.commands.claw.AutoCloseClaw;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.commands.sequencing.ArmToSubstation;
import frc.robot.commands.sequencing.ArmToTopCube;
import frc.robot.commands.sequencing.RotateArmGroup;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NewClaw;
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
    // private final Limelight sys_limelight;
    // public final Claw sys_claw;
    public final NewClaw sys_claw;
    public final Candle sys_candle;
    public final ArmPIDSubsystem sys_armPIDSubsystem;
    public final Telescope sys_telescope;
    public final IntakePivot sys_intakePivot;
    public final IntakeWrist sys_intakeWrist;
    public final IntakeRoller sys_intakeRoller;

    // Commands
    private final DefaultDrive cmd_defaultDrive;
    private final PivotManualMove cmd_pivotManualUp;
    private final PivotManualMove cmd_pivotManualDown;
    // private final ConeNodeAim cmd_coneNodeAim;
    private final PivotMove cmd_pivotTestA;
    private final PivotMove cmd_pivotTestB;

    // Sequential commands
    private final IntakePickupSequence seq_intakePickup;
    private final IntakeHandoffSequence seq_intakeHandoff;

    // Gear shifting
    private final GearShift cmd_lowSpeed;
    private final GearShift cmd_midSpeed;
    private final GearShift cmd_highSpeed;

    // Trajectory & autonomous path chooser
    private PathPlannerTrajectory[] m_paths = new PathPlannerTrajectory[kTrajectoryPath.paths.length];
    private ShuffleboardTab sb_driveteam;
    private SendableChooser<PathPlannerTrajectory> sc_choosePath;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Driver controllers
        joystickMain = new CommandXboxController(kOperator.port_joystickMain);
        joystickSecondary = new CommandXboxController(kOperator.port_joystickSecondary);

        // Subsystems
        sys_drivetrain = new Drivetrain();
        sys_intakePivot = new IntakePivot();
        sys_intakeWrist = new IntakeWrist();
        sys_intakeRoller = new IntakeRoller();

        // Sequential commands
        seq_intakePickup = new IntakePickupSequence(sys_intakePivot, sys_intakeWrist, sys_intakeRoller);
        seq_intakeHandoff = new IntakeHandoffSequence(sys_intakePivot, sys_intakeWrist, sys_intakeRoller);
        
        sys_claw = new NewClaw();
        sys_candle = new Candle();
        sys_armPIDSubsystem = new ArmPIDSubsystem();
        sys_telescope = new Telescope();

        // Commands
        cmd_defaultDrive = new DefaultDrive(sys_drivetrain, joystickMain);

        // Trajectory & autonomous path chooser
        PathConstraints pathConstraints = new PathConstraints(kAuto.kMaxSpeed, kAuto.kMaxAcceleration);

        for (int i = 0; i < m_paths.length; i++) {
            m_paths[i] = PathPlanner.loadPath(kTrajectoryPath.paths[i], pathConstraints, true);
        }

        sb_driveteam = Shuffleboard.getTab("Drive Team");
        sc_choosePath = new SendableChooser<PathPlannerTrajectory>();
        sc_choosePath.setDefaultOption(kTrajectoryPath.paths[0], m_paths[0]);
        for (int i = 0; i < m_paths.length; i++) {
            sc_choosePath.addOption(kTrajectoryPath.paths[i], m_paths[i]);
        }
        sb_driveteam.add("Auto path", sc_choosePath)
            .withSize(3, 1);
        
        cmd_lowSpeed = new GearShift(GearState.kSlow, sys_drivetrain);
        cmd_midSpeed = new GearShift(GearState.kDefault, sys_drivetrain);
        cmd_highSpeed = new GearShift(GearState.kBoost, sys_drivetrain);
        cmd_pivotManualUp = new PivotManualMove(sys_intakePivot, 3);
        cmd_pivotManualDown = new PivotManualMove(sys_intakePivot, -3);
        // sys_limelight = new Limelight(joystickMain);
        // cmd_coneNodeAim = new ConeNodeAim(sys_limelight, sys_drivetrain, joystickMain);
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

        joystickMain.x()
            .whileTrue(
                Commands.either(
                    new TelescopeTo(sys_telescope, kTelescope.kDestinations.kGroundBack),
                    new WaitCommand(0), 
                    () -> sys_armPIDSubsystem.getPrevPos() == kArmSubsystem.kSetpoints.kBalancing)
                .andThen(new AutoCloseClaw(sys_claw, kClaw.coneClosePosition, kClaw.coneDistanceThreshold))
            )
            .onFalse(
                new InstantCommand(() -> sys_claw.disable())
                .andThen(new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted))
            );

        joystickMain.y()
            .whileTrue(
                Commands.either(
                    new AutoCloseClaw(sys_claw, kClaw.cubeClosePosition, kClaw.coneDistanceThreshold),
                    new AutoCloseClaw(sys_claw, kClaw.cubeClosePosition, kClaw.cubeDistanceThreshold), 
                    () -> sys_armPIDSubsystem.getPrevPos() == kArmSubsystem.kSetpoints.kBalancing)
            )
            .onFalse(
                new InstantCommand(() -> sys_claw.disable())
            );

        joystickMain.a()
            .onTrue(new ClawMovement(sys_claw, kClaw.openPosition).withTimeout(kClaw.timeout));
        

        joystickMain.leftBumper()
            .onTrue(cmd_lowSpeed)
            .onFalse(cmd_midSpeed);

        joystickMain.rightBumper()
            .onTrue(cmd_highSpeed)
            .onFalse(cmd_midSpeed);

        joystickMain.povDown()
            .onTrue(
                new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotExtended)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(new WristMove(sys_intakeWrist, kWristSetpoints.kWristPickup))
                .andThen(new RollerMove(sys_intakeRoller, kIntake.kRollerInVolts))
            )

            .onFalse(
                new WristMove(sys_intakeWrist, kWristSetpoints.kWristHandoff)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotHugging))
        );
        
        joystickMain.povRight()
            .onTrue(
                new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotExtended)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(new WristMove(sys_intakeWrist, kWristSetpoints.kWristPickup))
                .andThen(new RollerMove(sys_intakeRoller, kIntake.kRollerReverseVolts))
            )
            
            .onFalse(
                new WristMove(sys_intakeWrist, kWristSetpoints.kWristHandoff)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotStoring))
                .andThen(Commands.waitSeconds(1))
            );

        /*--------------------------------------------------------------------------------------*/

        joystickSecondary.povUp()
            .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kExtended));
        joystickSecondary.povDown()
            .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kRetracted));
        
        joystickSecondary.y()
        .onTrue(
            new ArmToTopCube(
                sys_armPIDSubsystem, 
                sys_telescope
            )
        );
                
        joystickSecondary.x()
            .onTrue(
                new RotateArmGroup(
                    sys_telescope, 
                    sys_armPIDSubsystem, 
                    kArmSubsystem.kSetpoints.kConeMid
                )
            );

        joystickSecondary.b()
            .onTrue(
                new RotateArmGroup(
                    sys_telescope, 
                    sys_armPIDSubsystem, 
                    kArmSubsystem.kSetpoints.kConeAbove
                )
            );

        joystickSecondary.a()
            .onTrue(
                new RotateArmGroup(
                    sys_telescope, 
                    sys_armPIDSubsystem, 
                    kArmSubsystem.kSetpoints.kToMid
                )
            );

        joystickSecondary.rightBumper()
            .onTrue(
                new ArmToSubstation(sys_armPIDSubsystem, sys_telescope, sys_claw)
            ); // pickup from loading station
            
            joystickSecondary.leftBumper()
            .onTrue(
                new RotateArmGroup(
                    sys_telescope, 
                    sys_armPIDSubsystem, 
                    kArmSubsystem.kSetpoints.kIdling
                    )
                );
                    
            joystickSecondary.back()
                .onTrue(
                    new RotateArmGroup(
                        sys_telescope, 
                        sys_armPIDSubsystem, 
                        kArmSubsystem.kSetpoints.kBalancing
                    )
                );
                
            joystickSecondary.start()
                .onTrue(
                    new RotateArmGroup(
                        sys_telescope, 
                        sys_armPIDSubsystem, 
                        kArmSubsystem.kSetpoints.kToLoadingRamp
                    )
                );
                    
        joystickSecondary.rightTrigger()
            .whileTrue(new MoveArmManual(sys_armPIDSubsystem, kArmSubsystem.kVoltageManual));

        joystickSecondary.leftTrigger()
            .whileTrue(new MoveArmManual(sys_armPIDSubsystem, -kArmSubsystem.kVoltageManual));             

        joystickSecondary.leftStick()
            .onTrue(Commands.runOnce(
                () -> sys_candle.setAnimation(
                    AnimationTypes.Static,
                    kCANdle.kColors.cone[0],
                    kCANdle.kColors.cone[1],
                    kCANdle.kColors.cone[2]
                )
            )
        );
        joystickSecondary.rightStick()
            .onTrue(Commands.runOnce(
                () -> sys_candle.setAnimation(
                    AnimationTypes.Static,
                    kCANdle.kColors.cube[0],
                    kCANdle.kColors.cube[1],
                    kCANdle.kColors.cube[2]
                )
            )
        );

        joystickSecondary.back()
            .onTrue(
                new RotateArmGroup(
                    sys_telescope, 
                    sys_armPIDSubsystem, 
                    kArmSubsystem.kSetpoints.kBalancing
                )
            );
        joystickSecondary.start()
            .onTrue(
                new RotateArmGroup(
                    sys_telescope, 
                    sys_armPIDSubsystem, 
                    kArmSubsystem.kSetpoints.kToLoadingRamp
                )
            );
                   
    }

    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous

        sys_armPIDSubsystem.disable();

        PathPlannerTrajectory chosenTrajectory = sc_choosePath.getSelected();


        // Disable ramp rate
        sys_drivetrain.rampRate(0);
        // Reset odometry
        sys_drivetrain.resetOdometry(chosenTrajectory.getInitialPose());
        // Run auto path, then stop and re-set ramp rate
        return new MidConeAuto(sys_drivetrain, sys_armPIDSubsystem, sys_telescope, sys_claw, chosenTrajectory)
            .andThen(() -> sys_drivetrain.tankDriveVoltages(0, 0))
            .andThen(() -> sys_drivetrain.rampRate(kDrivetrain.kDriveteam.rampRate));
    }

}