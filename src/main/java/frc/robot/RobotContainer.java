// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kArmSubsystem;
import frc.robot.Constants.kAutoRoutines.kConePlacePickupPlaceAuto;
import frc.robot.Constants.kAutoRoutines.kOneConeAuto;
import frc.robot.Constants.kAutoRoutines.kOneConeOnePickup;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.Constants.kCANdle.LEDColorType;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.Constants.kDrivetrain.kDriveteam.GearState;
import frc.robot.Constants.kIntake.kSetpoints.kPivotSetpoints;
import frc.robot.Constants.kOperator;
import frc.robot.Constants.kTelescope;
import frc.robot.commands.StallDriveOnChargeStation;
import frc.robot.commands.LEDs.BlinkLEDs;
import frc.robot.commands.arm.MoveAndRetract;
import frc.robot.commands.arm.MoveArmManual;
import frc.robot.commands.arm.MoveThenExtend;
import frc.robot.commands.arm.TelescopeTo;
import frc.robot.commands.auto.ConePlacePickupPlaceAuto;
import frc.robot.commands.auto.OneConeAuto;
import frc.robot.commands.auto.OneConeOnePickupConeAuto;
import frc.robot.commands.claw.AutoCloseClaw;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.commands.claw.DetectGamepiece;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.GearShift;
import frc.robot.commands.intake.IntakeHandoffSequence;
import frc.robot.commands.intake.IntakePickupSequence;
import frc.robot.commands.intake.PivotMove;
import frc.robot.commands.intake.manual.PivotManualMove;
import frc.robot.commands.vision.ConeNodeAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeWrist;


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
    private final Limelight sys_limelight;
    // public final Claw sys_claw;
    public final Claw sys_claw;
    public final Candle sys_candle;
    public final Arm sys_arm;
    public final Telescope sys_telescope;
    public final IntakePivot sys_intakePivot;
    public final IntakeWrist sys_intakeWrist;
    public final IntakeRoller sys_intakeRoller;
    private final UsbCamera sys_camera;

    // Commands
    private final DefaultDrive cmd_defaultDrive;
    private final PivotManualMove cmd_pivotManualUp;
    private final PivotManualMove cmd_pivotManualDown;
    private final ConeNodeAim cmd_coneNodeAim;
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
    private ShuffleboardTab sb_driveteam;
    private SendableChooser<Command> sc_chooseAutoRoutine;

    private int rumbleTime = 0;

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
        sys_claw = new Claw();
        sys_candle = new Candle();
        sys_arm = new Arm();
        sys_telescope = new Telescope();
        // sys_limelight = new Limelight(joystickMain);

        // Sequential commands
        seq_intakePickup = new IntakePickupSequence(sys_intakePivot, sys_intakeWrist, sys_intakeRoller);
        seq_intakeHandoff = new IntakeHandoffSequence(sys_intakePivot, sys_intakeWrist, sys_intakeRoller);

        // Commands
        cmd_defaultDrive = new DefaultDrive(sys_drivetrain, joystickMain);        
        cmd_lowSpeed = new GearShift(GearState.kSlow, sys_drivetrain);
        cmd_midSpeed = new GearShift(GearState.kDefault, sys_drivetrain);
        cmd_highSpeed = new GearShift(GearState.kBoost, sys_drivetrain);
        cmd_pivotManualUp = new PivotManualMove(sys_intakePivot, 3);
        cmd_pivotManualDown = new PivotManualMove(sys_intakePivot, -3);
        sys_limelight = new Limelight(joystickMain);
        cmd_coneNodeAim = new ConeNodeAim(sys_limelight, sys_telescope, sys_drivetrain, joystickMain);
        cmd_pivotTestA = new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotTestA);
        cmd_pivotTestB = new PivotMove(sys_intakePivot, kPivotSetpoints.kPivotTestB);

        // Set default drive as drivetrain's default command
        sys_drivetrain.setDefaultCommand(cmd_defaultDrive);

        // Add auto routines to Shuffleboard
        sb_driveteam = Shuffleboard.getTab("Drive team");

        //putting camera format on shuffleboard
        sb_driveteam.addInteger("Camera FPS", () -> 10).withPosition(0, 2);
        sb_driveteam.addInteger("Camera Width", () -> 1024).withPosition(1, 2);
        sb_driveteam.addInteger("Camera Height", () -> 768).withPosition(2, 2);

        addAutoRoutinesToShuffleboard();

        // Camera
        sys_camera = CameraServer.startAutomaticCapture();
        configCamera();

        // Configure the trigger bindings
        configureBindings();
    }

    private void configCamera() {
        /*
        Camera model: HBVCAM-3M2111 V22
        Link (contains some useful info): https://www.amazon.com/Camera-Module-HBVCAM-3M2111-Distance-Recognition/dp/B09NB2ZYW5

        Supported resolutions:

        MJPEG 320x240 30FPS, YUY2 320x240 30FPS
        MJPEG 352x288 30FPS, YUY2 352x288 30FPS
        MJPEG 640x480 20FPS, YUY2 640x480 8FPS
        MJPEG 1280x720 20FPS, YUY2 1280x720 8FPS
        MJPEG 1920x1080 20FPS, YUY2 1920x1080 5FPS
        MJPEG 2048x1536 15FPS, YUY2 2048x1536 5FPS
         */
        // int[] cam_defaultRes = { 2048, 1536 };
        // int cam_width = (int)(cam_defaultRes[0] / 2);
        // int cam_height = (int)(cam_defaultRes[1] / 2);
        int cam_fps = 10;

        int cam_width = 1024;
        int cam_height = 768;

        // System.out.println(sys_camera.getVideoMode().width);
        // System.out.println(sys_camera.getVideoMode().height);

        try {
            sys_camera.setVideoMode(sys_camera.getVideoMode().pixelFormat, cam_width, cam_height, cam_fps);
            sys_camera.setFPS(cam_fps);
            sys_camera.setResolution(cam_width, cam_height);
            // System.out.println(sys_camera.getVideoMode().width);
            // System.out.println(sys_camera.getVideoMode().height);
            // sys_camera.setResolution(1, 1);
        } catch (VideoException ve) {
            ve.printStackTrace();
        }
    }

    /**
     * Add the auto routines selection to Shuffleboard.
     * 
     * This MUST be called in the RobotContainer constructor.
     */
    private void addAutoRoutinesToShuffleboard() {
        // Trajectory & autonomous path chooser
        sc_chooseAutoRoutine = new SendableChooser<Command>();

        for (String pathName : kOneConeOnePickup.all) {
            List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, kAuto.kMaxSpeed, kAuto.kMaxAcceleration, true);
            OneConeOnePickupConeAuto autoCommand = new OneConeOnePickupConeAuto(sys_drivetrain, sys_arm, sys_telescope, sys_claw, sys_candle, pathGroup);
            sc_chooseAutoRoutine.addOption(pathName, autoCommand);
        }
        for (String pathName : kOneConeAuto.all) {
            PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, kAuto.kMaxSpeed, kAuto.kMaxAcceleration, true);
            OneConeAuto autoCommand = new OneConeAuto(sys_drivetrain, sys_arm, sys_telescope, sys_claw, sys_candle, trajectory);
            sc_chooseAutoRoutine.addOption(pathName, autoCommand);
        }
        for (String pathName : kConePlacePickupPlaceAuto.all) {
            List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, kAuto.kMaxSpeed+1, kAuto.kMaxAcceleration, true);
            ConePlacePickupPlaceAuto autoCommand = new ConePlacePickupPlaceAuto(sys_drivetrain, sys_arm, sys_telescope, sys_claw, sys_candle, sys_limelight, pathGroup);
            sc_chooseAutoRoutine.addOption(pathName, autoCommand);
        }

        sb_driveteam.add("Choose auto routine", sc_chooseAutoRoutine)
            .withPosition(0, 0)
            .withSize(3, 1);
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

        // Auto-close claw for cone
        joystickMain.x()
            .whileTrue(
                new ConditionalCommand(
                    new ClawMovement(sys_claw, kClaw.armedOpenPosition),
                    new ClawMovement(sys_claw, kClaw.armedDoublePosition),
                    () -> sys_arm.getController().getSetpoint() == kArmSubsystem.kSetpoints.kGroundPickupCone
                )
                .andThen(
                    new ConditionalCommand(
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kGroundBack),
                        new WaitCommand(0), 
                        () -> sys_arm.getController().getSetpoint() == kArmSubsystem.kSetpoints.kGroundPickupCone
                    )
                )
                .andThen(
                    new ConditionalCommand(
                        new AutoCloseClaw(sys_claw, kClaw.coneClosePosition, kClaw.coneDistanceThreshold),
                        new AutoCloseClaw(sys_claw, kClaw.coneClosePosition, kClaw.doubleDistanceThreshold),
                        () -> sys_arm.getController().getSetpoint() == kArmSubsystem.kSetpoints.kGroundPickupCone
                    )
                    .andThen(new WaitCommand(0.4))
                    .andThen(new DetectGamepiece(sys_claw, joystickMain, joystickSecondary, false))
                )
            )
            .onFalse(
                new SequentialCommandGroup(
                    new InstantCommand(() -> sys_claw.disable()),
                    new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted)
                )
            );

        // Auto-close claw for cube
        joystickMain.y()
            .whileTrue(
                new ClawMovement(sys_claw, kClaw.openPosition)
                .andThen(
                    new ConditionalCommand(
                        new TelescopeTo(sys_telescope, kTelescope.kDestinations.kCubeGround)
                        .alongWith(new AutoCloseClaw(sys_claw, kClaw.coneClosePosition, kClaw.coneDistanceThreshold)),
                        new AutoCloseClaw(sys_claw, kClaw.cubeClosePosition, kClaw.cubeDistanceThreshold), 
                        () -> sys_arm.getController().getSetpoint() == kArmSubsystem.kSetpoints.kGroundPickupCone
                        )
                    )
                .andThen(new WaitCommand(0.5))
                .andThen(new DetectGamepiece(sys_claw, joystickMain, joystickSecondary, true))
            )
            .onFalse(
                new SequentialCommandGroup(
                    new InstantCommand(() -> sys_claw.disable()),
                    new TelescopeTo(sys_telescope, kTelescope.kDestinations.kRetracted)
                )
            );
        
        // Open claw to armed position
        joystickMain.b()
            .onTrue(
                new ClawMovement(sys_claw, kClaw.armedOpenPosition)
            );

        // Open claw
        joystickMain.a()
            .onTrue(new ClawMovement(sys_claw, kClaw.openPosition).withTimeout(kClaw.timeout));

        // Limelight: cone node aim
        joystickMain.leftBumper()
            .whileTrue(cmd_coneNodeAim);

        // Gear shifting (high-mid)
        joystickMain.rightBumper()
            .onTrue(cmd_highSpeed)
            .onFalse(cmd_midSpeed);

        // Manual claw movement, open
        joystickMain.povUp()
            .onTrue(Commands.runOnce(() -> sys_claw.setSpeed(kClaw.manualMovementSpeed)))
            .onFalse(Commands.runOnce(() -> sys_claw.stopMotor()));
        // Manual claw movement, close
        joystickMain.povDown()
            .onTrue(Commands.runOnce(() -> sys_claw.setSpeed(-kClaw.manualMovementSpeed)))
            .onFalse(Commands.runOnce(() -> sys_claw.stopMotor()));

        // Stall motors on charge station
        joystickMain.start()
            .whileTrue(new StallDriveOnChargeStation(sys_drivetrain))
            .onFalse(Commands.runOnce(() -> sys_drivetrain.arcadeDrive(0, 0)));

        /*--------------------------------------------------------------------------------------*/

        // Manual telescope movement
        joystickSecondary.povUp()
            .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kExtended));
            
        joystickSecondary.povDown()
            .onTrue(new TelescopeTo(sys_telescope, Constants.kTelescope.kDestinations.kRetracted));
        
        // Orchestra controls
        joystickSecondary.povLeft()
            .onTrue(Commands.runOnce(() -> sys_drivetrain.toggleMusic(), sys_drivetrain));

        joystickSecondary.povRight()
            .onTrue(Commands.runOnce(() -> sys_drivetrain.stopAndRewindMusic(), sys_drivetrain));
        
        // Move arm and extend to top cube position
        joystickSecondary.y()
            .onTrue(
                new MoveThenExtend(sys_arm, Constants.kArmSubsystem.kSetpoints.kToTop, 
                sys_telescope, Constants.kTelescope.kDestinations.kExtended)
            );

        // Move arm and retract ABOVE mid cone node position
        joystickSecondary.b()
            .onTrue(
                new MoveAndRetract(sys_arm, kArmSubsystem.kSetpoints.kConeAboveNew, sys_telescope)
            );
        
        // Move arm and retract to cone low position
        joystickSecondary.x()
            .onTrue(
                new MoveAndRetract(sys_arm, kArmSubsystem.kSetpoints.kConeLow, sys_telescope)
            );

        // Move arm and retract to mid cube position
        joystickSecondary.a()
            .onTrue(
                new MoveAndRetract(sys_arm, kArmSubsystem.kSetpoints.kToMid, sys_telescope)
            );

        // Move arm and retract to double substation
        joystickSecondary.rightBumper()
            .onTrue(
                new MoveAndRetract(sys_arm, kArmSubsystem.kSetpoints.kToLoadingshoulder, sys_telescope)
            ); // pickup from loading station
            
        // Move arm and retract to idling position
        joystickSecondary.leftBumper()
            .onTrue(
                new MoveAndRetract(sys_arm, kArmSubsystem.kSetpoints.kIdling, sys_telescope)
            );
                    
        // Move arm and retract to ground pickup (resting on intake) position
        joystickSecondary.back()
            .onTrue(
                new MoveAndRetract(sys_arm, kArmSubsystem.kSetpoints.kGroundPickupCone, sys_telescope)
            );
                    
        // Manual arm movement
        joystickSecondary.rightTrigger()
            .whileTrue(new MoveArmManual(sys_arm, kArmSubsystem.kVoltageManual).alongWith(
                new BlinkLEDs(sys_candle, 255, 255, 255, kCANdle.kColors.blinkSpeed, -1)
                )
            );
        joystickSecondary.leftTrigger()
            .whileTrue(new MoveArmManual(sys_arm, -kArmSubsystem.kVoltageManual).alongWith(
                new BlinkLEDs(sys_candle, 255, 255, 255, kCANdle.kColors.blinkSpeed, -1)
                )
            );             

        // Set LED to cone (yellow)
        joystickSecondary.leftStick()
            .onTrue(Commands.runOnce(
                () -> sys_candle.setAnimation(
                    AnimationTypes.Static,
                    kCANdle.kColors.cone[0],
                    kCANdle.kColors.cone[1],
                    kCANdle.kColors.cone[2],
                    LEDColorType.Cone
                )
            ).alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(0.05),
                    new BlinkLEDs(sys_candle, 255, 0, 0, kCANdle.kColors.blinkSpeed, kCANdle.kColors.blinkTime)
                    )
                )
            );

        // Set LED to cube (purple)
        joystickSecondary.rightStick()
            .onTrue(Commands.runOnce(
                () -> sys_candle.setAnimation(
                    AnimationTypes.Static,
                    kCANdle.kColors.cube[0],
                    kCANdle.kColors.cube[1],
                    kCANdle.kColors.cube[2],
                    LEDColorType.Cube
                )
            ).alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(0.05),
                    new BlinkLEDs(sys_candle, 255, 0, 0, kCANdle.kColors.blinkSpeed, kCANdle.kColors.blinkTime)
                    )
                )
            );

            // Set LED to red
            joystickSecondary.start()
                .onTrue(Commands.runOnce(
                    () -> sys_candle.setAnimation(
                        AnimationTypes.Static,
                        255,
                        0,
                        0
                    )
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

        sys_arm.disable();

        Command chosenAutoRoutine = sc_chooseAutoRoutine.getSelected();

        // Disable ramp rate
        sys_drivetrain.rampRate(0);

        // Run auto path, then stop and re-set ramp rate
        return chosenAutoRoutine
            .andThen(() -> sys_drivetrain.tankDriveVoltages(0, 0))
            .andThen(() -> sys_drivetrain.rampRate(kDrivetrain.kDriveteam.rampRate))
            .andThen(() -> sys_drivetrain.setNeutralMode(NeutralMode.Brake));
    }

    public void rumbleController(double value, int time) {
        rumbleTime = time;
        joystickMain.getHID().setRumble(RumbleType.kBothRumble, value);
    }

    public void updateRumble() {
        if (rumbleTime == 0) {
            joystickMain.getHID().setRumble(RumbleType.kBothRumble, 0);
            rumbleTime = -1;
        } else {
            rumbleTime--;
        }
    }

}