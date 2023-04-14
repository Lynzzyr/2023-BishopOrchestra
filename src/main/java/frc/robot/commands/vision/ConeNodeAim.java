// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.kLimelight;
import frc.robot.Constants.kLimelight.kConeNodeAim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;

public class ConeNodeAim extends CommandBase {

    private final Limelight sys_limelight;
    private final Drivetrain sys_drivetrain;
    private final Telescope sys_Telescope;
    private CommandXboxController m_joystick;

    private ShuffleboardTab sb_nodeAimTab;
    private GenericEntry nt_kP, nt_kI, nt_kD;

    private final PIDController m_pidController;

    double dirInRad, turning;
    double[] lowNodeCrop, highNodeCrop;
    double currentOffset;

    /** Creates a new ConeNodeAim. */
    public ConeNodeAim(Limelight limelight, Telescope telescope, Drivetrain drivetrain) {
        sys_limelight = limelight;
        sys_drivetrain = drivetrain;
        sys_Telescope = telescope;
        m_joystick = null;

        m_pidController = new PIDController(kLimelight.kConeNodeAim.kP, kLimelight.kConeNodeAim.kI, kLimelight.kConeNodeAim.kD);
        m_pidController.setSetpoint(0);
        m_pidController.setTolerance(kLimelight.kConeNodeAim.KretroTargetTolerance);

        // Use addRequirements() here to declare subsysstem dependencies.
        addRequirements(sys_drivetrain, sys_limelight);
    }

    /** Creates a new ConeNodeAim. */
    public ConeNodeAim(Limelight limelight, Telescope telescope, Drivetrain drivetrain, CommandXboxController joystick) {
        this(limelight, telescope, drivetrain);
        m_joystick = joystick;
    }

    public void setTargetMode(){
        lowNodeCrop = kLimelight.KretroTarget.lowNodeCrop;
        highNodeCrop = kLimelight.KretroTarget.highNodeCrop;
        if (sys_Telescope.getPrevPos() == Constants.kTelescope.kDestinations.kExtended) {
            sys_limelight.setCropSize(highNodeCrop);
            currentOffset = kConeNodeAim.KhighNodeOffset;
        } else {
            sys_limelight.setCropSize(lowNodeCrop);
            currentOffset = kConeNodeAim.KlowNodeOffset;
        }
    }

    public void getShuffleboardPID(){
        if (Constants.kLimelight.kConeNodeAim.doPIDTuning) {
            sb_nodeAimTab = Shuffleboard.getTab("Field Localization");
            nt_kP = sb_nodeAimTab.add("kP", kLimelight.kConeNodeAim.kP).getEntry();
            nt_kI = sb_nodeAimTab.add("kI", kLimelight.kConeNodeAim.kI).getEntry();
            nt_kD = sb_nodeAimTab.add("kD", kLimelight.kConeNodeAim.kD).getEntry();
            m_pidController.setPID(nt_kP.getDouble(0), nt_kI.getDouble(0), nt_kD.getDouble(0));
        }
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_limelight.turnOn();
        sys_limelight.setData("pipeline", 0);
        getShuffleboardPID();
    }

    public double getTargetSpeed(){
        double calculatedOutput;

        if (kConeNodeAim.KdoTargetOffset){
            calculatedOutput = m_pidController.calculate((sys_limelight.getXOffset() + currentOffset));
            System.out.println(sys_limelight.getXOffset() + currentOffset);
        } else {
            calculatedOutput =  m_pidController.calculate(sys_limelight.getXOffset());
        }

        if (calculatedOutput >= 0){
            calculatedOutput += Constants.kLimelight.kConeNodeAim.KretroTargetFF;
        } else if (calculatedOutput < 0){
            calculatedOutput -= Constants.kLimelight.kConeNodeAim.KretroTargetFF;
        }

        if (m_pidController.atSetpoint()){
            calculatedOutput = 0; 
        }

        //debugging
        if (Constants.kLimelight.kConeNodeAim.debugMode){
            System.out.println(calculatedOutput);
        }

        return calculatedOutput;
    }

    public double getXSpeed(){
        double speed;
        if (m_joystick != null){
            speed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();   
        } else {
            speed = 0;
        }

        return speed;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        setTargetMode();
        sys_drivetrain.autoTurnDrive(getXSpeed(), getTargetSpeed());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sys_drivetrain.arcadeDrive(0, 0);
        sys_limelight.turnOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}