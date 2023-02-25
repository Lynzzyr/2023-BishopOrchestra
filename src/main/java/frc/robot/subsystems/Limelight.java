// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Arrays;

import javax.management.Notification;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight extends SubsystemBase {
    // networktables
    NetworkTable limelightTable;

    // shuffleboard
    private final ShuffleboardLayout localizationPos, localizationRot, localizationPipeline, localizationTarget;
    private final GenericEntry xWidget, yWidget, zWidget;
    private final GenericEntry rxWidget, ryWidget, rzWidget;
    private final GenericEntry pipelineIndexWidget, pipelineLatencyWidget;
    private final GenericEntry targetSizeWidget;

    // robot
    private double targetDistance;
    private double[] robotPos;

    // time
    private double lastLightUpdate;
    
    private double[] positionDefaults = new double[] { 0 };

    // Important NetworkTable values
    NetworkTableEntry nt_xOffset, nt_yOffset, nt_targetArea, nt_visibility, nt_ledMode, nt_crop;

    // Important variables
    double angleToTarget;
    double lowTargetDist, highTargetDist;
    double turningDir = 0;

    double  lastTick =  0;

    // Joystick for Retro-reflective
    private final CommandXboxController c_joystick;
    private final XboxController joystickMain = new XboxController(0); //temp port

    // Shuffleboard Tab and Entries
    private ShuffleboardTab sb_limelight;
    private GenericEntry xOffEntry, yOffEntry, targetAreaEntry, visibilityEntry, ledModeEntry;


    public Limelight(CommandXboxController joystick) {
        // networktables
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().startServer();
        NetworkTableInstance.getDefault().setServerTeam(5409);

        // shuffleboard
        Shuffleboard.getTab("Field Localization").add("Position", 0);
        Shuffleboard.getTab("Field Localization").add("Rotation", 0);
        Shuffleboard.getTab("Field Localization").add("Pipeline Info", 0);
        Shuffleboard.getTab("Field Localization").add("Target Size", 0);

        localizationPos = Shuffleboard.getTab("Field Localization")
                .getLayout("Position", BuiltInLayouts.kGrid)
                .withSize(1, 2);

        xWidget = localizationPos.add("X", 0).getEntry();
        yWidget = localizationPos.add("Y", 0).getEntry();
        zWidget = localizationPos.add("Z", 0).getEntry();

        localizationRot = Shuffleboard.getTab("Field Localization")
                .getLayout("Rotation", BuiltInLayouts.kGrid)
                .withSize(1, 2);

        rxWidget = localizationRot.add("rX", 0).getEntry();
        ryWidget = localizationRot.add("rY", 0).getEntry();
        rzWidget = localizationRot.add("rZ", 0).getEntry();

        localizationPipeline = Shuffleboard.getTab("Field Localization")
                .getLayout("Pipeline Info", BuiltInLayouts.kGrid)
                .withSize(1, 2);

        localizationTarget = Shuffleboard.getTab("Field Localization")
                .getLayout("Pipeline Info", BuiltInLayouts.kGrid)
                .withSize(1, 1);

        pipelineIndexWidget = localizationPipeline.add("Pipeline", 0).getEntry();
        pipelineLatencyWidget = localizationPipeline.add("Latency", 0).getEntry();
        targetSizeWidget = localizationTarget.add("Size", 0).getEntry();

        // setting startup millis
        lastLightUpdate = System.currentTimeMillis();

        // {Retroreflective tape} Getting data from NetworkTables
        nt_xOffset =  limelightTable.getEntry("tx");
        nt_yOffset =  limelightTable.getEntry("ty");
        nt_targetArea =  limelightTable.getEntry("ta");
        nt_visibility =  limelightTable.getEntry("tv");
        nt_ledMode =  limelightTable.getEntry("ledMode");
        // nt_crop =  limelightTable.getEntry("crop");

        // {Retroreflective tape} Shuffleboard stuff
        sb_limelight = Shuffleboard.getTab("LimelightR");

        xOffEntry = sb_limelight.add("X Offset", nt_xOffset.getDouble(0)).getEntry();
        yOffEntry = sb_limelight.add("Y Offset", nt_yOffset.getDouble(0)).getEntry();
        targetAreaEntry = sb_limelight.add("Target Area", nt_targetArea.getDouble(-1)).getEntry();
        visibilityEntry = sb_limelight.add("Target Visibility", isVisible()).getEntry();
        ledModeEntry = sb_limelight.add("LED Mode", nt_ledMode.getDouble(-1)).getEntry();
        // cropEntry = sb_limelight.add("Crop", nt_crop.getDoubleArray(new double[] {2, 2, 2, 2})).getEntry();

        c_joystick = new CommandXboxController(0);
    }

    public void updateRobotPosition() {
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

        // get the position of the robot in 3d fieldspace as calculated by fiducial
        // targets
        robotPos = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("botpose")
                .getDoubleArray(positionDefaults); // TEMPORARY

        // updating target size data to shuffleboard
        targetDistance = LimelightHelpers.getTA("");
        targetSizeWidget.setDouble(LimelightHelpers.getTA(""));

        // updating pipeline data to shuffleboard
        pipelineIndexWidget.setDouble(LimelightHelpers.getCurrentPipelineIndex("limelight"));
        pipelineLatencyWidget.setDouble(LimelightHelpers.getLatency_Pipeline("limelight"));

        // Shuffleboard robotpos update
        // System.out.println(Arrays.toString(robotPos));
        if (robotPos.length >= 6) {
            // update Rotation and Position here
            xWidget.setDouble(robotPos[0]);
            yWidget.setDouble(robotPos[1]);
            zWidget.setDouble(robotPos[2]);

            rxWidget.setDouble(robotPos[3]);
            ryWidget.setDouble(robotPos[4]);
            rzWidget.setDouble(robotPos[5]);
        }

        //setting startup millis
        lastLightUpdate = System.currentTimeMillis();
    }
    public void autoLight() {
        // MIGHT BE EXPENSIVE ON THE CPU
        //System.out.println(System.currentTimeMillis() - lastLightUpdate);
        if (Constants.kLimelight.kDoAutoLight) {
            lastLightUpdate = System.currentTimeMillis();
            if (targetDistance >= Constants.kLimelight.kALTriggerDistance && (System.currentTimeMillis() - lastLightUpdate) >= Constants.kLimelight.kAutoLightTimeout) {
                LimelightHelpers.setLEDMode_ForceOn("");
            } else if (targetDistance <= Constants.kLimelight.kALTriggerDistance && (System.currentTimeMillis() - lastLightUpdate) >= Constants.kLimelight.kAutoLightTimeout) {
                LimelightHelpers.setLEDMode_ForceOff("");
            }
        }
    }
    
    // Retroreflective tape-related code
    /** Turns the limelight off */
    public void turnOff() {
        nt_ledMode.setNumber(1);
    }

    /** Turns the limelight on */
    public void turnOn() {
        nt_ledMode.setNumber(3);
    }

    /** Gets the X position/offset */
    public double getXOffset() {
        return nt_xOffset.getDouble(0);
    }

    /** Gets the Y position/offset */
    public double getYOffset() {
        return nt_yOffset.getDouble(0);
    }

    /** Checks if the target is visible or not */
    public boolean isVisible() {
        return nt_visibility.getDouble(0) == 1;
    }

    /** Sets data in an entry */
    public void setData(String key, double data) {
            limelightTable.getEntry(key).setDouble(data);
    }

    /** Gets the turning direction */
    public double getTurningDir() {
        return turningDir;
    }

    /** Sets the turning direction */
    public void setTurningDir(double dir) {
        turningDir = dir;
    }

    /** Gets data from an entry */
    public double getData(String key) {
        return  limelightTable.getEntry(key).getDouble(0);
    }

    public void setCropSize(double[] cropSize){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropSize);
    }

    public void dynamicCrop(char targetType, double[] targetPos){
        ;    
    }
    
    @Override
    public void periodic() {
        updateRobotPosition();
    }
}
