// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.kLimelight;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    // Networktables
    NetworkTable limelightTable;
    NetworkTableEntry nt_xOffset, nt_yOffset, nt_targetArea, nt_visibility, nt_ledMode, nt_crop;

    // Shuffleboard
    private ShuffleboardLayout localizationPos, localizationRot, localizationPipeline, localizationTarget, retroTarget = null;
    private ShuffleboardTab sb_limelight;
    private GenericEntry xOffEntry, yOffEntry, targetAreaEntry, visibilityEntry, ledModeEntry;
    private GenericEntry xWidget, yWidget, zWidget;
    private GenericEntry rxWidget, ryWidget, rzWidget;
    private GenericEntry pipelineIndexWidget, pipelineLatencyWidget;
    private GenericEntry targetSizeWidget; 
    private GenericEntry retroDistanceWidget; 

    // Fiducial
    private double targetDistance;
    private double[] robotPos;

    // Time
    private double lastLightUpdate;
    private double[] positionDefaults = new double[] { 0 };

    // Joystick for Retro-reflective
    private final CommandXboxController c_joystick;
    private final XboxController joystickMain = new XboxController(0); //temp port;

    // Retro reflective targetting
    private double retroTargetDistance;
    private double lastRetroDistance; //DEBUGGING
    double angleToTarget;
    double turningDir = 0;

    public Limelight(CommandXboxController joystick) {
        // Networktables
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().startServer();
        NetworkTableInstance.getDefault().setServerTeam(5409); 

        if (kLimelight.doShuffleboard){
            // Shuffleboard
            Shuffleboard.getTab("Field Localization").add("Position", 0);
            Shuffleboard.getTab("Field Localization").add("Rotation", 0);
            Shuffleboard.getTab("Field Localization").add("Pipeline Info", 0);
            Shuffleboard.getTab("Field Localization").add("Target Size", 0);
            Shuffleboard.getTab("Field Localization").add("Retro Distance", 0);
        
            // Robot pos
            localizationPos = Shuffleboard.getTab("Field Localization")
                .getLayout("Position", BuiltInLayouts.kGrid)
                .withSize(1, 2);

            xWidget = localizationPos.add("X", 0).getEntry();
            yWidget = localizationPos.add("Y", 0).getEntry();
            zWidget = localizationPos.add("Z", 0).getEntry();

            // Robot rotation
            localizationRot = Shuffleboard.getTab("Field Localization")
                .getLayout("Rotation", BuiltInLayouts.kGrid)
                .withSize(1, 2);

            rxWidget = localizationRot.add("rX", 0).getEntry();
            ryWidget = localizationRot.add("rY", 0).getEntry();
            rzWidget = localizationRot.add("rZ", 0).getEntry();

            // Pipeline index
            localizationPipeline = Shuffleboard.getTab("Field Localization")
                .getLayout("Pipeline Info", BuiltInLayouts.kGrid)
                .withSize(1, 2);

            pipelineIndexWidget = localizationPipeline.add("Pipeline", 0).getEntry();
            pipelineLatencyWidget = localizationPipeline.add("Latency", 0).getEntry();
        
            // Retro target size
            localizationTarget = Shuffleboard.getTab("Field Localization")
                .getLayout("Target Size", BuiltInLayouts.kGrid)
                .withSize(1, 1);

            targetSizeWidget = localizationTarget.add("Size", 0).getEntry();
        
            // Retro target Distance
            retroTarget = Shuffleboard.getTab("Field Localization")
                .getLayout("Retro Distance", BuiltInLayouts.kGrid)
                .withSize(1, 1);

            retroDistanceWidget = retroTarget.add("Distance", 0).getEntry();

            // Old shuffleboard retro-targetting data - TO BE MERGED WITH EXISTING
            sb_limelight = Shuffleboard.getTab("Limelight Retro");
            xOffEntry = sb_limelight.add("X Offset", nt_xOffset.getDouble(0)).getEntry();
            yOffEntry = sb_limelight.add("Y Offset", nt_yOffset.getDouble(0)).getEntry();
            targetAreaEntry = sb_limelight.add("Target Area", nt_targetArea.getDouble(-1)).getEntry();
            visibilityEntry = sb_limelight.add("Target Visibility", isVisible()).getEntry();
            ledModeEntry = sb_limelight.add("LED Mode", nt_ledMode.getDouble(-1)).getEntry();
        }

        // Polling retro-reflective data
        nt_xOffset =  limelightTable.getEntry("tx");        //possibly move to lime light helpers.
        nt_yOffset =  limelightTable.getEntry("ty");
        nt_targetArea =  limelightTable.getEntry("ta");
        nt_visibility =  limelightTable.getEntry("tv");
        nt_ledMode =  limelightTable.getEntry("ledMode");   // Robot rotation

        //joystick
        c_joystick = new CommandXboxController(0); // To be removed
        
        //startup time
        lastLightUpdate = System.currentTimeMillis(); // setting startup millis
    }

    public void updateRobotPosition() {
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

        // get the position of the robot in 3d fieldspace as calculated by fiducial
        robotPos = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("botpose")
                .getDoubleArray(positionDefaults); // TEMPORARY

        if (kLimelight.doShuffleboard){
            // updating target size data to shuffleboards
            targetDistance = LimelightHelpers.getTA("");
            targetSizeWidget.setDouble(LimelightHelpers.getTA(""));

            // updating pipeline data to shuffleboard
            pipelineIndexWidget.setDouble(LimelightHelpers.getCurrentPipelineIndex("limelight"));
            pipelineLatencyWidget.setDouble(LimelightHelpers.getLatency_Pipeline("limelight"));

            // Shuffleboard robotpos update
            if (robotPos.length >= 6) {
                // update Rotation and Position here
                xWidget.setDouble(robotPos[0]);
                yWidget.setDouble(robotPos[1]);
                zWidget.setDouble(robotPos[2]);

                rxWidget.setDouble(robotPos[3]);
                ryWidget.setDouble(robotPos[4]);
                rzWidget.setDouble(robotPos[5]);
            }
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

    /*--------------------------------------------------------------------------------------*/
    
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

    public void setCropSize(double[] cropSize) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropSize);
    }

    public void dynamicCrop(char targetType, double[] targetPos){
        ;    
    }

    public void updateRetroDistance() {
        double cameraTargetAngle = LimelightHelpers.getTY("");
        double realTargetAngle = Constants.kLimelight.angle + cameraTargetAngle;
        double realTargetAngleRadians = realTargetAngle * (3.14159 / 180.0); //converting angle to radians
        
        if (cameraTargetAngle != 0){ //prone to error if retro is direectly in line
            retroTargetDistance = (Constants.kLimelight.KretroTarget.lowNodeHeight - Constants.kLimelight.heightOffFloor)/Math.tan(realTargetAngleRadians); 
        } else  { 
            retroTargetDistance = 0;
            if (Constants.kLimelight.KretroTarget.retroDistanceDebug){
                if (Constants.kLimelight.KretroTarget.retroDistanceDebug){
                    System.out.println("No-RetroTarget");
                }
            }
        }

        if (kLimelight.doShuffleboard){
            //Pushing readings to shuffleboard
            if (retroTargetDistance != lastRetroDistance){
                if (Constants.kLimelight.KretroTarget.retroDistanceDebug){
                    // System.out.printf("[Update] Retro-Distance:");
                }
                retroDistanceWidget.setDouble(retroTargetDistance); //Updating shuffleboard
            }
        }
        lastRetroDistance = retroTargetDistance;
    }
    
    @Override
    public void periodic() {
        updateRobotPosition();
        updateRetroDistance();
    }
}
