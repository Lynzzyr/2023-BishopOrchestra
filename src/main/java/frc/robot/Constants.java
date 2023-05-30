// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final class kOperator {
                                                          /*   Team 5409   */
        public static final int teamNumber                      = 5409;
                                                          /*  The Chargers */
                                                          
        public static final int port_joystickMain               = 0;
        public static final int port_joystickSecondary          = 1;

        public static final double timerRumbleIntensity         = 0.5;
        public static final double clawRumbleIntensity          = 1;
    }

    public static final class kCANBus {
        public static final String bus_rio                      = "rio";
        public static final String bus_drive                    = "rio";
    }

    public static final class kDrivetrain {

        public static final class kMotor {
            public static final int id_leftFrontDrive           = 20;
            public static final int id_leftCentreDrive          = 21;
            public static final int id_leftRearDrive            = 22;

            public static final int id_rightFrontDrive          = 23;
            public static final int id_rightCentreDrive         = 24;
            public static final int id_rightRearDrive           = 25;

            public static final int currentLimit                = 40;

        }

        public static final class kCANCoder {
            public static final int id_leftEncoder              = 29;
            public static final int id_rightEncoder             = 30;
            public static final double enc_CountsPerRevolution  = 4096;
            public static final double enc_SensorCoefficient    = (Math.PI * kDrivetrain.kWheel.wheelDiameter) / enc_CountsPerRevolution;
            public static final String enc_UnitString           = "m";
        }

        public static final class kWheel {
            public static final double wheelDiameter            = 0.15;
            public static final double wheelCircumference       = Math.PI * wheelDiameter; // metres
        }

        public static final double ksVolts                      = 0.18289;
        public static final double kvVolts                      = 1.9159;
        public static final double kaVolts                      = 0.29742;

        public static final double kPDriveVel                   = 2.5398;

        public static final double kTrackWidth                  = 0.6;
        public static final DifferentialDriveKinematics kDriveKinematics
            = new DifferentialDriveKinematics(kTrackWidth);

        public static final class kAuto {
            public static final double kMaxVolts                = 10;

            public static final double kMaxSpeed                = 2;
            public static final double kMaxAcceleration         = 2;

            // Default baseline values
            // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html#ramsete-parameters
            public static final double kRamseteB                = 2;
            public static final double kRamseteZeta             = 0.7;
        }

        public static final class kDriveteam {
            public static final double rampRate                 = 0.0;

            public static final double defaultSpeedMultiplier   = 0.85;
            public static final double defaultTurningMultiplier = 0.8;
                
            public static final double slowSpeed                = 0.5;
            public static final double slowTurn                 = 0.6;

            public static final double boostSpeed               = 1;
            public static final double boostTurningSpeed        = 0.7;

            public static enum GearState {
                kSlow,
                kDefault,
                kBoost
            }
        }

        public static final double kRotateP                     = 0;
        public static final double kRotateI                     = 0;
        public static final double kRotateD                     = 0;


        public static final class kSlew {
            public static final double kForwardSlew             = 2.2;
            public static final double kSidewaysSlew            = 3.4;
        }
    }

    public static final class kClaw {

        public static final int clawCANID                       = 39;

        public static final int dutyCycleChannel                = 3;

        public static final int LEFT_ToFCANID                   = 36;
        public static final int RIGHT_ToFCANID                  = 37;

        public static final int currentLimit                    = 20;

        public static final double outputLimit                  = 3;

        public static final double timeout                      = 1.25;

        public static final double openPosition                 = 0.77; // old 0.77
        public static final double armedOpenPosition            = 0.67; // old 0.77
        public static final double armedDoublePosition          = 0.6;
        public static final double coneClosePosition            = 0.54;
        public static final double cubeClosePosition            = 0.585;

        public static final int coneDistanceThreshold           = 160; // old 160
        public static final int cubeDistanceThreshold           = 130;
        public static final int doubleDistanceThreshold         = 220;
        
        public static final int stallTime                       = 40;

        public static final double encoderTolerance             = 0.01;

        public static final double kP                           = 20;
        public static final double kI                           = 0;
        public static final double kD                           = 0;
        public static final double kF                           = 0;

        //distance from the claw to the object in front of it
        public static final double objectRange                  = 150;

        public static final int dutyCycleRatio                  = 111538;

        public static enum kClawState {
            kOpen,
            kClose
        }

        public static final double manualMovementSpeed          = 0.15;

    }

    public static final class kGyro {
        public static final int id_gyro                         = 10;

        public static final AxisDirection mountPoseForward      = AxisDirection.NegativeY;
        public static final AxisDirection mountPoseUp           = AxisDirection.PositiveZ;
    }

    /**
     * Refer to auto routines diagram document:
     * https://docs.google.com/document/d/1pk5vwyWT9BPNdzbD-7wMtg9T3llIuKD4-a5t5yMFHZo/edit?usp=sharing
     */
    public static final class kAutoRoutines {

        public static final class kOneConeOnePickup {
            public static final String TURN_LEFT_place_pickup_balance = "A1L. TURN LEFT place, pickup, balance";
            public static final String TURN_RIGHT_place_pickup_balance = "A1R. TURN RIGHT place, pickup, balance";

            public static final String CENTER_place_pickup_balance = "A2. CENTER place, pickup, balance";

            public static final String TURN_LEFT_place_pickup_no_balance = "A3L. TURN LEFT place, pickup, no balance";
            public static final String TURN_RIGHT_place_pickup_no_balance = "A3R. TURN RIGHT place, pickup, no balance";

            public static final String[] all = {
                TURN_LEFT_place_pickup_balance,
                TURN_RIGHT_place_pickup_balance,

                CENTER_place_pickup_balance,

                TURN_LEFT_place_pickup_no_balance,
                TURN_RIGHT_place_pickup_no_balance
            };
        }
        
        public static final class kOneConeAuto {
            public static final String TURN_LEFT_place_and_balance = "B1L. TURN LEFT place and balance";
            public static final String TURN_RIGHT_place_and_balance = "B1R. TURN RIGHT place and balance";

            public static final String PLACE_SIDE_and_leave_community_no_balance = "B2. PLACE SIDE and leave community, no balance";

            public static final String CENTER_place_leave_community_and_balance = "B3. CENTER place, leave community, and balance";

            public static final String[] all = {
                TURN_LEFT_place_and_balance,
                TURN_RIGHT_place_and_balance,

                PLACE_SIDE_and_leave_community_no_balance,
                
                CENTER_place_leave_community_and_balance
            };
        }

        public static final class kConePlacePickupPlaceAuto {
            public static final String TURN_LEFT_place_pickup_place_no_balance = "C1L. TURN LEFT place, pickup, place, no balance";
            public static final String TURN_RIGHT_place_pickup_place_no_balance = "C1R. TURN RIGHT place, pickup, place, no balance";

            public static final String[] all = {
                TURN_LEFT_place_pickup_place_no_balance,
                TURN_RIGHT_place_pickup_place_no_balance
            };
        }
    }

    public static final class kBalancing {
        public static final double targetPitch                  = 0;
        public static final double maxAngle                     = 33.25;
        public static final double angleTolerance               = 1.5;

        public static final double kP                           = 0.0318; //0.0319 from newmarket
        public static final double kI                           = 0;
        public static final double kD                           = 0;
    }

    public static final class kTurn90DegreesChargeStation {
        public static final double maxAngle                     = 90;
        public static final double angleTolerance               = 1.5;

        public static final double kP_chargeStation             = 0.0125;
        public static final double kI_chargeStation             = 0;
        public static final double kD_chargeStation             = 0;
    }
    public static class kCANdle {
        public static final int staticTime                      = 750;

        public static class kConfig {

            public static final int CANID                       = 19;
            public static final int LEDCount                    = 94;

            public static final int LEDInnerRight               = 30;
            public static final int LEDInnerLeft                = 26;
            public static final int LEDOutter                   = 15;
        }

        public static class kColors {

            public static final int[] idle                      = {255, 134 , 0};
            public static final int[] cube                      = {142, 39, 245};
            public static final int[] cone                      = {237, 120, 0};

            public static final int LEDSinCount                 = 8;
            public static final double kSpeed                   = 0.5;

            public static final double sinFrequency             = 0.025;
            public static final double sinFrequencySpeed        = 20;

            public static final int chargeSpeed                 = 4;

            public static final double gameSpeed                = 0.2;

            public static final int blinkSpeed                   = 6;
            public static final int blinkTime                    = 10;
            
        }

        public enum AnimationTypes {
            Static,
            ColorFlow,
            //custom
            SinWave,
            SinFlow,
            ChargedUp,
            EStopped,
            EndGame
        }

        public enum LEDColorType {
            Cone,
            Cube
        }
    }

    public static class kArmSubsystem {
        public static final int kMotor1ID                       = 32;
        public static final int kMotor2ID                       = 33;
        public static final int kEncoderChannel                 = 4;

        public final static double kVoltageLimit                = 6; //50% speed
        public final static double kVoltageManual               = 1.5;
        public final static int kCurrentLimit                   = 30;
        public final static double kPositionTolerance           = 0.001;
        public final static double kg                           = 0.4;

        public static class kPID {
            public static final double kP                       = 50;
            public static final double kI                       = 0;
            public static final double kD                       = 0;
        }
    
        public static class kSetpoints {

            public final static double kToTop                   = 0.67; // 41.75 inches
            public final static double kConeMid                 = 0.62; // 36 inches
            public final static double kConeAbove               = 0.67;  // 43.75 inches
            public final static double kConeAboveNew            = 0.66;  // 43.75 inches
            public final static double kToMid                   = 0.63; // 28.75 inches

            public final static double kToLoadingshoulder       = 0.672; // 38.5 inches
            public final static double kIdling                  = 0.96; // 48.5 inches
            public final static double kGroundPickupCube        = 1.19; 
            public final static double kGroundPickupCone        = 1.21; 
            public final static double kAutoDrivingWithCone     = 1.15;
            public final static double kConeLow                 = 1.16;

            // setpoint of hardstop of shoulder side of robot: 0.513
            // measurement in inches is from the edge of the claw plate
        } 

    }



    public static final class kTelescope {

        public static final double kCentemetreSafetyFactor      = 1.0;

        public static final class kDeviceID {
            public static final int MOTOR_CAN_ID                = 38;

            public static final int MAX_LIMIT_SWITCH_ID         = 1;
            public static final int MIN_LIMIT_SWITCH_ID         = 2;
        }

        public static final class kPID {
            public static final double kP                       = 0.1;
            public static final double kI                       = 0.0;
            public static final double kD                       = 0.0;
            public static final double kF                       = 0.0;
        }

        public static final class kSprocket {
            public static final double kGearPitchDiameter       = 3.637; // Pitch diameter of sprocket in cm
            public static final double kGearPitchCircumfrence   = Math.PI * kGearPitchDiameter; // Pitch Circumfrence in cm
            
            // public static final double kNumberOfEncoderTicks        = 42;
            public static final double kGearConversionFactor    = kGearPitchCircumfrence * 0.05;
        }

        public static final class kDestinations {
            public static final double kRetracted               = 0.0;

            public static final double kExtended                = 33.1;
            public static final double kMidFront                = 1.50;
            public static final double kMidBack                 = 33.1;
            
            public static final double kHandoff                 = 0.0;

            public static final double kGroundFront             = 0.0; // placeholder
            public static final double kAutoGroundBack          = 11.95; 
            public static final double kCubeGround              = 10;
            public static final double kGroundBack              = 18; 
            
            public static final double kLoading                 = 0.0; // placeholder

        }
    }

    public static final class kIntake {
        public static final int id_motPivot                     = 35;
        public static final int id_motWrist                     = 34;
        public static final int id_motRoller                    = 28;

        public static final int port_encPivot                   = 5;
        public static final int port_encWrist                   = 0;

        public static final double kPivotP                      = 25; /* testing */
        public static final double kWristP                      = 20;

        public static final double kPivotI                      = 0.0; /* placeholder */
        public static final double kWristI                      = 0.0; /* placeholder */

        public static final double kPivotD                      = 0.0; /* placeholder */
        public static final double kWristD                      = 0.0; /* placeholder */

        public static final int kIntakeCurrentLimit             = 30;

        public static final double kRollerInVolts               = 3.6;
        public static final double kRollerReverseVolts          = -1;

        public static final class kSetpoints {
            public static final class kPivotSetpoints {
                public static final double kPivotExtended       = 0.36;
                public static final double kPivotHugging        = 0.17;
                public static final double kPivotStoring        = 0.088;
                public static final double kPivotTestA          = 0.13;
                public static final double kPivotTestB          = 0.3;
            }

            public static final class kWristSetpoints {
                public static final double kWristPickup         = 0.77;
                public static final double kWristHandoff        = 0.41;
                public static final double kWristStoring        = 0.0; /* placeholder */
            }
        }

        public static final class kVoltageLimits {
            public static final double kPivotVoltageLimit       = 6; /* testing */
            public static final double kWristVoltageLimit       = 6;
        }
    }

    public static class kLimelight {
        public static final double heightOffFloor               = 45.72; //cm
        public static final int angle                           = 11; //degrees
        public static final int kAutoLightTimeout               = 1000; //ms
        public static final boolean kDoAutoLight                = false; 
        public static final double kALTriggerDistance           = 1; //PLACEHOLDER
        public static final boolean doShuffleboard              = false; 
        
        public static final class limeLightAlert {
            public static final double disconnectNotifLength    = 200; //rumble time in ms
            public static final double limelightTimeout         = 500; //limelight disconnect timeout time in ms 
        }

        public static final class KretroTarget {
            public static final boolean retroDistanceDebug      = false; 
            public static final double lowNodeHeight            = 60.14; //cm
            public static final double[] lowNodeCrop            = {-1, 1, -1, 0.22}; // x, x, y, y
            public static final double[] highNodeCrop           = {-1, 1, 0.25, 1}; // x, x, y, y
        }

        public static class kConeNodeAim {
            public static final boolean KdoTargetOffset         = true;
            public static final double KlowNodeOffset           = 4.5;      //in degrees
            public static final double KhighNodeOffset          = 3.5;      //in degres 
            public static final double KretroTargetFF           = 0.275;    
            public static final double KretroTargetTolerance    = 0.3;    
            public static final double kP                       = 0.007;
            public static final double kI                       = 0.0001;
            public static final double kD                       = 0.0008;
            public static final boolean doPIDTuning             = false; 
            public static final boolean debugMode               = false;
        }
    }

    public static final class kStallDriveOnChargeStation {
        public static final double kForwardSpeed                = 0.345;
        public static final double kBackwardSpeed               = -0.34;
    }
}