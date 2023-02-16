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
    }

    public static final class kCANBus {
        public static final String bus_rio                      = "rio";
        public static final String bus_drive                    = "drive";
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
            public static final int id_leftEncoder              = 30;
            public static final int id_rightEncoder             = 29;
            public static final double enc_CountsPerRevolution  = 4096;
            public static final double enc_SensorCoefficient    = (Math.PI * kDrivetrain.kWheel.wheelDiameter) / enc_CountsPerRevolution;
            public static final String enc_UnitString           = "m";
        }

        public static final class kWheel {
            public static final double wheelDiameter            = 0.1; // metres, placeholder value
            public static final double wheelCircumference       = Math.PI * wheelDiameter; // metres
        }

        public static final double ksVolts                      = 0.08122;
        public static final double kvVolts                      = 2.796;
        public static final double kaVolts                      = 0.28485;

        public static final double kPDriveVel                   = 3.3466;

        public static final double kTrackWidth                  = 0.6;
        public static final DifferentialDriveKinematics kDriveKinematics
            = new DifferentialDriveKinematics(kTrackWidth);

        public static final class kAuto {
            public static final double kMaxVolts                = 10;

            public static final double kMaxSpeed                = 3;
            public static final double kMaxAcceleration         = 3;

            // Default baseline values
            // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html#ramsete-parameters
            public static final double kRamseteB                = 2;
            public static final double kRamseteZeta             = 0.7;
        }

        public static final class kDriveteam {
            public static final double rampRate                  = 0.2;

            public static final double defaultSpeedMultiplier    = 0.8;
            public static final double defaultTurningMultiplier  = 0.8;
                
            public static final double slowSpeed                 = 0.5;
            public static final double slowTurn                  = 0.6;

            public static final double boostSpeed                = 1;
            public static final double boostTurningSpeed         = 1;
                
            public static final double kChangeRamp               = 0.5;
            public static final int timerLength                  = 50;

            public static final double maxSpinSpeed              = 3;
            public static final double lowerSpinSpeed            = 0.7;
            public static final double spinRamp                  = 1;
            public static final int lowerTimer                   = 10;

            public static final double rumbleIntensity           = 1;

            public static enum GearState {
                kSlow,
                kDefault,
                kBoost
            }
        }
    }

    public static final class kGyro {
        public static final int id_gyro                         = 10;

        public static final AxisDirection mountPoseForward      = AxisDirection.NegativeY;
        public static final AxisDirection mountPoseUp           = AxisDirection.PositiveZ;
    }

    public static final class kTrajectoryPath {
        public static final String path1 = "Path1";
    }

    public static final class kBalancing {
        public static final double targetPitch                  = 0;
        public static final double maxAngle                     = 33.25;
        public static final double angleTolerance               = 1.5;

        public static final double kP                           = 0.04;
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

    public static class kArmSubsystem {
        public final static int kMotor1ID                       = 32;
        public final static int kMotor2ID                       = 33;
        public final static int kEncoderChannel                 = 4;

        public final static double kVoltageLimit                = 10.5;
        public final static int kCurrentLimit                   = 30;
        public final static double kPositionTolerance           = 0.1;
        public final static double kg                           = 0.4;
        public final static double knintydegreepos              = -0.042;

        public static class kPID {
            public final static double kP                       = 50;
            public final static double kI                       = 0;
            public final static double kD                       = 0;
        }

        public static class kSetpoints{
            public final static double kfront = 0.5; //0.43
            public final static double kback = 0.0; //0.56
            public final static double keight = 0.5;
            public final static double kzero = 0;
            // public final static double kplacehigh = 0.39;
            // public final static double kplacelow = 0.43;
            // public final static double kIdlepos = -0.02;

        }
    
    }
}