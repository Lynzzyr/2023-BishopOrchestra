// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

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
        public static final int port_joystickMain               = 0;
        public static final int port_joystickSecondary          = 1;
    }

    public static final class kDrivetrain {

        public static final class kMotor {
            public static final int id_leftFrontDrive           = 20;
            public static final int id_leftCentreDrive          = 21;
            public static final int id_leftRearDrive            = 22;

            public static final int id_rightFrontDrive          = 23;
            public static final int id_rightCentreDrive         = 24;
            public static final int id_rightRearDrive           = 25;

            public static final double rampRate                 = 0.3; // seconds

            public static final int currentLimit                = 40;
        }

        public static final class kCANCoder {
            public static final int id_leftEncoder              = 30;
            public static final int id_rightEncoder             = 29;
            public final static double enc_CountsPerRevolution  = 4096;
            public final static double enc_SensorCoefficient    = (Math.PI * kDrivetrain.kWheel.wheelDiameter) / enc_CountsPerRevolution;
            public final static String enc_UnitString           = "m";
        }

        public static class kWheel {
            public final static double wheelDiameter            = 0.09404; // metres, placeholder value
            public final static double wheelCircumference       = Math.PI * wheelDiameter; // metres
        }
    }

    public static final class kGyro {
        public static final int id_gyro                         = 10;

        public static final AxisDirection mountPoseForward      = AxisDirection.NegativeY;
        public static final AxisDirection mountPoseUp           = AxisDirection.PositiveZ;
    }
}
