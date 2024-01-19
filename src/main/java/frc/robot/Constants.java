// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Global constants
    public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

    public static final class AutoConstants {

        public static enum AutoPattern {
            DO_NOTHING,
            DRIVE_FORWARD,
            DRIVE_FORWARD_PID_TIMED,
            DRIVE_FORWARD_PID_MEASURED,
            TURN_TO_HEADING
        };
    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, SINGLE_STICK_ARCADE, DUAL_STICK_ARCADE;
        }

        public static final int     LEFT_MOTOR_PORT                = 10;
        public static final int     RIGHT_MOTOR_PORT               = 20;

        public static final boolean LEFT_MOTOR_REVERSED            = false;
        public static final boolean RIGHT_MOTOR_REVERSED           = true;

        public static final boolean LEFT_ENCODER_REVERSED          = false;
        public static final boolean RIGHT_ENCODER_REVERSED         = true;

        public static final int     ENCODER_COUNTS_PER_REVOLUTION  = 1024;
        public static final double  ROBOT_WHEEL_DIAMETER_CMS       = 6 * 2.54;

        public static final double  CMS_PER_ENCODER_COUNT          =
            // Assumes the encoders are directly mounted on the wheel shafts
            (ROBOT_WHEEL_DIAMETER_CMS * Math.PI) / ENCODER_COUNTS_PER_REVOLUTION;

        public static final double  TIMED_STRAIGHT_DRIVE_PID_KP    = 1;
        public static final double  TIMED_STRAIGHT_DRIVE_PID_KI    = 0;
        public static final double  TIMED_STRAIGHT_DRIVE_PID_KD    = 0;

        public static final double  MEASURED_STRAIGHT_DRIVE_PID_KP = 1;
        public static final double  MEASURED_STRAIGHT_DRIVE_PID_KI = 0;
        public static final double  MEASURED_STRAIGHT_DRIVE_PID_KD = 0;

        public static final double  TURN_TO_HEADING_PID_KP         = 1;
        public static final double  TURN_TO_HEADING_PID_KI         = 0;
        public static final double  TURN_TO_HEADING_PID_KD         = 0;

        public static final double  HEADING_ERROR_BUFFER           = 0.5;

    }

    public static final class lightConstants {

        public static final int LED_STRIP_ONE_PORT   = 0;
        public static final int LED_STRIP_ONE_LENGTH = 60;


    }

    public static final class OperatorConstants {

        public static final int    DRIVER_CONTROLLER_PORT         = 0;
        public static final double GAME_CONTROLLER_STICK_DEADBAND = 0.2;
    }

}
