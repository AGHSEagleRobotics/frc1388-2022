// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public final class USBConstants {
        public static final int DRIVE_CONTROLLER = 0;
        public static final int OP_CONTROLLER = 1;
    }

    public final class DriveTrainConstants {
        public static final double EXPIRATION = 0.1;
        public static final double MAX_OUTPUT = 1.0;
        public static final double DEADBAND = 0.2;

        //THIS is for the 2022 ROBOT
        public static final int CANID_LEFT_FRONT    = -1;
        public static final int CANID_LEFT_BACK     = -1;
        public static final int CANID_RIGHT_FRONT   = -1;
        public static final int CANID_RIGHT_BACK    = -1;


        //This is for KNIGHTMARE!
        // public static final int CANID_LEFT_FRONT    = 4;
        // public static final int CANID_LEFT_BACK     = 3;
        // public static final int CANID_RIGHT_FRONT   = 2;
        // public static final int CANID_RIGHT_BACK    = 1;

    }

    public final class ShooterConstants {
        public static final int CANID_SHOOTER_MOTOR = 5;
        public static final int CANID_FEEDER_MOTOR = 6;

        public static final double GAINS_VELOCITY_F = 0;
        public static final double GAINS_VELOCITY_P = 0.1; // using motor for testing
        public static final double GAINS_VELOCITY_I = 0;
        public static final double GAINS_VELOCITY_D = 0;

        public static final double FORWARD_FEEDER_SPEED = 1.0;
        public static final double REVERSE_FEEDER_SPEED = -1.0;
        public static final double FEEDER_SPEED_OFF = 0.0;
        
    }

    public final class ClimberConstants {
        public static final int CANID_WINCH       = 8;
        public static final int CANID_ARTICULATOR = 7;
        
        public static final double CLIMBER_MAX_POWER_FORWARDS      = 0.3;
        public static final double CLIMBER_MAX_POWER_REVERSE       = -0.3;
        public static final double ARTICULATOR_MAX_POWER_FORWARDS  = 0.2;
        public static final double ARTICULATOR_MAX_POWER_REVERSE   = -1.0;

        public static final double GAINS_VELOCITY_F = 0;
        public static final double GAINS_VELOCITY_P = 0.1; // using motor for testing
        public static final double GAINS_VELOCITY_I = 0;
        public static final double GAINS_VELOCITY_D = 0;
    }

    public final class ClimberCommandConstants {
            /** speed in inches per second */
        public static final double MAX_WINCH_SPEED = 10;
            public static final double DEADBAND = 0.2;


    }

    public final class FalconConstants {
        public static final int SENSOR_CYCLES_PER_SECOND = 10;
        public static final double COUNTS_PER_REV = 2048.0;

    }
 
}
