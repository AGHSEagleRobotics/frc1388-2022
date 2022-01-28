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

        public static final int CANID_LEFT_FRONT    = -1;
        public static final int CANID_LEFT_BACK     = -1;
        public static final int CANID_RIGHT_FRONT   = -1;
        public static final int CANID_RIGHT_BACK    = -1;

    }

    public final class ShooterConstants {
        public static final double GAINS_VELOCITY_F = 0;
        public static final double GAINS_VELOCITY_P = 0;
        public static final double GAINS_VELOCITY_I = 0;
        public static final double GAINS_VELOCITY_D = 0;
    }

    public final class ClimberConstants {
        public static final int CANIN_WINCH       = 6;
        public static final int CANIN_ARTICULATOR = 7;

    }
 
}
