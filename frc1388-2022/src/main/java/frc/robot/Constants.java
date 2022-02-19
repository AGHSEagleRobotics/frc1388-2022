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

        public static final int CANID_LEFT_FRONT    = 1;
        public static final int CANID_LEFT_BACK     = 2;
        public static final int CANID_RIGHT_FRONT   = 3;
        public static final int CANID_RIGHT_BACK    = 4;

        //Encoder stuff
        private final static double WHEEL_DIAMETER = 6.0; //FIXME make range value?
        private final static double COUNTS_PER_REV = 2048;
        private final static double FALCON_GEAR_RATIO = 4.67;
        private final static double WHEEL_TO_SPROCKETS_DRIVE_RATIO = 5/2;
        public final static double encoderDistanceRatio = 
           COUNTS_PER_REV *
           FALCON_GEAR_RATIO *
           WHEEL_TO_SPROCKETS_DRIVE_RATIO;

        public final static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    }

    public final class ShooterConstants {
        public static final int CANID_SHOOTER_MOTOR = 5;
        public static final int CANID_FEEDER_MOTOR  = 6;

        public static final double GAINS_VELOCITY_F = 0;
        public static final double GAINS_VELOCITY_P = 0.1; // using motor for testing
        public static final double GAINS_VELOCITY_I = 0;
        public static final double GAINS_VELOCITY_D = 0;

        public static final double RPM_RANGE = 30;              // EX. 30 = +/-30 rpm
        public static final double ITERATIONS_AT_TARGET_RPM = 20;    // the amount of times rpm needs to be between +/- RPM_RANGE for shooterSpeedIsReady() to return true

        public static final double SHOOTER_COOLDOWN_TIME = 0.5; // time in seconds to wait before shooter is turned off
        
        public static final double SHOOTER_RPM_STEP_CHANGE = 50;
        public static final double MAX_SHOOTER_RPM = 6000;
        public static final double MIN_SHOOTER_RPM = 0;

        public static final double FORWARD_FEEDER_SPEED = 1.0;
        public static final double REVERSE_FEEDER_SPEED = -1.0;
        public static final double FEEDER_SPEED_OFF = 0.0;

    }

    public final class ClimberConstants {
        public static final int CANID_WINCH = 8;
        public static final int CANID_ARTICULATOR = 7;

    }

    public final class IntakeConstants { 
        //FIXME 
        public static final int CANID_WHEEL_MOTOR = 10; 
        public static final int CANID_ARM_MOTOR = 9; 

        public static final double ARM_SPEED_DEPLOY = 0.5;
        public static final double WHEEL_SPEED_INTAKE = 1.0;
    }

    public final class TransitionConstants {
        public static final int CANID_TRANSITION_MOTOR = 11;
        public static final double TRANSITION_SPEED_FORWARD_FAST = 0.5; 
        public static final double TRANSITION_SPEED_FORWARD_SLOW = 0.1; 
        public static final double TRANSITION_SPEED_REVERSE_SLOW = -0.1;

    }

    public final class AutoMoveConstants {
        //TEST P VALUE LATER
        public static final double P_VALUE = 0.03;
        public static final int USB_CAMERACOLOR = 0; //Not confirmed
    }
 
}
