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
        public static final double DEADBAND   = 0.2;

        //THIS is for the 2022 ROBOT
        public static final int CANID_LEFT_FRONT    = 1;
        public static final int CANID_LEFT_BACK     = 2;
        public static final int CANID_RIGHT_FRONT   = 3;
        public static final int CANID_RIGHT_BACK    = 4;
    }

    public final class ShooterConstants {
        public static final int CANID_SHOOTER_MOTOR = 5;
        public static final int CANID_FEEDER_MOTOR  = 6;

        public static final double GAINS_VELOCITY_F = 0.047;
        public static final double GAINS_VELOCITY_P = 0.25; // using motor for testing
        public static final double GAINS_VELOCITY_I = 0;
        public static final double GAINS_VELOCITY_D = 0;

        public static final double RPM_RANGE = 200;              // EX. 30 = +/-30 rpm
        public static final double ITERATIONS_AT_TARGET_RPM = 20;    // the amount of times rpm needs to be between +/- RPM_RANGE for shooterSpeedIsReady() to return true

        public static final double SHOOTER_COOLDOWN_TIME = 0.5; // time in seconds to wait before shooter is turned off
        
        public static final double SHOOTER_RPM_STEP_CHANGE = 50;
        public static final double MAX_SHOOTER_RPM         = 6000;
        public static final double MIN_SHOOTER_RPM         = 0;

        public static final double FORWARD_FEEDER_SPEED = 1.0;
        public static final double REVERSE_FEEDER_SPEED = -1.0;
        public static final double FEEDER_SPEED_OFF     = 0.0;

    }

    public final static class ClimberConstants {
        public static final int CANID_WINCH       = 8;
        public static final int CANID_ARTICULATOR = 7;
        
        public static final double CLIMBER_MAX_POWER_FORWARDS      = 0.3;
        public static final double CLIMBER_MAX_POWER_REVERSE       = -0.3;
        public static final double ARTICULATOR_MAX_POWER_FORWARDS  = 0.2;
        public static final double ARTICULATOR_MAX_POWER_REVERSE   = -1.0;

        public static final double WINCH_GAINS_VELOCITY_F = 0;
        public static final double WINCH_GAINS_VELOCITY_P = 0.1; // using motor for testing
        public static final double WINCH_GAINS_VELOCITY_I = 0;
        public static final double WINCH_GAINS_VELOCITY_D = 0;

        public static final double ARTICULATOR_GAINS_POSITION_F = 0;
        public static final double ARTICULATOR_GAINS_POSITION_P = 0.1; // using motor for testing
        public static final double ARTICULATOR_GAINS_POSITION_I = 0;
        public static final double ARTICULATOR_GAINS_POSITION_D = 0;

        public static final int ARTICULATOR_POSITION_REACH    = 5;  // rotations  // TODO change this
        public static final int ARTICULATOR_POSITION_VERTICAL = 0;   // rotations  // TODO change this

        public static final int ARTICULATOR_IN_RANGE    =  0;
        public static final int ARTICULATOR_BELOW_RANGE = -1;
        public static final int ARTICULATOR_ABOVE_RANGE =  1;

        public static final float ARTIUCLATOR_REACH_SOFT_LIMIT    = 10.0f; // mesured in rotations unless scaled otherwis NOTE: these are floats 
        public static final float ARTIUCLATOR_VERTICAL_SOFT_LIMIT = 0.0f;  // mesured in rotations unless scaled otherwis NOTE: these are floats 

        public static enum ArticulatorPositions {
            VERTICAL(ARTICULATOR_POSITION_VERTICAL), REACH(ARTICULATOR_POSITION_REACH);

            private int m_position;

            ArticulatorPositions(int position){
                m_position = position;
            }

            public int getPosition() {
                return m_position;
            }
          }
    }

    public final class ClimberCommandConstants {
            /** speed in inches per second */
        public static final double MAX_WINCH_SPEED = 10;
        public static final double DEADBAND        = 0.2;


    }

    public final class FalconConstants {
        public static final int SENSOR_CYCLES_PER_SECOND = 10;
        public static final double COUNTS_PER_REV        = 2048.0;

    }

    public final class IntakeConstants { 
        //FIXME 
        public static final int CANID_WHEEL_MOTOR = 10; 
        public static final int CANID_ARM_MOTOR = 9; 

        public static final double ARM_SPEED_DEPLOY   = 0.5;
        public static final double WHEEL_SPEED_INTAKE = 1.0;
        public static final double REVERSE_WHEEL_SPEED_INTAKE = - 1.0;
    }

    public final class TransitionConstants {
        public static final int CANID_TRANSITION_MOTOR = 11;
        public static final double TRANSITION_SPEED_FORWARD_FAST = 0.75; 
        public static final double TRANSITION_SPEED_FORWARD_SLOW = 0.2; 
        public static final double TRANSITION_SPEED_REVERSE_SLOW = -0.2;

    }
 
}
