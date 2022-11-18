// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public final class USBConstants { //ADD STATIC?
        public static final int DRIVE_CONTROLLER = 0;
        public static final int GUEST_CONTROLLER = 2;
        //public static final void info(String msg) {    wanted to add this
        //}
    }

    public final class XBoxControllerConstants {
        public static final double TRIGGER_THRESHOLD = 0.5;
    }

    public final class DriveTrainConstants {
        public static final double EXPIRATION = 0.1;
        public static final double MAX_OUTPUT = 1.0;
        public static final double DEADBAND =   0.1;

        public static final int CANID_LEFT_FRONT    = 1;
        public static final int CANID_LEFT_BACK     = 2;
        public static final int CANID_RIGHT_FRONT   = 3;
        public static final int CANID_RIGHT_BACK    = 4;

        //Encoder stuff
        private final static double COUNTS_PER_REV = 2048;
        private final static double REVS_PER_COUNT = 1/COUNTS_PER_REV;
        private final static double WHEEL_DIAMETER_INCHES = 6.0; // make range value?
        private final static double FALCON_TO_SIMPLE_BOX_GEAR_RATIO = 1/4.67;
        private final static double SIMPLE_BOX_TO_WHEELS_RATIO = 12.0/30.0; //12 sprockets simple box to 30 sprockets wheel
        public final static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
        public final static double INCHES_PER_ENCODER_UNITS = 
           REVS_PER_COUNT *
           FALCON_TO_SIMPLE_BOX_GEAR_RATIO *
           SIMPLE_BOX_TO_WHEELS_RATIO *
           WHEEL_CIRCUMFERENCE;
        
    }

    public final class ShooterConstants {
        public static final int CANID_SHOOTER_MOTOR = 5;
        public static final int CANID_FEEDER_MOTOR  = 6;   

        public static final double GAINS_VELOCITY_F = 0.047;
        public static final double GAINS_VELOCITY_P = 0.25; // using motor for testing
        public static final double GAINS_VELOCITY_I = 0;
        public static final double GAINS_VELOCITY_D = 0;

        /**RPM RANGE sets the range at which the shooter is considered "at target speed"
         * It 
        */
        public static final double TARGET_RPM_TOLERANCE = 200;             // EX. 30 = +/-30 rpm
        //ITERATIONS is subject to change if shooting is sloppy
        public static final double ITERATIONS_AT_TARGET_RPM = 3;    // the amount of times rpm needs to be between +/- RPM_RANGE for shooterSpeedIsReady() to return true

        public static final double SHOOTER_COOLDOWN_TIME = 2.5; // time in seconds to wait before shooter is turned off (NOTE: feeder turns off instantly)
        

        //DEV Mode things
        public static final double SHOOTER_RPM_STEP_CHANGE = 50;
        public static final double MAX_SHOOTER_RPM = 6000;
        public static final double MIN_SHOOTER_RPM = 0;

        //High and Low goal RPM 
        public static final double SHOOTER_RPM_HAIL_HARRY  = 5900;
        public static final double SHOOTER_RPM_HIGHGOAL    = 3900; //lowered, subject to change
        public static final double SHOOTER_RMP_LOWGOAL     = 2200;
        public static final double SHOOTER_RPM_EJECT       = 1200;

        public static final double SHOOT_LOW_RIGHT_DRIVE_TRIGGER = 0.5;

        //Feeder speed
        public static final double FORWARD_FEEDER_SPEED = 1.0;
        public static final double REVERSE_FEEDER_SPEED = -1.0;
        public static final double FEEDER_SPEED_OFF = 0.0;

    }

    public final static class ClimberConstants {
        public static final int CANID_WINCH       = 8;
        public static final int CANID_ARTICULATOR = 7;

        public static final int DIO_WINCH_LIMIT = 4;

        public static final double CLIMBER_MAX_POWER_FORWARDS      = 0.5;
        public static final double CLIMBER_MAX_POWER_REVERSE       = -0.5;
        public static final double ARTICULATOR_MAX_POWER_FORWARDS  = 0.25; // was 0.15 // TODO: i changed this :)
        public static final double ARTICULATOR_MAX_POWER_REVERSE   = -0.15;

        public static final int    ARTICULATOR_MAX_SMART_CURRENT_LIMIT = 40; // AMPS

        public static final double WINCH_GAINS_VELOCITY_F = 0;
        public static final double WINCH_GAINS_VELOCITY_P = 0.1; // using motor for testing
        public static final double WINCH_GAINS_VELOCITY_I = 0;
        public static final double WINCH_GAINS_VELOCITY_D = 0;

        public static final double ARTICULATOR_GAINS_POSITION_F = 0;
        public static final double ARTICULATOR_GAINS_POSITION_P = 0.1; // using motor for testing
        public static final double ARTICULATOR_GAINS_POSITION_I = 0;
        public static final double ARTICULATOR_GAINS_POSITION_D = 0;

        //Possibly arbitrary
        public static final int ARTICULATOR_POSITION_REACH    = 5;  // rotations of motor  // TODO change these
        public static final int ARTICULATOR_POSITION_VERTICAL = 0;   // rotations of motor  // 

        //Tolerances - used like enums
        public static final int ARTICULATOR_IN_RANGE    =  0;
        public static final int ARTICULATOR_BELOW_RANGE = -1;
        public static final int ARTICULATOR_ABOVE_RANGE =  1;

        public static final float ARTIUCLATOR_REACH_SOFT_LIMIT    = 10.0f; // measured in rotations unless scaled otherwise NOTE: these are floats 
        public static final float ARTIUCLATOR_VERTICAL_SOFT_LIMIT = 0.0f;  // measured in rotations unless scaled otherwise NOTE: these are floats 

        //Get rid of extra constant, use 5 and 0? TODO
        public static enum ArticulatorPositions {
            VERTICAL(ARTICULATOR_POSITION_VERTICAL), REACH(ARTICULATOR_POSITION_REACH);

            private int m_position;

            ArticulatorPositions(int position) {
                m_position = position;
    }

            public int getPosition() {
                return m_position;
            }
          }
    }

    //change deadband if needed
    public final class ClimberCommandConstants {
            /** speed in inches per second */
        public static final double MAX_WINCH_SPEED = 10;
        public static final double DEADBAND        = 0.2;


    }

    public final class FalconConstants {
        public static final int SENSOR_CYCLES_PER_SECOND = 10;
        public static final int COUNTS_PER_REV        = 2048;

    }

    public final class IntakeConstants { 
        public static final int CANID_WHEEL_MOTOR = 10; 
        public static final int CANID_ARM_MOTOR = 9;

        public static final double ARM_SPEED_DEPLOY   = 1;
        public static final double ARM_SLOW_DEPLOY    = 0.25;
        public static final int ARM_SLOW_SPEED_TICKS  = 50;
        public static final double WHEEL_SPEED_INTAKE = 0.5;

        //DIO ports for sensors
        public static final int DIGITAL_INPUT_LIMIT_SWITCH_PORT = 0;
        public static final int DIGITAL_INPUT_ENCODER_CHANNEL_A = 1;
        public static final int DIGITAL_INPUT_ENCODER_CHANNEL_B = 2;

        //Encoder constants
        public static final int INTAKE_ARM_ENCODER_COUNTS_PER_REV = 2048;
        public static final double INTAKE_ARM_ENCODER_DOWN_ROTATIONS = 1.75; //specifically down, because down to up is 1.75 back to 0
        public static final int INTAKE_ARM_DOWN_ENCODER_COUNT = 
        (int) (INTAKE_ARM_ENCODER_DOWN_ROTATIONS * INTAKE_ARM_ENCODER_COUNTS_PER_REV);

        public static final double INTAKE_ARM_ENCODER_NEAR_UP_ROTATIONS = 0.5; //this is going from 1.75 to 0, so it's closer to 0
        public static final int INTAKE_ARM_NEAR_UP_ENCODER_COUNT = 
        (int) (INTAKE_ARM_ENCODER_NEAR_UP_ROTATIONS * INTAKE_ARM_ENCODER_COUNTS_PER_REV);
    }

    public final class TransitionConstants {
        public static final int CANID_TRANSITION_MOTOR = 11;
        public static final double TRANSITION_SPEED_FORWARD_FAST = 0.75; 
        public static final double TRANSITION_SPEED_FORWARD_MEDIUM = 0.5;
        public static final double TRANSITION_SPEED_FORWARD_SLOW = 0.3; 
        public static final double TRANSITION_SPEED_REVERSE_SLOW = -0.3;
        public static final double TRANSITION_SPEED_REVERSE_MEDIUM = -0.5;
        public static final double TRANSITION_SPEED_REVERSE_FAST = -0.75;

    }

    public final class AutoConstants {
        //TEST P VALUE LATER
        
        public static final double TURN_P_VALUE = 0.03;
        public static final double TURN_P_TOLERANCE = 1.25;
        public static final double MOVE_P_VALUE = 0.04;
        public static final double MOVE_P_TOLERANCE = 0.5;

        public static final double MOVE_F_VALUE = 0;
     

        public static final int USB_CAMERACOLOR = 0; //FIXME Not used?

        public static final double AUTO_SHOOT_RPM = 3700; 
        public static final double SHOOTER_TIMER_1 = 1;
        public static final double SHOOTER_TIMER_2 = 1.7;

        public static final double ENCODER_DISTANCE_CUTOFF = 1.0; //TODO change - is this cutoff??
        public static final double AUTO_DRIVE_SPEED = 0.5;

        public static final double AUTO_TIME = 15.0; //seconds
        public static final double AUTO_LEAVE_TARMAC_DISTANCE = 70; //inches

        public static final double AUTO_TURN_SPEED = 0.25;
        public static final double AUTO_TURN_ANGLE_MAX = 75; //degrees //changed from 79

        public static final double AUTO_POSITION_4_DISTANCE_TO_WALL_BALL = 42;
        public static final double AUTO_POSITION_4_DISTANCE_TAXI = 7;
        public static final double AUTO_POSITION_4_DISTANCE_2_BALL_BACK = -37; //was -28
        public static final double AUTO_POSITION_4_DISTANCE_3_BALL = -26;

    }
 

    public static final class DashboardConstants {
        public static enum Cameras {
            FORWARDS, REVERSE
        }

        public final static int FRONT_CAMERA_PORT = 2;
        public final static int REVERSE_CAMERA_PORT = 1;
        public final static int BALL_CAMERA_PORT = 0;

        public final static String BALL_CAM_URL = "http://roboRIO-1388-FRC.local:1183/?action=stream";
    }
 
    public final static class RumbleConstants {
        public static final double RUMBLE_PUSLE_TIME = 0.2;
        public static final double ANTI_RUMBLE_TIME = 0.2;
        public static final double RUMBLE_STRENGTH = 1.0;
        public static final int RUMBLE_OFF = 0;
        public static final int NUMBER_OF_PULSES = 3;

        //TODO change to number of buzzes instead of side
        public static enum RumbleSide{
            LEFT(RumbleType.kLeftRumble), 
            RIGHT(RumbleType.kRightRumble), 
            BOTH(RumbleType.kLeftRumble, RumbleType.kRightRumble), 
            NONE;
            private final RumbleType[] rumbleTypes;

            RumbleSide( RumbleType... types ){
                rumbleTypes = types;
            }
            public RumbleType[] getRumbleType(){
                return rumbleTypes;
            }
        }
    }

    public final class LEDConstants {
        public static final int PWM_LED_BODY = 0;
        public static final int PWM_LED_ARMS = 1;

        public static final double RED_SOLID = 0.61;
        public static final double BLUE_SOLID = 0.87;

        public static final double RED_FLASH = -0.85;
        public static final double BLUE_FLASH = -0.83;

        public static final double RED_LARSON = -0.35;
        public static final double BLUE_LARSON = 0.19;
    }
    public final class GuestModeConstants{
        public static final double GUEST_MODE_MAX_SPEED = 0.8;
        public static final double GUEST_MODE_MINIMUM_SPEED = 0.4;
        
    }

}