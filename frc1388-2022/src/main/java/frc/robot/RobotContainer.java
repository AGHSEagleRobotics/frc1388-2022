// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import static frc.robot.Constants.AutoConstants.*;
// import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveTrainConstants;     // climber constats
import frc.robot.Constants.GuestModeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.USBConstants;            // USB
import frc.robot.Constants.XBoxControllerConstants;
// import frc.robot.Constants.ClimberConstants.ArticulatorPositions;
import frc.robot.Constants.DashboardConstants.Cameras;
import frc.robot.Dashboard.Objective;
import frc.robot.Dashboard.Position;
import frc.robot.commands.Drive;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.ShootHailHarry;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.REject;
import frc.robot.commands.ShootEject;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.ClimberCommand;
// import frc.robot.commands.ClimberRetract;
import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.commands.ClimberCommand;           // climber command
// import frc.robot.subsystems.ClimberSubsystem;       // climber subsystem
import frc.robot.subsystems.DriveTrainSubsystem;    // drive train subsystem
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final Logger log = LogManager.getLogger(Robot.class);

  // components
  public static XboxController m_driveController = new XboxController(USBConstants.DRIVE_CONTROLLER);
  public static XboxController m_guestController = new XboxController(USBConstants.GUEST_CONTROLLER);
  public static GuestMode m_guestMode = new GuestMode();
  // The robot's subsystems and commands are defined here...

  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final RumbleSubsystem m_rumbleSubsystem = new RumbleSubsystem(m_driveController);

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT), 
      new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK),
      new ADIS16470_IMU()
      );

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(
    new WPI_TalonFX(ClimberConstants.CANID_WINCH),
    new CANSparkMax(ClimberConstants.CANID_ARTICULATOR, MotorType.kBrushless),
    new DigitalInput(ClimberConstants.DIO_WINCH_LIMIT)
  );
  
  private final ShooterFeederSubsystem m_shooterFeederSubsystem = new ShooterFeederSubsystem(
    new WPI_TalonFX(ShooterConstants.CANID_SHOOTER_MOTOR),
      new CANSparkMax(ShooterConstants.CANID_FEEDER_MOTOR, MotorType.kBrushless));

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
      new WPI_TalonSRX(IntakeConstants.CANID_ARM_MOTOR),
    new CANSparkMax(IntakeConstants.CANID_WHEEL_MOTOR, MotorType.kBrushless), 
      new DigitalInput(IntakeConstants.DIGITAL_INPUT_LIMIT_SWITCH_PORT),
      new Encoder(IntakeConstants.DIGITAL_INPUT_ENCODER_CHANNEL_A, IntakeConstants.DIGITAL_INPUT_ENCODER_CHANNEL_B));

  private final TransitionSubsystem m_transitionSubsystem = new TransitionSubsystem(
      new CANSparkMax(TransitionConstants.CANID_TRANSITION_MOTOR, MotorType.kBrushless));

  private final Dashboard m_dashboard = new Dashboard();

  private final LED m_LED = new LED(
    new PWMSparkMax(LEDConstants.PWM_LED_BODY),
    new PWMSparkMax(LEDConstants.PWM_LED_ARMS)
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // m_climberSubsystem.setDefaultCommand(
    //   new ClimberCommand(
    //     m_climberSubsystem, 
    //         () -> m_guestController.getLeftY(), // extend
    //     () -> m_guestController.getRightY())    // articulate
    //   );
    // set default commands


    m_driveTrainSubsystem.setDefaultCommand(
        new Drive(
            m_driveTrainSubsystem,
            m_rumbleSubsystem,
            () -> m_driveController.getLeftY(),
            () -> m_driveController.getRightY(),
            () -> m_driveController.getRightX(),
            // rumble for precision mode
            () -> m_driveController.getRightStickButtonPressed(),
            () -> m_guestController.getLeftY(),
            () -> m_guestController.getRightX(),
            m_guestMode));
            


    m_transitionSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW),
            m_transitionSubsystem));
      
  // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //dpad climber
    new Button(() -> isDriverDPadUp())
      .whileHeld(() -> m_climberSubsystem.setWinchPower(-1));
    new Button(() -> isDriverDPadDown())
      .whileHeld(() -> m_climberSubsystem.setWinchPower(1));
    new Button(() -> isDriverDPadOff())
      .whileHeld(() -> m_climberSubsystem.setWinchPower(0));

    new JoystickButton(m_driveController, XboxController.Button.kA.value)
        .whenPressed(()-> m_guestMode.setGuestMode(true));


    new JoystickButton(m_driveController, XboxController.Button.kB.value)
        .whenPressed(() -> m_guestMode.setGuestMode(false));

    new Button(()-> isDriverJoysticksMoved())
      .whenPressed(()-> m_guestMode.setGuestMode(false));

    
    //  dev mode
     //new JoystickButton(m_driveController, XboxController.Button.kX.value)
     //.whenPressed(() -> m_shooterFeederSubsystem.shooterRpmStepIncrease());
     
     //new JoystickButton(m_driveController, XboxController.Button.kY.value)
    //.whenPressed(() -> m_shooterFeederSubsystem.shooterRpmStepDecrease());
     
     //new JoystickButton(m_driveController, XboxController.Button.kA.value)
     //.whenPressed(() -> m_shooterFeederSubsystem.setShooterEnabled(true));
     
     //new JoystickButton(m_driveController, XboxController.Button.kB.value)
     //.whenPressed(() -> m_shooterFeederSubsystem.setShooterEnabled(false));
     
    //  new JoystickButton(m_guestController, XboxController.Button.kRightBumper.value)
    //   .whileHeld(() -> m_transitionSubsystem.setTransitionSpeed(
    //         TransitionConstants.TRANSITION_SPEED_REVERSE_SLOW),
    //         m_transitionSubsystem);
     


    // INTAKE DEPLOY LEFT TRIGGER
    new Button(() -> isLeftDriverTriggerPressed())
        .whenPressed(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));

    //INTAKE DRIVE UP
    new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value)
        .whenPressed(new RetractIntake(m_intakeSubsystem));

    // INTAKE OP UP
    // new JoystickButton(m_guestController, XboxController.Button.kLeftBumper.value)
    //     .whenPressed(new RetractIntake(m_intakeSubsystem));

    // SHOOT LOW HIGH GOAL and EJECT
    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
        .whenHeld(new ShootHigh(m_shooterFeederSubsystem, m_transitionSubsystem, m_LED));
    
    new JoystickButton(m_guestController, XboxController.Button.kRightBumper.value)
        .whenHeld(new ShootHigh(m_shooterFeederSubsystem, m_transitionSubsystem, m_LED));

    new Button(RobotContainer::isRightDriverTriggerPressed)
        .whenHeld(new ShootLow(m_shooterFeederSubsystem, m_transitionSubsystem, m_LED));

    new Button(RobotContainer::isRightGuestTriggerPressed)
    .whenHeld(new ShootLow(m_shooterFeederSubsystem, m_transitionSubsystem, m_LED));

    // new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value)
    //     .whenHeld(new ShootHailHarry(m_shooterFeederSubsystem, m_transitionSubsystem, m_LED));

    //EJECT AND REJECT commands DRIVER 
    new JoystickButton(m_driveController, XboxController.Button.kBack.value)
      .whenHeld(new ShootEject(m_shooterFeederSubsystem, m_transitionSubsystem));

    // new Button (() -> isDriverDPadPressed()).whenHeld(new REject(
    //   m_intakeSubsystem, m_transitionSubsystem, m_shooterFeederSubsystem));

    // new Button (() -> isDriverDPadPressed())
    // .whenReleased(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));
  

    //EJECT AND REJECT commands OPERATOR
    new Button (() -> isRightGuestTriggerPressed())
    .whenHeld(new ShootEject(m_shooterFeederSubsystem, m_transitionSubsystem));

    // new JoystickButton(m_guestController, XboxController.Button.kRightBumper.value)
    // .whenHeld(new REject(m_intakeSubsystem, m_transitionSubsystem, m_shooterFeederSubsystem));

    // new JoystickButton(m_guestController, XboxController.Button.kRightBumper.value)
    // .whenReleased(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));



    // Lower priority
    /*
    new JoystickButton(m_guestController, XboxController.Button.kX.value)
      .whenPressed(() -> m_climberSubsystem.setArticulatorReach());
      
   new JoystickButton(m_guestController, XboxController.Button.kY.value)
      .whenPressed(() -> m_climberSubsystem.setArticulatorVertical());
    */

    // Reverse
    // new JoystickButton(m_driveController, XboxController.Button.kB.value)
    //   .whenPressed(() -> setForward(true));
    
    // new JoystickButton(m_driveController, XboxController.Button.kA.value)
    //   .whenPressed(() -> setForward(false));

    //Clean up?
    new JoystickButton(m_driveController, XboxController.Button.kStart.value)
        .whenPressed(
            new InstantCommand(() -> m_dashboard.switchCamera()) {
              @Override
              public boolean runsWhenDisabled() {
                return true;
  }
            });

    //OP
    // new JoystickButton(m_guestController, XboxController.Button.kStart.value)
    //     .whenPressed(
    //         new InstantCommand(() -> m_dashboard.switchCamera()) {
    //           @Override
    //           public boolean runsWhenDisabled() {
    //             return true;
    //           }
    //         });

    // .whenPressed(() -> m_dashboard.switchCamera());

  //   new Button(() -> isOpUpDpadPressed())
  //     .whenPressed(() -> m_LED.increaseLED());

  //   new Button(() -> isOpDownDpadPressed())
  //     .whenPressed(() -> m_LED.decreaseLED());
  }

  private void setForward(boolean isForward) {
    if (isForward) {
      m_driveTrainSubsystem.setForward(true);
      m_dashboard.setCamView(Cameras.FORWARDS);
    } else {
      m_driveTrainSubsystem.setForward(false);
      m_dashboard.setCamView(Cameras.REVERSE);
    }
  } 

  // triggers
  public static boolean isRightDriverTriggerPressed() {
    return m_driveController.getRightTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  public static boolean isRightGuestTriggerPressed() {
    return m_guestController.getRightTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  public static boolean isLeftDriverTriggerPressed() {
    return m_driveController.getLeftTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  public static boolean isLeftGuestTriggerPressed() {
    return m_guestController.getLeftTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  // driver dpad
  public static boolean isDriverDPadUp() {
    return m_driveController.getPOV() == 0;
  }
  public static boolean isDriverDPadDown() {
    return m_driveController.getPOV() == 180;
  }
  public static boolean isDriverDPadOff() {
    return m_driveController.getPOV() == -1;
  }

  // op dpad
  // public static boolean isOpUpDpadPressed() {
  //   return m_guestController.getPOV() == 0;
  // }

  // public static boolean isOpDownDpadPressed() {
  //   return m_guestController.getPOV() == 180;
  // }

  //EJECT REJECT FOR OP
  // public static boolean isRightOpTriggerPressed() {
  //   return m_guestController.getRightTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //new PrintCommand("************************\n************************\n getAutonomousCommand() \n\n");

    // Position position = m_dashboard.getPosition();
    // // m_dashboard.getPosition();
    // switch (position) {
    //   case POSITION1:
    //   //return new (Command Group)

    //   case POSITION2:
      
    //   case POSITION3:

    //   case POSITION4:

    // }

    Objective objective = m_dashboard.getObjective();
    Position position = m_dashboard.getPosition();
    log.info("Objective: " + objective + "   Position: " + position);
    //objective = Objective.MOVESHOOT1;
    
    switch (objective) {
      case LEAVETARMAC:
      //position 4 starts with robot toucing edge of tarmac
      if (position == Position.POSITION4) {
      return new RetractIntake(m_intakeSubsystem)
      .withTimeout(1.5)
      .andThen(new AutoMove(m_driveTrainSubsystem, 42, AUTO_DRIVE_SPEED))
      .andThen(new WaitCommand(1))
      .andThen(new AutoMove(m_driveTrainSubsystem, 8, 0.25));
      } else {
      // default:
      return new RetractIntake(m_intakeSubsystem)
        .withTimeout(2)
      .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_LEAVE_TARMAC_DISTANCE, AUTO_DRIVE_SPEED)
        .withTimeout(4)); 
    }

      case MOVEPICKUPSHOOT2:
      if (position == Position.POSITION4) {
        return new RetractIntake(m_intakeSubsystem)
          .withTimeout(2)
        .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem)
          .withTimeout(2))
        .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_TO_WALL_BALL, AUTO_DRIVE_SPEED)
          .withTimeout(3))//change pos 4 distance to 43
        .andThen(new WaitCommand(0.5))
        .andThen(new RetractIntake(m_intakeSubsystem)
          .withTimeout(2))
          //change distance going backwards on automove to Auto_Tarmac distance -40
        .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_TAXI, AUTO_DRIVE_SPEED)
          .withTimeout(2))
        .andThen(new AutoMove (m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_2_BALL_BACK, AUTO_DRIVE_SPEED)
          .withTimeout(2))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(1.7))
        .andThen(new WaitCommand(2.5))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_2))
        .andThen(new RetractIntake(m_intakeSubsystem)
          .withTimeout(2));
      } else {
        return new RetractIntake(m_intakeSubsystem)
          .withTimeout(2)
        .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem)
          .withTimeout(2))
        .andThen(new AutoMove(m_driveTrainSubsystem, 64, AUTO_DRIVE_SPEED)
          .withTimeout(3))
          //change distance going backwards on automove to Auto_Tarmac distance -40 //old comment
        .andThen(new AutoMove(m_driveTrainSubsystem, -52, AUTO_DRIVE_SPEED) //was -37
          .withTimeout(2))
          //If intake works properly, 1.8 or less works for both
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(1.7))
        .andThen(new WaitCommand(2.5))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_2))
        .andThen(new RetractIntake(m_intakeSubsystem)
          .withTimeout(2));
       }

        case MOVESHOOT1:
        if (position == Position.POSITION4) {
      return new RetractIntake(m_intakeSubsystem)
        .withTimeout(1.5)
      .andThen(new AutoMove(m_driveTrainSubsystem, 37, AUTO_DRIVE_SPEED))
      .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
        .withTimeout(SHOOTER_TIMER_1))
      .andThen(new RetractIntake(m_intakeSubsystem)
      .withTimeout(2));
        } else {
      return new RetractIntake(m_intakeSubsystem).withTimeout(2)
      .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_LEAVE_TARMAC_DISTANCE, AUTO_DRIVE_SPEED)
        .withTimeout(3))
      .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
        .withTimeout(SHOOTER_TIMER_1))
      // .andThen( new AutoTurn(m_driveTrainSubsystem,
      //     AUTO_TURN_SPEED,
      //     AUTO_TURN_ANGLE_MAX)
      //   .withTimeout(0.15))
      .andThen(new RetractIntake(m_intakeSubsystem)
        .withTimeout(2));
      }
      
      // case LOWSHOOTMOVE:
      // return new RetractIntake(m_intakeSubsystem).withTimeout(2)
      // .andThen(new AutoMove(m_driveTrainSubsystem, -15, AUTO_DRIVE_SPEED)
      // .withTimeout(0.5))
      // .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, ShooterConstants.SHOOTER_RMP_LOWGOAL)
      // .withTimeout(1.2))
      // .andThen(new RetractIntake(m_intakeSubsystem)
      // .withTimeout(2));

      // case TURN:
      // return new AutoTurn(m_driveTrainSubsystem,
      //     AUTO_TURN_SPEED,
      //     AUTO_TURN_ANGLE_MAX)
      // .andThen(new RetractIntake(m_intakeSubsystem)
      //   .withTimeout(2));

    case THREEBALLAUTO:
    if (position == Position.POSITION4) {
    return new RetractIntake(m_intakeSubsystem)
          .withTimeout(2)
        .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem) //test along with
          .alongWith(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_TO_WALL_BALL, AUTO_DRIVE_SPEED))
          .withTimeout(3)) //test
          // .andThen(new RetractIntake(m_intakeSubsystem)
          //   .withTimeout(2))
          // .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_2, AUTO_DRIVE_SPEED)
          //   .withTimeout(2))
        .andThen(new AutoMove (m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_3_BALL, 0.5) //test
          .withTimeout(2))
        .andThen(new AutoTurn(m_driveTrainSubsystem, AUTO_TURN_SPEED, 8)) // TODO 
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_1))
        .andThen(new WaitCommand(1.7))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
           .withTimeout(SHOOTER_TIMER_2))
            // .andThen(new RetractIntake(m_intakeSubsystem)
            //   .withTimeout(2))
        .andThen(new AutoTurn(m_driveTrainSubsystem,
          AUTO_TURN_SPEED,
          AUTO_TURN_ANGLE_MAX))
         // .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem))
        .andThen(new AutoMove(m_driveTrainSubsystem, 96, 0.6)) //changed speed from 0.8
        .andThen(new WaitCommand(0.5))
        .andThen(new AutoTurn(m_driveTrainSubsystem, AUTO_TURN_SPEED, -30.5)) // was -40, changed to -34
        .andThen(new AutoMove(m_driveTrainSubsystem, -37, 0.8)) //
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(2)
          .alongWith(new RetractIntake(m_intakeSubsystem)));
          }

      case FOURBALLAUTO: //MAKE SURE IN A GOOD SPOT TO TEST
      if (position == Position.POSITION4) {
        return new RetractIntake(m_intakeSubsystem)
            .withTimeout(2)
            // return new DeployIntake(m_intakeSubsystem, m_transitionSubsystem) //test
            // along with
            .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem) // test along with
                .alongWith(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_TO_WALL_BALL, AUTO_DRIVE_SPEED))
                .withTimeout(3))
            // .andThen(new RetractIntake(m_intakeSubsystem)
            // .withTimeout(2))
            // .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_2,
            // AUTO_DRIVE_SPEED)
            // .withTimeout(2))
            .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_3_BALL, 0.5) // test
                .withTimeout(2))
            .andThen(new AutoTurn(m_driveTrainSubsystem, AUTO_TURN_SPEED, 8)) // TODO
            .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
                .withTimeout(SHOOTER_TIMER_1))
            .andThen(new WaitCommand(1.7)) // was 1
            .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
                .withTimeout(SHOOTER_TIMER_2))
            // .andThen(new RetractIntake(m_intakeSubsystem)
            // .withTimeout(2))
            .andThen(new AutoTurn(m_driveTrainSubsystem,
                AUTO_TURN_SPEED,
                AUTO_TURN_ANGLE_MAX))
            // .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem))
            .andThen(new AutoMove(m_driveTrainSubsystem, 96, 0.8))// 
            .andThen(new WaitCommand(0.5))
            .andThen(new AutoTurn(m_driveTrainSubsystem, AUTO_TURN_SPEED, -30.5)) // was -40, changed to -34
            .andThen(new AutoMove(m_driveTrainSubsystem, -37, 0.8)) //
            .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
                .withTimeout(2))
            .andThen(new AutoTurn(m_driveTrainSubsystem, AUTO_TURN_SPEED, 13)) // change 16?
            .andThen(new AutoMove(m_driveTrainSubsystem, 177, 0.75, 0)) // actual distance 177?
            .andThen(new WaitCommand(.75))
            .andThen(new RetractIntake(m_intakeSubsystem));
      }
        
      case DONOTHING:
      return null;

    }
      return null;
    }
  
  // public Command getRetractCommand() {
  //   ClimberRetract m_climberRetract = new ClimberRetract(m_climberSubsystem);
  //   return m_climberRetract;
  // }
  

  public void simulationInit() {
    m_transitionSubsystem.simulationInit();
  }

  public void setNeutralMode(NeutralMode mode) {
    m_driveTrainSubsystem.setNeutralMode(mode);
  }

  //guest mode stuff

  public boolean isDriverJoysticksMoved(){
    // boolean isGuestJoysticksMoved = (m_driveController.getLeftX()!= 0) || (m_driveController.getLeftY()!= 0) || (m_driveController.getRightX()!= 0) || (m_driveController.getRightY()!= 0);
    boolean isDriverJoysticksMoved = 
      (!isClosetoZero(m_driveController.getLeftX())) 
      || (!isClosetoZero(m_driveController.getLeftY())) 
      || (!isClosetoZero(m_driveController.getRightX())) 
      || (!isClosetoZero(m_driveController.getRightY()));
    return isDriverJoysticksMoved; 
  }

  public boolean isClosetoZero(double number){
    boolean isClosetoZero = (number < DriveTrainConstants.DEADBAND && number > -DriveTrainConstants.DEADBAND);
    // System.out.println(number + " is close to 0? " + isClosetoZero);
    return isClosetoZero;
  }


  public static class GuestMode{
    private static boolean isGuestModeEnabled = false;
    private static double guestModeSpeed = GuestModeConstants.GUEST_MODE_MINIMUM_SPEED;

    public double getSpeed () {
      return guestModeSpeed;
    }

    public static void setSpeed(double speed){
      guestModeSpeed = MathUtil.clamp(speed, GuestModeConstants.GUEST_MODE_MINIMUM_SPEED, GuestModeConstants.GUEST_MODE_MAX_SPEED);
    
    }
    public static void increasespeed(){
      guestModeSpeed += (GuestModeConstants.GUEST_MODE_MAX_SPEED-GuestModeConstants.GUEST_MODE_MINIMUM_SPEED) / 4.0;
      guestModeSpeed = MathUtil.clamp(guestModeSpeed,GuestModeConstants.GUEST_MODE_MINIMUM_SPEED, GuestModeConstants.GUEST_MODE_MAX_SPEED);
    }

    public boolean isEnabled () {
      return isGuestModeEnabled;
    }

    public void setGuestMode(boolean enabled){
      if (isEnabled()){
        increasespeed();
      } else {
        guestModeSpeed = GuestModeConstants.GUEST_MODE_MINIMUM_SPEED;
      }
      isGuestModeEnabled = enabled;
    }
  }

}
