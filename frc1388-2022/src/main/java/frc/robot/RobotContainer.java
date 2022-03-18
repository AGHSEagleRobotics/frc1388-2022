// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import static frc.robot.Constants.AutoConstants.*;
// import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.Constants.DriveTrainConstants;     // climber constats
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.USBConstants;            // USB
import frc.robot.Constants.XBoxControllerConstants;
// import frc.robot.Constants.ClimberConstants.ArticulatorPositions;
import frc.robot.Constants.DashboardConstants.Cameras;
import frc.robot.Dashboard.Objective;
import frc.robot.Dashboard.Position;
import frc.robot.commands.Drive;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.REject;
import frc.robot.commands.ShootEject;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurn;
// import frc.robot.commands.ClimberCommand;           // climber command
// import frc.robot.subsystems.ClimberSubsystem;       // climber subsystem
import frc.robot.subsystems.DriveTrainSubsystem;    // drive train subsystem
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  // components
  public static XboxController m_driveController = new XboxController(USBConstants.DRIVE_CONTROLLER);
  public static XboxController m_opController = new XboxController(USBConstants.OP_CONTROLLER);

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

  /*
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(
    new WPI_TalonFX(ClimberConstants.CANID_WINCH),
      new CANSparkMax(ClimberConstants.CANID_ARTICULATOR, MotorType.kBrushless));
  */
  
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // m_climberSubsystem.setDefaultCommand(
    //   new ClimberCommand(
    //     m_climberSubsystem, 
    //         () -> m_opController.getLeftY(), // extend
    //     () -> m_opController.getRightY())    // articulate
    //   );
    // set default commands


    m_driveTrainSubsystem.setDefaultCommand(
      new Drive(
            m_driveTrainSubsystem, m_rumbleSubsystem,
        () -> m_driveController.getLeftY(),
        () -> m_driveController.getRightY(),
            () -> m_driveController.getRightX(),
            //rumble for precision mode
            () -> m_driveController.getRightStickButtonPressed()));

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


    new JoystickButton(m_driveController, XboxController.Button.kB.value)
        .whenPressed(() -> m_driveTrainSubsystem.setForward(true));

    /*
     * dev mode
     * new JoystickButton(m_driveController, XboxController.Button.kX.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterRpmStepIncrease());
     * 
     * new JoystickButton(m_driveController, XboxController.Button.kY.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterRpmStepDecrease());
     * 
     * new JoystickButton(m_driveController, XboxController.Button.kA.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterEnabled(true));
     * 
     * new JoystickButton(m_driveController, XboxController.Button.kB.value)
     * .whenPressed(() -> m_shooterSubsystem.shooterEnabled(false));
     * 
     * new JoystickButton(m_opController, XboxController.Button.kRightBumper.value)
      .whileHeld(() -> m_transitionSubsystem.setTransitionSpeed(
            TransitionConstants.TRANSITION_SPEED_REVERSE_SLOW),
            m_transitionSubsystem);
     */


    // INTAKE DEPLOY LEFT TRIGGER
    new Button(() -> isLeftDriverTriggerPressed() || isLeftOpTriggerPressed())
        .whenPressed(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));

    //INTAKE DRIVE UP
    new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value)
        .whenPressed(new RetractIntake(m_intakeSubsystem));

    // INTAKE OP UP
    new JoystickButton(m_opController, XboxController.Button.kLeftBumper.value)
        .whenPressed(new RetractIntake(m_intakeSubsystem));

    // SHOOT LOW HIGH GOAL and EJECT
    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
        .whenHeld(new ShootHigh(m_shooterFeederSubsystem, m_transitionSubsystem));
    
    new Button(RobotContainer::isRightDriverTriggerPressed)
        .whenHeld(new ShootLow(m_shooterFeederSubsystem, m_transitionSubsystem));

    //EJECT AND REJECT commands DRIVER 
    new JoystickButton(m_driveController, XboxController.Button.kBack.value)
      .whenHeld(new ShootEject(m_shooterFeederSubsystem, m_transitionSubsystem));

    new Button (() -> isDriverDPadPressed()).whenHeld(new REject(
      m_intakeSubsystem, m_transitionSubsystem, m_shooterFeederSubsystem));

    new Button (() -> isDriverDPadPressed())
    .whenReleased(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));
  

    //EJECT AND REJECT commands OPERATOR
    new Button (() -> isRightOpTriggerPressed())
    .whenHeld(new ShootEject(m_shooterFeederSubsystem, m_transitionSubsystem));

    new JoystickButton(m_opController, XboxController.Button.kRightBumper.value)
    .whenHeld(new REject(m_intakeSubsystem, m_transitionSubsystem, m_shooterFeederSubsystem));

    new JoystickButton(m_opController, XboxController.Button.kRightBumper.value)
    .whenReleased(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem));



    // Lower priority
    /*
    new JoystickButton(m_opController, XboxController.Button.kX.value)
      .whenPressed(() -> m_climberSubsystem.setArticulatorReach());
      
   new JoystickButton(m_opController, XboxController.Button.kY.value)
      .whenPressed(() -> m_climberSubsystem.setArticulatorVertical());
    */

    // Reverse
    new JoystickButton(m_driveController, XboxController.Button.kB.value)
      .whenPressed(() -> setForward(true));
    
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
      .whenPressed(() -> setForward(false));

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
    new JoystickButton(m_opController, XboxController.Button.kStart.value)
        .whenPressed(
            new InstantCommand(() -> m_dashboard.switchCamera()) {
              @Override
              public boolean runsWhenDisabled() {
                return true;
              }
            });
    // .whenPressed(() -> m_dashboard.switchCamera());
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

  public static boolean isRightDriverTriggerPressed() {
    return m_driveController.getRightTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  public static boolean isLeftDriverTriggerPressed() {
    return m_driveController.getLeftTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  public static boolean isLeftOpTriggerPressed() {
    return m_opController.getLeftTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  public static boolean isDriverDPadPressed() {
    return m_driveController.getPOV() != -1 ;
  }

  //EJECT REJECT FOR OP
  public static boolean isRightOpTriggerPressed() {
    return m_opController.getRightTriggerAxis() > XBoxControllerConstants.TRIGGER_THRESHOLD;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    new PrintCommand("************************\n************************\n getAutonomousCommand() \n\n");

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
    //objective = Objective.MOVESHOOT1;
    switch (objective) {
      case LEAVETARMAC:
      // default:
      return new RetractIntake(m_intakeSubsystem)
        .withTimeout(2)
      .andThen(new AutoMove(m_driveTrainSubsystem, 
          AUTO_TARMAC_DISTANCE,    
          AUTO_DRIVE_SPEED)
        .withTimeout(4)); 

      case MOVEPICKUPSHOOT2:
      if (position == Position.POSITION4) {
        return new RetractIntake(m_intakeSubsystem)
          .withTimeout(2)
        .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem)
          .withTimeout(2))
        .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_1, AUTO_DRIVE_SPEED)
          .withTimeout(3))
        .andThen(new RetractIntake(m_intakeSubsystem)
          .withTimeout(2))
          //change distance going backwards on automove to Auto_Tarmac distance -40
        .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_2, AUTO_DRIVE_SPEED)
          .withTimeout(2))
        .andThen(new AutoMove (m_driveTrainSubsystem, AUTO_POSITION_4_DISTANCE_3, AUTO_DRIVE_SPEED)
          .withTimeout(2))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_1))
        .andThen(new WaitCommand(1))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_2))
        .andThen(new RetractIntake(m_intakeSubsystem)
          .withTimeout(2)
        .andThen(new AutoTurn(m_driveTrainSubsystem,
          AUTO_TURN_SPEED,
          AUTO_TURN_ANGLE_MAX)
          .withTimeout(0.4)));
      } else {
        return new RetractIntake(m_intakeSubsystem)
          .withTimeout(2)
        .andThen(new DeployIntake(m_intakeSubsystem, m_transitionSubsystem)
          .withTimeout(2))
        .andThen(new AutoMove(m_driveTrainSubsystem, 64, AUTO_DRIVE_SPEED)
          .withTimeout(3))
          //change distance going backwards on automove to Auto_Tarmac distance -40
        .andThen(new AutoMove(m_driveTrainSubsystem, -27, AUTO_DRIVE_SPEED)
          .withTimeout(2))
          //If intake works properly, 1.8 or less works for both
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_1))
        .andThen(new WaitCommand(1))
        .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
          .withTimeout(SHOOTER_TIMER_2))
        .andThen(new RetractIntake(m_intakeSubsystem)
          .withTimeout(2));
       }

      case MOVESHOOT1:
      return new RetractIntake(m_intakeSubsystem).withTimeout(2)
      .andThen(new AutoMove(m_driveTrainSubsystem, AUTO_TARMAC_DISTANCE, AUTO_DRIVE_SPEED)
        .withTimeout(3))
      .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, AUTO_SHOOT_RPM)
        .withTimeout(SHOOTER_TIMER_1))
      .andThen( new AutoTurn(m_driveTrainSubsystem,
          AUTO_TURN_SPEED,
          AUTO_TURN_ANGLE_MAX)
        .withTimeout(0.15))
      .andThen(new RetractIntake(m_intakeSubsystem)
        .withTimeout(2));
      
      // case LOWSHOOTMOVE:
      // return new RetractIntake(m_intakeSubsystem).withTimeout(2)
      // .andThen(new AutoMove(m_driveTrainSubsystem, -15, AUTO_DRIVE_SPEED)
      // .withTimeout(0.5))
      // .andThen(new AutoShoot(m_shooterFeederSubsystem, m_transitionSubsystem, ShooterConstants.SHOOTER_RMP_LOWGOAL)
      // .withTimeout(1.2))
      // .andThen(new RetractIntake(m_intakeSubsystem)
      // .withTimeout(2));

      case TURN:
      new AutoTurn(m_driveTrainSubsystem,
          AUTO_TURN_SPEED,
          AUTO_TURN_ANGLE_MAX)
      .andThen(new RetractIntake(m_intakeSubsystem)
        .withTimeout(2));

      case DONOTHING:
      return null;

      /* case SHOOT3!:
       
      */
    }
      return null;
    }
  

  public void simulationInit() {
    m_transitionSubsystem.simulationInit();
  }

  public void setNeutralMode(NeutralMode mode) {
    m_driveTrainSubsystem.setNeutralMode(mode);
  }
}
