// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveTrainConstants;     // climber constats
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.USBConstants;            // USB
import frc.robot.commands.Drive;
import frc.robot.commands.Shoot;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.SetShooterTargetRPM;
import frc.robot.commands.ClimberCommand;           // climber command
import frc.robot.subsystems.ClimberSubsystem;       // climber subsystem
import frc.robot.subsystems.DriveTrainSubsystem;    // drive train subsystem
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // components
  public static XboxController m_driveController = new XboxController(USBConstants.DRIVE_CONTROLLER);
  public static XboxController m_opController = new XboxController(USBConstants.OP_CONTROLLER);

  // The robot's subsystems and commands are defined here...

  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
  //THIS is for the 2022 ROBOT  
  new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK)

    //This is for KNIGHTMARE !
    // new WPI_VictorSPX(DriveTrainConstants.CANID_LEFT_FRONT), 
    // new WPI_TalonSRX(DriveTrainConstants.CANID_LEFT_BACK), 
    // new WPI_TalonSRX(DriveTrainConstants.CANID_RIGHT_FRONT), 
    // new WPI_VictorSPX(DriveTrainConstants.CANID_RIGHT_BACK)
    
  );

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(

    new WPI_TalonFX(ClimberConstants.CANID_WINCH),
    new WPI_TalonSRX(ClimberConstants.CANID_ARTICULATOR)
  );


  private final ShooterFeederSubsystem m_shooterSubsystem = new ShooterFeederSubsystem(
    new WPI_TalonFX(ShooterConstants.CANID_SHOOTER_MOTOR),
    new WPI_VictorSPX(ShooterConstants.CANID_FEEDER_MOTOR)
  );

  private Shoot m_shooterCommands;
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
    new CANSparkMax(IntakeConstants.CANID_WHEEL_MOTOR, MotorType.kBrushless), 
    new CANSparkMax(IntakeConstants.CANID_ARM_MOTOR, MotorType.kBrushless)
    );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   m_shooterCommands = new Shoot(m_shooterSubsystem);

    m_climberSubsystem.setDefaultCommand(
      new ClimberCommand(
        m_climberSubsystem, 
        () -> m_opController.getLeftY(), 
        () -> m_opController.getRightY()
      )
    );
    // set default commands
    m_driveTrainSubsystem.setDefaultCommand(
      new Drive(
        m_driveTrainSubsystem,
        () -> m_driveController.getLeftY(),
        () -> m_driveController.getRightY(),
        () -> m_driveController.getRightX()
      ) 
    );
  // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Reverse

    new JoystickButton(m_driveController, XboxController.Button.kBack.value)
      .whenPressed(() -> m_driveTrainSubsystem.toggleReverse());

    new JoystickButton(m_driveController, XboxController.Button.kX.value)
      .whenPressed(() -> m_shooterSubsystem.shooterRpmStepIncrease());

    new JoystickButton(m_driveController, XboxController.Button.kY.value)
      .whenPressed(() -> m_shooterSubsystem.shooterRpmStepDecrease());

    new JoystickButton(m_driveController, XboxController.Button.kA.value)
      .whenPressed(() -> m_shooterSubsystem.shooterEnabled(true));

    new JoystickButton(m_driveController, XboxController.Button.kB.value)
      .whenPressed(() -> m_shooterSubsystem.shooterEnabled(false));

    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
      .whenHeld(m_shooterCommands);    new JoystickButton(m_opController, XboxController.Button.kA.value)
      .whenPressed(new DeployIntake(m_intakeSubsystem));

      new JoystickButton(m_opController, XboxController.Button.kB.value)
      .whenPressed(new RetractIntake(m_intakeSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
