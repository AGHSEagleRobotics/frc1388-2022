// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.USBConstants;
import frc.robot.commands.Drive;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // components
  public static XboxController m_driveController = new XboxController(USBConstants.DRIVE_CONTROLLER);
  public static XboxController m_opController = new XboxController(USBConstants.OP_CONTROLLER);

  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
  //THIS is for the 2022 ROBOT  
  // new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT), 
    // new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK), 
    // new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT), 
    // new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK)

    //This is for KNIGHTMARE !
    new WPI_VictorSPX(DriveTrainConstants.CANID_LEFT_FRONT), 
    new WPI_TalonSRX(DriveTrainConstants.CANID_LEFT_BACK), 
    new WPI_TalonSRX(DriveTrainConstants.CANID_RIGHT_FRONT), 
    new WPI_VictorSPX(DriveTrainConstants.CANID_RIGHT_BACK)

    
  );

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // set default commands
    m_driveTrainSubsystem.setDefaultCommand(
      new Drive(
        m_driveTrainSubsystem,
        () -> m_driveController.getLeftY(),
        () -> m_driveController.getRightY(),
        () -> m_driveController.getRightX()
      ) 
    );

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   //return m_autoCommand;
  // }
}
