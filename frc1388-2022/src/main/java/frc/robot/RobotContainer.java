// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControlerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetShooterTargetRPM;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static XboxController driverControler = new XboxController(ControlerConstants.USB_DRIVER_CONTROLER);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK)
  );

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(
    new WPI_TalonFX(ShooterConstants.CANID_SHOOTER_MOTOR)
  );

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    new JoystickButton(driverControler, XboxController.Button.kX.value)
      .whenPressed(new SetShooterTargetRPM(m_ShooterSubsystem, 1000.0));

    new JoystickButton(driverControler, XboxController.Button.kY.value)
      .whenPressed(new SetShooterTargetRPM(m_ShooterSubsystem, 0));

    new JoystickButton(driverControler, XboxController.Button.kA.value)
      .whenPressed(() -> m_ShooterSubsystem.setEnabled(true));

    new JoystickButton(driverControler, XboxController.Button.kB.value)
      .whenPressed(() -> m_ShooterSubsystem.setEnabled(false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
