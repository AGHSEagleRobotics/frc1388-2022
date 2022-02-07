// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.shuffleboard.AxisAction;
import frc.robot.shuffleboard.ButtonAction;
import frc.robot.shuffleboard.ControllerBindings;
import frc.robot.shuffleboard.OISubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final Logger log = LogManager.getLogger(RobotContainer.class);

  private final XboxController m_controller = new XboxController(0);

  private final ControllerBindings<AxisAction, ButtonAction> m_controllerBindings = new ControllerBindings<>(AxisAction.class, ButtonAction.class, new OISubsystem(m_controller));

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem(
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_LEFT_BACK), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_FRONT), 
    new WPI_TalonFX(DriveTrainConstants.CANID_RIGHT_BACK)
  );

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveTrainSubsystem.setDefaultCommand(
      new RunCommand(() -> {
        m_driveTrainSubsystem.tankDrive(
          m_controllerBindings.getAxisValue(AxisAction.LEFT_DRIVE),
          m_controllerBindings.getAxisValue(AxisAction.RIGHT_DRIVE)
        );
      }, m_driveTrainSubsystem)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_controllerBindings.getButton(ButtonAction.DEPLOY_INTAKE).whenPressed(() -> log.info("Deploying intake"));
    m_controllerBindings.getButton(ButtonAction.RETRACT_INTAKE).whenPressed(() -> log.info("Retracting intake"));
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
